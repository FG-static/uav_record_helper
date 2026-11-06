"""CLI orchestration for Linux CSI cameras and flight-controller HIGHRES_IMU."""
import argparse
from contextlib import ExitStack
from collections import deque
import csv
from concurrent.futures import ThreadPoolExecutor
import json
import math
import os
from pathlib import Path
import re
import signal
import sys
import time

from .core import (atomic_json, load_config, ClockMapping, Pairer, ImuDecoder,
                   HighresRequest, export_calibration, CALIB_SILENCE_MSGIDS)


def clock_reply(mapping, remote_ns, echoed_ns, received_ns, log=None, controller=None):
    """Preserve rejected exchanges for offline fitting without stopping raw capture."""
    try:
        result = mapping.reply(remote_ns, echoed_ns, received_ns)
    except ValueError as exc:
        if not getattr(controller, 'advisory_quality', False):
            raise
        if log:
            log.write(json.dumps(dict(remote_ns=remote_ns, echoed_ns=echoed_ns,
                receive_ns=received_ns, model_update_applied=False, error=str(exc)))+'\n')
        controller.warn('软件时钟估计异常；保留上次有效映射及原始同步交换，供离线检查。')
        return None
    if result and log:
        log.write(json.dumps(dict(result, remote_ns=remote_ns, echoed_ns=echoed_ns,
                                  receive_ns=received_ns))+'\n')
    return result


def run_session(args, config, controller=None):
    from .linux import Cameras, SerialMavlink, PairWriter
    recording = args.command == 'record'
    advisory_quality = getattr(controller, 'advisory_quality', False)
    root = None
    manifest = {'schema_version': 1, 'backend': 'csi-mavlink', 'complete': False,
                'replay_ready': False, 'config': config, 'calibration': None,
                'timestamp_reference': 'GStreamer running-time + pipeline base-time; driver buffer, not verified exposure',
                'pairing': 'software in-order within tolerance; cam0 is shared CSV key',
                'shutter': 'rolling', 'hardware_sync_verified': False,
                'imu_frame': 'FLU', 'imu_units': 'rad/s and specific force m/s^2',
                'limitations': ['rolling shutter is not modelled by current unav_vio',
                                'software pairing is not hardware exposure synchronization',
                                'TIMESYNC is an affine software clock estimate, not sensor hardware sync',
                                'compare actual imu_max_gap_ns against current unav_vio default max_imu_gap=10ms'],
                'started_unix_ns': time.time_ns()}
    if recording:
        if not args.seq or not re.fullmatch(r'[A-Za-z0-9][A-Za-z0-9_.-]*', args.seq):
            raise ValueError('--seq must be a nonempty simple sequence name')
        root = Path(args.out).expanduser().resolve()/args.seq
        root.parent.mkdir(parents=True, exist_ok=True)
        root.mkdir()  # Never overwrite an existing sequence, including an incomplete one.
        for d in ['raw', 'mav0/imu0', 'mav0/cam0/data', 'mav0/cam1/data']:
            (root/d).mkdir(parents=True, exist_ok=True)
        for directory in [root.parent,root,*[p for p in root.rglob('*') if p.is_dir()]]:
            fd=os.open(str(directory),os.O_RDONLY|getattr(os,'O_DIRECTORY',0))
            try:os.fsync(fd)
            finally:os.close(fd)
        contract='independent_async_rolling_shutter' if controller else 'soft_paired_rolling_shutter'
        (root/'record_summary.yaml').write_text('recorder_backend: csi-mavlink\ncapture_contract: '+contract+'\nreplay_ready: false\n')
        atomic_json(root/'capture_manifest.json', manifest)
    cameras = link = writer = None
    stop = False
    def stop_signal(*_):
        nonlocal stop
        stop = True
    saved = {s: signal.signal(s, stop_signal) for s in (signal.SIGINT, signal.SIGTERM)}
    rate_request = HighresRequest(config['mavlink'].get('imu_rate_hz', 200))
    mapping = ClockMapping()
    decoder = ImuDecoder(config['mavlink'])
    pairer = Pairer(round(config['pair_tolerance_ms']*1e6))
    stats = {'imu_rows': 0, 'imu_max_gap_ns': 0, 'imu_first_ns': None, 'imu_last_ns': None,
             'pairs_observed': 0, 'camera_frames': [0, 0], 'driver_sequence_gaps': [0, 0],
             'timesync_replies': 0, 'raw_bytes': 0}
    silence_acks = []
    error = None
    outputs = []
    try:
        with ExitStack() as stack:
            cameras = Cameras(config['cameras'])
            origin = cameras.now()
            manifest['origin_host_clock_ns'] = origin
            manifest['clock'] = 'GstSystemClock monotonic nanoseconds, common to both pipelines and TIMESYNC'
            link = SerialMavlink(config['mavlink'])
            # The first transmitted message is always heartbeat, before the first read.
            link.heartbeat()
            raw_imu = canonical = frames_log = timesync_log = ack_log = rx = tx = None
            if recording:
                def output(name, mode='w'):
                    handle = stack.enter_context((root/name).open(mode))
                    outputs.append(handle)
                    return handle
                raw_imu = csv.writer(output('raw/imu.csv'))
                raw_imu.writerow(['device_boot_us', 'receive_host_ns', 'system_id', 'component_id', 'sensor_id',
                                  'fields_updated', 'gx', 'gy', 'gz', 'ax', 'ay', 'az', 'accepted'])
                canonical = csv.writer(output('mav0/imu0/data.csv'))
                canonical.writerow(['#timestamp [ns]', 'w_RS_S_x [rad s^-1]', 'w_RS_S_y [rad s^-1]',
                                     'w_RS_S_z [rad s^-1]', 'a_RS_S_x [m s^-2]', 'a_RS_S_y [m s^-2]', 'a_RS_S_z [m s^-2]'])
                frames_log = csv.writer(output('raw/cameras.csv'))
                frames_log.writerow(['camera', 'buffer_host_ns', 'receive_host_ns', 'sequence', 'driver_sequence', 'pts_ns'])
                timesync_log = output('raw/timesync.jsonl')
                ack_log = output('raw/command-acks.jsonl')
                rx, tx = output('raw/mavlink-rx.bin', 'wb'), output('raw/mavlink-tx.jsonl')
                tx.write(json.dumps({'type': 'HEARTBEAT', 'host_ns': origin})+'\n')
                writer = PairWriter(root, config['cameras'], origin,
                                    capacity=32 if controller else 8,
                                    independent=controller is not None)
                if controller: controller.writer = writer
            start = cameras.now()
            next_hb, next_sync = start+1_000_000_000, start
            active = None
            tail = None
            last_imu = start
            last_camera = [start, start]
            last_driver = [None, None]
            last_progress = start
            recent_imu_gaps = deque(maxlen=2000)
            last_checkpoint = start
            capture_start = None
            checkpoints = stack.enter_context(ThreadPoolExecutor(max_workers=1, thread_name_prefix='capture-sync'))
            checkpoint_future = None
            if controller: controller.setup(root, manifest)
            silence_sent = False
            silence_done = False
            silence_deadline = None
            silence_acks = []
            def ingest():
                nonlocal last_imu
                data, messages = link.read()
                received = cameras.now()
                stats['raw_bytes'] += len(data)
                if rx: rx.write(data)
                for m in messages:
                    if m.get_srcSystem() != config['mavlink']['system_id'] or m.get_srcComponent() != config['mavlink']['component_id']:
                        continue
                    if m.get_type() == 'COMMAND_ACK' and m.command == 511:
                        if getattr(m, 'target_system', 0) not in (0, 255) or getattr(m, 'target_component', 0) not in (0, 190):
                            continue
                        # Log rejection before handling it so failed captures retain the response.
                        if ack_log: ack_log.write(json.dumps({'command': m.command, 'result': m.result,
                            'progress': getattr(m, 'progress', None), 'receive_ns': received,
                            'silence_phase': not silence_done})+'\n')
                        if not silence_done:
                            silence_acks.append(int(m.result))
                            continue
                        rate_request.acknowledge(m.command, m.result, received)
                    elif m.get_type() == 'TIMESYNC' and m.tc1 > 0:
                        result = clock_reply(mapping, m.tc1, m.ts1, received, timesync_log, controller)
                        if result:
                            stats['timesync_replies'] += 1
                    elif m.get_type() == 'HIGHRES_IMU':
                        last_imu = received
                        row = decoder.accept(m, mapping, received)
                        # Mapped device time stays valid if USB read was late; do not drop on receive lag.
                        valid = rate_request.accepted and row is not None and row[0] > origin
                        if raw_imu:
                            values = [getattr(m, k) for k in ('xgyro', 'ygyro', 'zgyro', 'xacc', 'yacc', 'zacc')]
                            raw_imu.writerow([m.time_usec, received, m.get_srcSystem(), m.get_srcComponent(),
                                              getattr(m, 'id', 0), m.fields_updated, *values, int(valid)])
                        if valid:
                            t = row[0]-origin
                            if canonical: canonical.writerow([t, *[format(v, '.17g') for v in row[1]]])
                            if stats['imu_last_ns'] is not None:
                                stats['imu_max_gap_ns'] = max(stats['imu_max_gap_ns'], t-stats['imu_last_ns'])
                                recent_imu_gaps.append((received,t-stats['imu_last_ns']))
                            stats['imu_first_ns'] = t if stats['imu_first_ns'] is None else stats['imu_first_ns']
                            stats['imu_last_ns'] = t
                            stats['imu_rows'] += 1
            while True:
                now = cameras.now()
                if now >= next_hb:
                    link.heartbeat(); next_hb = now+1_000_000_000
                    if tx: tx.write(json.dumps({'type': 'HEARTBEAT', 'host_ns': now})+'\n')
                if not silence_sent and link.sent.get('HEARTBEAT'):
                    for msgid in CALIB_SILENCE_MSGIDS:
                        link.request_message_interval(msgid, -1)
                        if tx: tx.write(json.dumps({'type': 'COMMAND_LONG', 'command': 511,
                            'message_id': msgid, 'interval_us': -1, 'host_ns': now})+'\n')
                    silence_sent = True
                    silence_deadline = now + 2_000_000_000
                    print('silencing %d extra MAVLink streams on this USB instance' % len(CALIB_SILENCE_MSGIDS), flush=True)
                if silence_sent and not silence_done and (len(silence_acks) >= len(CALIB_SILENCE_MSGIDS) or now >= silence_deadline):
                    silence_done = True
                if silence_done and rate_request.due(now):
                    link.request_imu_rate(rate_request.interval_us, rate_request.attempts-1)
                    if tx: tx.write(json.dumps(dict(rate_request.snapshot(), type='COMMAND_LONG', host_ns=now))+'\n')
                if now >= next_sync:
                    mapping.request(now); link.timesync(now); next_sync = now+100_000_000
                    if tx: tx.write(json.dumps({'type': 'TIMESYNC', 'tc1': 0, 'ts1': now})+'\n')
                ingest()
                if mapping.max_drift_ns > 2_000_000:
                    if advisory_quality:
                        controller.warn('软件时钟估计残差超过2ms；继续保存，离线检查同步精度。')
                    else:
                        raise RuntimeError('TIMESYNC affine model residual >2ms; recording stopped')
                if active is None and rate_request.accepted and mapping.offset_ns is not None and stats['imu_rows'] >= 5 and stats['imu_last_ns']-stats['imu_first_ns'] >= 100_000_000:
                    cameras.start(); active = now; last_camera = [now, now]
                    print('CSI stream active; waiting for start' if controller else
                          ('CSI recording active' if recording else 'CSI probe active'), flush=True)
                if cameras.started:
                    for f in cameras.poll():
                        if controller: controller.frame(f)
                        last_camera[f.camera] = f.receive_ns
                        stats['camera_frames'][f.camera] += 1
                        prior = last_driver[f.camera]
                        if f.driver_sequence is not None:
                            if prior is not None and f.driver_sequence > prior:
                                stats['driver_sequence_gaps'][f.camera] += max(0, f.driver_sequence-prior-1)
                            last_driver[f.camera] = f.driver_sequence
                        if frames_log: frames_log.writerow([f.camera, f.stamp_ns, f.receive_ns, f.sequence, f.driver_sequence, f.pts_ns])
                        if advisory_quality:
                            continue  # Independent capture has no stereo matching requirement.
                        for pair in pairer.push(f):
                            if pair[0].stamp_ns <= origin: continue
                            stats['pairs_observed'] += 1
                            if controller: controller.pair(pair)
                            if writer and (controller is None or controller.accept_pair(pair)):
                                writer.submit(pair)
                    ingest()
                if controller and active is not None and tail is None:
                    while recent_imu_gaps and now-recent_imu_gaps[0][0]>4_000_000_000:recent_imu_gaps.popleft()
                    stats['imu_window_max_gap_ns']=max((gap for _,gap in recent_imu_gaps),default=0)
                    controller.tick(now, stats, writer, mapping)
                    if controller.recording and capture_start is None: capture_start = now
                    if controller.stop_requested: stop = True
                if writer and writer.error: raise RuntimeError(str(writer.error))
                if active is None and now-start > 15_000_000_000:
                    raise RuntimeError('no accepted interval ACK and aligned HIGHRES_IMU within 15s; no arrival-time fallback')
                duration_start = capture_start if controller else active
                if active is not None and tail is None and (stop or (args.duration and duration_start is not None and now-duration_start >= args.duration*1e9)):
                    cameras.close(); tail = now
                if stop and active is None: raise RuntimeError('cancelled during synchronization')
                if tail is not None and now-tail >= 250_000_000: break
                if active is not None and tail is None and (now-last_imu > 3_000_000_000 or any(now-t > 3_000_000_000 for t in last_camera)):
                    raise RuntimeError('sensor stream stalled for >3s (imu_age_ms=%.0f cam_age_ms=%s)' %
                                       ((now-last_imu)/1e6, [round((now-t)/1e6) for t in last_camera]))
                if now-last_progress >= 2_000_000_000:
                    if controller:
                        controller.progress(now,stats,writer,mapping)
                    else:
                        print('pairs=%d IMU=%d offset=%s queue_drop=%d' %
                              (stats['pairs_observed'], stats['imu_rows'], mapping.offset_ns, writer.dropped if writer else 0), flush=True)
                    last_progress = now
                if checkpoint_future and checkpoint_future.done(): checkpoint_future.result()
                if recording and now-last_checkpoint >= 1_000_000_000 and (checkpoint_future is None or checkpoint_future.done()):
                    snapshot = dict(manifest, stats=dict(stats), checkpoint_unix_ns=time.time_ns(), complete=False)
                    def checkpoint(snapshot=snapshot):
                        for handle in outputs:
                            handle.flush(); os.fsync(handle.fileno())
                        for directory in {Path(h.name).parent for h in outputs}:
                            fd=os.open(str(directory),os.O_RDONLY|getattr(os,'O_DIRECTORY',0))
                            try:os.fsync(fd)
                            finally:os.close(fd)
                        atomic_json(root/'capture_manifest.json', snapshot)
                    checkpoint_future = checkpoints.submit(checkpoint)
                    last_checkpoint = now
                port = getattr(link, 'port', None)
                if not port or not getattr(port, 'in_waiting', 0):
                    time.sleep(.001)
            if writer:
                writer.close()
                if writer.written < 2 or stats['imu_rows'] < 2:
                    raise RuntimeError('insufficient image/IMU data')
                if stats['imu_first_ns'] > writer.first-55_000_000 or stats['imu_last_ns'] < writer.last:
                    if advisory_quality:
                        controller.warn('IMU未覆盖全部图像时段及55ms前置余量；数据已保留，离线检查有效重叠区间。')
                    else:
                        raise RuntimeError('IMU does not cover image interval plus 55ms leading margin')
            if checkpoint_future: checkpoint_future.result()
            for handle in outputs:
                handle.flush(); os.fsync(handle.fileno())
            if controller: controller.finish(stats, writer)
    except BaseException as e:
        error = '%s: %s' % (type(e).__name__, e)
    finally:
        if controller:
            try: controller.close()
            except Exception as e: error = error or str(e)
        if cameras: cameras.close()
        if link: link.close()
        if writer and writer.thread.is_alive():
            try: writer.close()
            except Exception as e: error = error or str(e)
        for s, handler in saved.items(): signal.signal(s, handler)
        stats.update({'highres_imu_request': rate_request.snapshot(),
                      'mavlink_silenced_msgids': list(CALIB_SILENCE_MSGIDS),
                      'mavlink_silence_acks': silence_acks,
                      'unpaired_dropped': pairer.dropped, 'max_pair_skew_ns': pairer.max_skew_ns,
                      'imu_rejected': decoder.rejected, 'clock_offset_ns': mapping.offset_ns,
                      'clock_max_drift_ns': mapping.max_drift_ns,
                      'clock_rate_scale': mapping.scale, 'clock_mapping_updates': mapping.updates,
                      'parser_errors': link.mav.total_receive_errors if link else None,
                      'transmitted': link.sent if link else {},
                      'pairs_written': writer.written if writer else 0,
                      'pair_queue_dropped': writer.dropped if writer else 0,
                      'png_workers': writer.workers if writer else None,
                      'png_queue_capacity_pairs': writer.queue.maxsize if writer else None,
                      'writer_queue_capacity_units': 'frames' if controller else 'pairs',
                      'camera_saved_counts': writer.stream_written if writer else None,
                      'camera_max_saved_gap_ns': writer.stream_max_gap_ns if writer else None,
                      'writer_drop_events': writer.drop_events if writer else [],
                      'first_pair_ns': writer.first if writer else None,
                      'last_pair_ns': writer.last if writer else None})
        count, first, last = stats['imu_rows'], stats['imu_first_ns'], stats['imu_last_ns']
        stats['imu_rate_hz'] = (count-1)*1e9/(last-first) if count > 1 and last > first else None
        stats['imu_rate_fraction_of_requested'] = stats['imu_rate_hz']/rate_request.rate_hz if stats['imu_rate_hz'] else None
        if stats['imu_rate_hz'] and stats['imu_rate_hz'] < rate_request.rate_hz*.8:
            manifest['limitations'].append('observed IMU rate below 80% of requested rate; inspect ACK, link and source gaps')
        if writer and writer.written > 1:
            stats['written_pair_rate_hz'] = (writer.written-1)*1e9/(writer.last-writer.first)
        manifest.update(stats=stats, complete=error is None, error=error, ended_unix_ns=time.time_ns())
        if controller: manifest['calibration_capture'] = controller.summary()
        if recording:
            atomic_json(root/'capture_manifest.json', manifest)
            if error is None and args.calibration:
                try: manifest = export_calibration(root, args.calibration)
                except Exception as e:
                    # Capture remains complete; calibration export failure is explicitly separate.
                    manifest['calibration_error'] = str(e); atomic_json(root/'capture_manifest.json', manifest)
                    error = 'capture saved, calibration export failed: '+str(e)
        if controller:
            print('[已结束] '+('数据保存完成。' if error is None else '本次数据未通过完成检查。'),flush=True)
            print('数据与详细状态: %s' % root,flush=True)
        else:
            print(json.dumps(manifest, indent=2), flush=True)
    if error: raise RuntimeError(error)
    if recording:
        print('Saved %s (research capture; replay_ready=false)' % root, flush=True)
    return manifest


def main(argv=None):
    p = argparse.ArgumentParser(description='CSI rolling stereo + MAVLink HIGHRES_IMU research recorder')
    sub = p.add_subparsers(dest='command', required=True)
    for name in ('record', 'probe'):
        s = sub.add_parser(name)
        s.add_argument('--backend', choices=['csi-mavlink'], default='csi-mavlink')
        s.add_argument('--config', required=True)
        s.add_argument('--duration', type=float, default=0 if name == 'record' else 5)
        if name == 'record':
            s.add_argument('--out', default='datasets')
            s.add_argument('--seq', required=True)
            s.add_argument('--calibration', help='directory containing cam0/cam1/imu0 sensor.yaml, FLU body')
    e = sub.add_parser('export-csi')
    e.add_argument('dataset'); e.add_argument('--calibration', required=True)
    args = p.parse_args(argv)
    try:
        if args.command == 'export-csi':
            export_calibration(args.dataset, args.calibration)
            print('Calibration exported; capture remains software-paired rolling shutter, replay_ready=false')
        else:
            if not math.isfinite(args.duration) or args.duration < 0 or (args.command == 'probe' and args.duration <= 0):
                raise ValueError('--duration must be finite and nonnegative (probe requires >0)')
            if sys.platform != 'linux': raise RuntimeError('CSI backend requires Linux V4L2/GStreamer')
            run_session(args, load_config(args.config))
        return 0
    except (Exception, KeyboardInterrupt) as e:
        print('CSI/MAVLink: '+str(e), file=sys.stderr)
        return 1
