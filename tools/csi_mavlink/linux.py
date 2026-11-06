"""Linux devices. CSI timestamps retain buffer semantics, never claim exposure sync."""
import csv
import os
import stat
import queue
import threading
from collections import deque
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

from .core import Frame, png_gray, atomic_json


def check_devices_idle(paths):
    # pyserial 的 flock 不能防止不加锁的其他程序分食同一串口。
    # 尽早检查当前用户可见的已打开设备；内核/串口锁仍负责后续竞争。
    devices = {os.stat(p).st_rdev: p for p in paths}
    for proc in Path('/proc').glob('[0-9]*'):
        if proc.name == str(os.getpid()): continue
        try:
            for fd in (proc/'fd').iterdir():
                try:
                    info = fd.stat()
                    if stat.S_ISCHR(info.st_mode) and info.st_rdev in devices:
                        raise RuntimeError('device already open by PID %s: %s' % (proc.name, devices[info.st_rdev]))
                except (OSError, PermissionError): pass
        except (OSError, PermissionError): pass


class Cameras:
    def __init__(self, config):
        check_devices_idle([c["device"] for c in config])
        import gi
        gi.require_version('Gst', '1.0'); gi.require_version('GstVideo', '1.0')
        from gi.repository import Gst, GstVideo
        Gst.init(None)
        self.Gst, self.GstVideo = Gst, GstVideo
        self.clock = Gst.SystemClock.obtain()
        self.pipelines = []
        self.sinks = []
        self.config = config
        self.counts = [0, 0]
        self.started = False
        try:
            for c in config:
                pipeline = Gst.Pipeline.new(None)
                self.pipelines.append(pipeline)
                pipeline.use_clock(self.clock)
                elements = [Gst.ElementFactory.make(k, None) for k in
                            ('v4l2src', 'capsfilter', 'videoconvert', 'capsfilter', 'appsink')]
                if not all(elements):
                    raise RuntimeError('missing GStreamer v4l2src/videoconvert/appsink plugin')
                src, caps, convert, gray, sink = elements
                src.set_property('device', c['device'])
                src.set_property('do-timestamp', False)
                caps.set_property('caps', Gst.Caps.from_string(
                    'video/x-raw,format=%s,width=%d,height=%d,framerate=%d/1' %
                    (c['format'], c['width'], c['height'], c['fps'])))
                gray.set_property('caps', Gst.Caps.from_string('video/x-raw,format=GRAY8'))
                sink.set_property('sync', False)
                sink.set_property('max-buffers', 4)
                sink.set_property('drop', True)
                sink.set_property('enable-last-sample', False)
                for e in elements: pipeline.add(e)
                for a, b in zip(elements, elements[1:]):
                    if not a.link(b): raise RuntimeError('cannot link camera pipeline')
                self.sinks.append(sink)
        except BaseException:
            self.close(); raise

    def now(self):
        return self.clock.get_time()

    def start(self):
        for p in self.pipelines:
            if p.set_state(self.Gst.State.PLAYING) == self.Gst.StateChangeReturn.FAILURE:
                raise RuntimeError('CSI pipeline failed to start')
        self.started = True

    def poll(self):
        G = self.Gst
        frames = []
        for side, (p, sink, c) in enumerate(zip(self.pipelines, self.sinks, self.config)):
            error = p.get_bus().pop_filtered(G.MessageType.ERROR | G.MessageType.EOS)
            if error:
                if error.type == G.MessageType.ERROR:
                    e, detail = error.parse_error(); raise RuntimeError(str(e) + ': ' + str(detail))
                raise RuntimeError('unexpected CSI end of stream')
            # One buffer per camera per poll. Copying more here stalls USB IMU reads.
            sample = sink.emit('try-pull-sample', 0)
            if sample is None:
                continue
            received = self.now()
            b = sample.get_buffer()
            if b.pts == G.CLOCK_TIME_NONE:
                raise RuntimeError('CSI buffer has no timestamp; refusing arrival-time substitution')
            segment = sample.get_segment()
            running = segment.to_running_time(G.Format.TIME, b.pts)
            if running == G.CLOCK_TIME_NONE:
                raise RuntimeError('invalid CSI buffer segment timestamp')
            stamp = p.get_base_time() + running
            if stamp > received + 10_000_000:
                raise RuntimeError('CSI timestamp lies in future')
            info = self.GstVideo.VideoInfo()
            if not info.from_caps(sample.get_caps()) or (info.width, info.height) != (c['width'], c['height']):
                raise RuntimeError('negotiated camera size differs from configuration')
            # Honor video meta as well as negotiated row padding, notably on Rockchip ISP.
            meta = self.GstVideo.buffer_get_video_meta(b)
            stride = meta.stride[0] if meta else info.stride[0]
            offset = meta.offset[0] if meta else info.offset[0]
            if stride < info.width or offset + (info.height-1)*stride + info.width > b.get_size():
                raise RuntimeError('invalid GRAY8 buffer layout')
            data = b.extract_dup(0, b.get_size())
            row, nbytes = info.width, info.width * info.height
            if stride == row and offset == 0:
                pixels = bytes(memoryview(data)[:nbytes])
            else:
                out = bytearray(nbytes)
                mv = memoryview(data)
                for y in range(info.height):
                    s = offset + y * stride
                    out[y*row:(y+1)*row] = mv[s:s+row]
                pixels = bytes(out)
            driver = None if b.offset == G.BUFFER_OFFSET_NONE else int(b.offset)
            self.counts[side] += 1
            frames.append(Frame(side, stamp, received, self.counts[side], driver, int(b.pts), pixels))
        return frames

    def close(self):
        for p in self.pipelines: p.set_state(self.Gst.State.NULL)
        self.started = False


class SerialMavlink:
    def __init__(self, config):
        check_devices_idle([config["serial"]])
        import serial
        from pymavlink.dialects.v20 import common
        self.protocol = common
        self.config = config
        self.port = serial.Serial(config['serial'], config['baud'], timeout=0, write_timeout=1,
                                  exclusive=True, rtscts=False, dsrdtr=False)
        self.mav = common.MAVLink(self.port, srcSystem=255, srcComponent=190)
        self.mav.robust_parsing = True
        self.sent = {'HEARTBEAT': 0, 'TIMESYNC': 0, 'COMMAND_LONG': 0}

    def heartbeat(self):
        m = self.protocol
        self.mav.heartbeat_send(m.MAV_TYPE_GCS, m.MAV_AUTOPILOT_INVALID, 0, 0, m.MAV_STATE_ACTIVE)
        self.port.flush()
        self.sent['HEARTBEAT'] += 1

    def request_message_interval(self, message_id, interval_us, confirmation=0):
        m = self.protocol
        self.mav.command_long_send(
            self.config['system_id'], self.config['component_id'],
            m.MAV_CMD_SET_MESSAGE_INTERVAL, confirmation,
            float(message_id), float(interval_us), 0., 0., 0., 0., 0.)
        self.port.flush()
        self.sent['COMMAND_LONG'] += 1

    def request_imu_rate(self, interval_us, confirmation=0):
        self.request_message_interval(self.protocol.MAVLINK_MSG_ID_HIGHRES_IMU, interval_us, confirmation)

    def timesync(self, now):
        self.mav.timesync_send(0, now)
        self.sent['TIMESYNC'] += 1

    def read(self):
        chunks = []
        messages = []
        total = 0
        while True:
            waiting = self.port.in_waiting
            if not waiting:
                break
            data = self.port.read(waiting)
            if not data:
                break
            chunks.append(data)
            total += len(data)
            parsed = self.mav.parse_buffer(data)
            if parsed:
                messages.extend(parsed)
            if total >= 262144:
                break
        return b''.join(chunks), messages

    def close(self):
        self.port.close()


class PairWriter:
    """Bounded pair queue; CSV rows are committed only after both PNG writes succeed."""
    def __init__(self, root, config, origin_ns, capacity=8, workers=6, independent=False):
        self.root, self.config, self.origin = Path(root), config, origin_ns
        if not 1 <= workers <= 8:
            raise ValueError("PNG workers must be in [1,8]")
        self.workers = workers
        self.independent = independent
        self.stream_written = [0, 0]
        self.stream_first = [None,None]
        self.stream_last = [None,None]
        self.stream_max_gap_ns = [0,0]
        self.drop_events = []
        self.queue = queue.Queue(capacity)
        self.error = None
        self.written = 0
        self.dropped = 0
        self.first = None
        self.last = None
        self.thread = threading.Thread(target=self._run, name='csi-png-writer')
        self.thread.start()

    def submit(self, pair):
        if self.error: raise RuntimeError('PNG writer failed: ' + str(self.error))
        try: self.queue.put_nowait(pair)
        except queue.Full:
            self.dropped += 1
            frames=[pair] if self.independent else pair
            if len(self.drop_events)<2048:
                self.drop_events.extend(dict(camera=f.camera,buffer_host_ns=f.stamp_ns,
                    sequence=f.sequence,driver_sequence=f.driver_sequence,reason='writer_queue_full') for f in frames)

    def close(self):
        self.queue.put(None)
        self.thread.join()
        if self.error: raise RuntimeError('PNG writer failed: ' + str(self.error))

    def _write_pair(self, pair):
        frames = [pair] if self.independent else list(pair)
        t = frames[0].stamp_ns - self.origin
        filename = str(t) + '.png'
        files = [self.root/'mav0'/('cam'+str(f.camera))/'data'/filename for f in frames]
        try:
            for f, frame in zip(files, frames):
                c = self.config[frame.camera]
                temp = f.with_suffix('.png.tmp')
                with temp.open('wb') as handle:
                    handle.write(png_gray(c['width'], c['height'], frame.pixels))
                    handle.flush()
                    os.fsync(handle.fileno())
                temp.replace(f)
        except BaseException:
            for f in files:
                f.unlink(missing_ok=True)
                f.with_suffix('.png.tmp').unlink(missing_ok=True)
            raise

    def _run(self):
        finished_input = False
        try:
            with (self.root/'mav0/cam0/data.csv').open('w') as a, (self.root/'mav0/cam1/data.csv').open('w') as b, (self.root/('raw/saved-frames.csv' if self.independent else 'raw/pairs.csv')).open('w') as timing:
                rows = [csv.writer(a), csv.writer(b)]
                for row in rows: row.writerow(['#timestamp [ns]', 'filename'])
                raw = csv.writer(timing)
                raw.writerow(['camera','timestamp_ns','buffer_host_ns','receive_host_ns','sequence','driver_sequence','pts_ns'] if self.independent else ['shared_csv_ns', 'left_buffer_ns', 'right_buffer_ns', 'skew_ns',
                              'left_receive_ns', 'right_receive_ns', 'left_sequence', 'right_sequence',
                              'left_driver_sequence', 'right_driver_sequence', 'left_pts_ns', 'right_pts_ns'])
                pending = deque()

                def checkpoint():
                    for stream in (a, b, timing):
                        stream.flush(); os.fsync(stream.fileno())
                    for side in ('cam0', 'cam1'):
                        for directory in (self.root/'mav0'/side, self.root/'mav0'/side/'data'):
                            fd = os.open(str(directory), os.O_RDONLY | os.O_DIRECTORY)
                            try: os.fsync(fd)
                            finally: os.close(fd)
                    atomic_json(self.root/'raw/durable-pairs.json',
                                {'pairs': self.written, 'stream_frames': self.stream_written,
                                 'independent_streams': self.independent, 'last_ns': self.last,
                                 'writer_dropped': self.dropped, 'writer_drop_events':list(self.drop_events)})

                def note_stamp(side,stamp):
                    old=self.stream_last[side]
                    if old is not None:self.stream_max_gap_ns[side]=max(self.stream_max_gap_ns[side],stamp-old)
                    self.stream_first[side]=stamp if self.stream_first[side] is None else self.stream_first[side]
                    self.stream_last[side]=stamp

                def commit_oldest():
                    pair, future = pending.popleft()
                    future.result()
                    if self.independent:
                        f = pair
                        t = f.stamp_ns-self.origin
                        rows[f.camera].writerow([t,str(t)+'.png'])
                        raw.writerow([f.camera,t,f.stamp_ns,f.receive_ns,f.sequence,f.driver_sequence,f.pts_ns])
                        self.stream_written[f.camera] += 1
                        note_stamp(f.camera,t)
                        self.written = min(self.stream_written)
                        self.first = t if self.first is None else min(self.first,t)
                        self.last = t if self.last is None else max(self.last,t)
                        if sum(self.stream_written)%20==0:checkpoint()
                        return
                    left, right = pair
                    for frame in pair:
                        note_stamp(frame.camera,frame.stamp_ns-self.origin)
                        self.stream_written[frame.camera]+=1
                    t = left.stamp_ns-self.origin
                    filename = str(t)+'.png'
                    for row in rows: row.writerow([t, filename])
                    raw.writerow([t, left.stamp_ns, right.stamp_ns, right.stamp_ns-left.stamp_ns,
                                  left.receive_ns, right.receive_ns, left.sequence, right.sequence,
                                  left.driver_sequence, right.driver_sequence, left.pts_ns, right.pts_ns])
                    self.written += 1
                    self.first = t if self.first is None else self.first
                    self.last = t
                    if self.written % 10 == 0: checkpoint()

                # Compression releases the GIL. Bound both in-flight pairs and input queue,
                # then commit CSV in capture order even when encoders finish out of order.
                with ThreadPoolExecutor(max_workers=self.workers, thread_name_prefix='csi-png') as pool:
                    while True:
                        pair = self.queue.get()
                        if pair is None:
                            finished_input = True
                            while pending: commit_oldest()
                            checkpoint()
                            return
                        pending.append((pair, pool.submit(self._write_pair, pair)))
                        if len(pending) >= self.workers:
                            commit_oldest()
        except BaseException as e:
            self.error = e
            # If the sentinel was already consumed, no second sentinel will arrive.
            if not finished_input:
                while self.queue.get() is not None: pass
