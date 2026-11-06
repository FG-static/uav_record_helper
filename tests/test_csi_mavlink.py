import contextlib
import csv
import io
import json
from pathlib import Path
import struct
import tempfile
import time
import types
import unittest
from unittest.mock import patch
import zlib
import queue

from csi_mavlink.core import (ClockMapping, Frame, ImuDecoder, Pairer, png_gray,
                              load_config, export_calibration, validate_calibration, HighresRequest,
                              CALIB_SILENCE_MSGIDS, CALIB_KEEP_MSGIDS)
from csi_mavlink.linux import PairWriter

ROOT = Path(__file__).resolve().parents[1]


def config():
    return json.loads((ROOT/'config/rk3588-csi-mavlink.json').read_text())


class Message:
    time_usec = 1000000
    fields_updated = 63
    id = 0
    xgyro, ygyro, zgyro, xacc, yacc, zacc = 1., 2., 3., 4., 5., 6.
    def get_srcSystem(self): return 1
    def get_srcComponent(self): return 1


def frame(side, stamp, seq=1):
    return Frame(side, stamp, stamp+10, seq, seq, stamp, b'1234')


class Contracts(unittest.TestCase):
    def test_writer_keeps_gap_metrics_and_exact_queue_drop_event(self):
        with tempfile.TemporaryDirectory() as d:
            root=Path(d)
            for folder in ('raw','mav0/cam0/data','mav0/cam1/data'):(root/folder).mkdir(parents=True)
            w=PairWriter(root,[dict(width=2,height=2)]*2,0,independent=True)
            with patch.object(w.queue,'put_nowait',side_effect=queue.Full):w.submit(frame(1,90,7))
            for t in (100,250):
                w.submit(frame(0,t));w.submit(frame(1,t+14))
            w.close()
            self.assertEqual(w.stream_max_gap_ns,[150,150])
            self.assertEqual(w.stream_written,[2,2])
            self.assertEqual(w.dropped,1)
            event=json.loads((root/'raw/durable-pairs.json').read_text())['writer_drop_events'][0]
            self.assertEqual((event['camera'],event['buffer_host_ns'],event['sequence']),(1,90,7))

    def test_clock_rate_tracking_over_two_minutes(self):
        mapping=ClockMapping()
        truth_scale=.9996
        mapped=[]
        for k in range(1,601):
            remote=k*200_000_000
            midpoint=round(remote*truth_scale)+5_000_000_000
            half_rtt=80_000+(k%7)*3000
            sent=midpoint-half_rtt
            mapping.request(sent)
            mapping.reply(remote,sent,midpoint+half_rtt)
            if mapping.offset_ns is not None:
                value=mapping.map(remote)
                mapped.append(value)
                if k>40:self.assertLess(abs(value-midpoint),500_000)
        self.assertTrue(all(b>a for a,b in zip(mapped,mapped[1:])))
        self.assertAlmostEqual(mapping.scale,truth_scale,delta=1e-7)
        self.assertLess(mapping.max_drift_ns,2_000_000)

    def test_independent_writer_commits_distinct_times_and_durable_prefix(self):
        with tempfile.TemporaryDirectory() as d:
            root=Path(d)
            for folder in ('raw','mav0/cam0/data','mav0/cam1/data'):(root/folder).mkdir(parents=True)
            cfg=[dict(width=2,height=2),dict(width=2,height=2)]
            writer=PairWriter(root,cfg,0,independent=True)
            writer.submit(frame(0,100));writer.submit(frame(1,114));writer.close()
            self.assertEqual(writer.stream_written,[1,1])
            self.assertIn('100,100.png',(root/'mav0/cam0/data.csv').read_text())
            self.assertIn('114,114.png',(root/'mav0/cam1/data.csv').read_text())
            journal=json.loads((root/'raw/durable-pairs.json').read_text())
            self.assertEqual(journal['stream_frames'],[1,1])
            self.assertTrue(journal['independent_streams'])

    def test_config_strict(self):
        with tempfile.TemporaryDirectory() as d:
            p = Path(d)/'c.json'; c = config(); p.write_text(json.dumps(c))
            self.assertEqual(load_config(p), c)
            for change in [('pair_tolerance_ms', float('nan')), ('extra', 1)]:
                bad = dict(c); bad[change[0]] = change[1]; p.write_text(json.dumps(bad))
                with self.assertRaises(ValueError): load_config(p)
            c['cameras'][1]['device'] = c['cameras'][0]['device']; p.write_text(json.dumps(c))
            with self.assertRaises(ValueError): load_config(p)

    def test_calib_silence_keeps_imu_timesync_heartbeat(self):
        self.assertFalse(set(CALIB_SILENCE_MSGIDS) & set(CALIB_KEEP_MSGIDS))
        self.assertEqual(len(CALIB_SILENCE_MSGIDS), len(set(CALIB_SILENCE_MSGIDS)))
        self.assertIn(331, CALIB_SILENCE_MSGIDS)
        self.assertNotIn(105, CALIB_SILENCE_MSGIDS)
        self.assertNotIn(111, CALIB_SILENCE_MSGIDS)

    def test_highres_rate_config_and_legacy_default(self):
        with tempfile.TemporaryDirectory() as d:
            p=Path(d)/'c.json'; c=config(); c['mavlink'].pop('imu_rate_hz')
            p.write_text(json.dumps(c));self.assertEqual(load_config(p)['mavlink']['imu_rate_hz'],200)
            for rate in (0,1001,True,200.5):
                c['mavlink']['imu_rate_hz']=rate;p.write_text(json.dumps(c))
                with self.assertRaises(ValueError):load_config(p)

    def test_highres_interval_ack_retry_and_failure(self):
        r=HighresRequest(200)
        self.assertEqual(r.interval_us,5000);self.assertTrue(r.due(0))
        self.assertFalse(r.acknowledge(400,0,100))
        self.assertFalse(r.due(999_999_999));self.assertTrue(r.due(1_000_000_000))
        self.assertTrue(r.due(2_000_000_000))
        with self.assertRaises(RuntimeError):r.due(3_000_000_000)
        r=HighresRequest();r.due(0);r.acknowledge(511,1,100)
        self.assertFalse(r.due(1000));self.assertTrue(r.due(1_000_000_100))
        r.acknowledge(511,0,1_000_000_200)
        self.assertTrue(r.accepted);self.assertFalse(r.due(10_000_000_000))
        r=HighresRequest();r.due(0);r.acknowledge(511,5,100)
        with self.assertRaises(RuntimeError):r.due(5_000_000_100)
        for result in (2,3,4,6):
            r=HighresRequest();r.due(0)
            with self.assertRaises(RuntimeError):r.acknowledge(511,result,100)

    def test_highres_command_wire_with_real_pymavlink(self):
        try:from pymavlink.dialects.v20 import common as mav
        except ImportError:self.skipTest('pymavlink not installed')
        from csi_mavlink.linux import SerialMavlink
        link=SerialMavlink.__new__(SerialMavlink)
        link.protocol=mav;link.config=config()['mavlink'];link.port=io.BytesIO()
        link.mav=mav.MAVLink(link.port,srcSystem=255,srcComponent=190)
        link.sent={'HEARTBEAT':0,'TIMESYNC':0,'COMMAND_LONG':0}
        link.heartbeat();link.request_imu_rate(5000)
        messages=mav.MAVLink(None).parse_buffer(link.port.getvalue())
        self.assertEqual([m.get_type() for m in messages],['HEARTBEAT','COMMAND_LONG'])
        cmd=messages[1]
        self.assertEqual((cmd.target_system,cmd.target_component,cmd.command,cmd.confirmation),(1,1,511,0))
        self.assertEqual((cmd.param1,cmd.param2),(105.,5000.))
        self.assertEqual([cmd.param3,cmd.param4,cmd.param5,cmd.param6,cmd.param7],[0.]*5)
        link.port.seek(0); link.port.truncate()
        link.request_message_interval(331, -1)
        stop=mav.MAVLink(None).parse_buffer(link.port.getvalue())[0]
        self.assertEqual((stop.command,stop.param1,stop.param2),(511,331.,-1.))

    def test_serial_read_drains_beyond_16kb(self):
        from csi_mavlink.linux import SerialMavlink
        class Port:
            def __init__(self, payload):
                self.payload = payload
            @property
            def in_waiting(self):
                return min(4096, len(self.payload))
            def read(self, n):
                data, self.payload = self.payload[:n], self.payload[n:]
                return data
        link = SerialMavlink.__new__(SerialMavlink)
        link.port = Port(b'x'*40000)
        seen = []
        link.mav = types.SimpleNamespace(parse_buffer=lambda d: seen.append(len(d)) or None)
        data, messages = link.read()
        self.assertEqual(len(data), 40000)
        self.assertEqual(sum(seen), 40000)
        self.assertGreater(len(seen), 1)
        self.assertEqual(messages, [])

    def test_timesync_matching_rtt_and_fixed_offset(self):
        c = ClockMapping(min_samples=3)
        self.assertIsNone(c.reply(10, 5, 20))
        for t in [100000000, 200000000, 300000000]:
            c.request(t)
            self.assertIsNotNone(c.reply(t-999000, t, t+2000))
        self.assertEqual(c.offset_ns, 1000000)
        c.request(400000000)
        self.assertIsNone(c.reply(1, 400000000, 430000000))
        c.request(500000000); c.reply(498001000, 500000000, 500002000)
        self.assertEqual(c.offset_ns, 1000000)
        self.assertIsNone(c.reply(1, 500000000, 500002000))

    def test_pairing_drops_early_retains_timestamps_and_is_bounded(self):
        p = Pairer(3, capacity=2)
        self.assertEqual(p.push(frame(0, 10)), [])
        pair = p.push(frame(1, 12))[0]
        self.assertEqual((pair[0].stamp_ns, pair[1].stamp_ns), (10, 12))
        p.push(frame(0, 20)); p.push(frame(0, 30)); p.push(frame(0, 40))
        self.assertEqual(len(p.queues[0]), 2)
        self.assertEqual(p.dropped, [1, 0])
        pair = p.push(frame(1, 41))[0]
        self.assertEqual(pair[0].stamp_ns, 40)
        self.assertEqual(p.dropped, [2, 0])
        with self.assertRaises(ValueError): p.push(frame(1, 41))

    def test_imu_source_units_frame_mask_duplicates_and_reboot(self):
        d = ImuDecoder(config()['mavlink']); m = Message()
        m.id = 2; self.assertIsNone(d.accept(m, 1000, 1000002000)); m.id = 0
        m.fields_updated = 31; self.assertIsNone(d.accept(m, 1000, 1000002000)); m.fields_updated = 63
        stamp, values = d.accept(m, 1000, 1000002000)
        self.assertEqual(stamp, 1000001000)
        self.assertEqual(values, [1., -2., -3., 4., -5., -6.])
        self.assertIsNone(d.accept(m, 1000, 1000002000))
        m.time_usec -= 1
        with self.assertRaises(ValueError): d.accept(m, 1000, 1000002000)
        m = Message(); m.xacc = float('nan')
        self.assertIsNone(ImuDecoder(config()['mavlink']).accept(m, 0, 2000000000))

    def test_png_is_8bit_lossless_and_wrong_size_rejected(self):
        data = png_gray(2, 2, b'1234')
        self.assertEqual(data[:8], b'\x89PNG\r\n\x1a\n')
        pos = 8; compressed = b''
        while pos < len(data):
            n, = struct.unpack('>I', data[pos:pos+4]); kind = data[pos+4:pos+8]
            payload = data[pos+8:pos+8+n]
            self.assertEqual(struct.unpack('>I', data[pos+8+n:pos+12+n])[0], zlib.crc32(kind+payload))
            if kind == b'IDAT': compressed += payload
            pos += n+12
        self.assertEqual(zlib.decompress(compressed), b'\0' + b'12' + b'\0' + b'34')
        with self.assertRaises(ValueError): png_gray(2, 2, b'bad')

    def test_pair_writer_success_and_disk_payload_failure(self):
        for bad in (False, True):
            with tempfile.TemporaryDirectory() as d:
                root = Path(d)
                for path in ['raw', 'mav0/cam0/data', 'mav0/cam1/data']: (root/path).mkdir(parents=True)
                c = [{'width': 2, 'height': 2}]*2
                w = PairWriter(root, c, 100)
                a, b = frame(0, 110), frame(1, 112)
                if bad: b.pixels = b'bad'
                w.submit((a, b))
                if bad:
                    with self.assertRaises(RuntimeError): w.close()
                else: w.close()
                rows = list(csv.reader(io.StringIO((root/'mav0/cam0/data.csv').read_text())))
                self.assertEqual(len(rows), 1 if bad else 2)
                self.assertEqual(w.written, 0 if bad else 1)
                self.assertEqual(len(list(root.glob('mav0/*/data/*.png'))), 0 if bad else 2)
                self.assertFalse(w.thread.is_alive())

    def test_parallel_writer_preserves_csv_order_and_closes_on_open_error(self):
        with tempfile.TemporaryDirectory() as d:
            root = Path(d)
            for path in ['raw', 'mav0/cam0/data', 'mav0/cam1/data']: (root/path).mkdir(parents=True)
            w = PairWriter(root, [{'width': 2, 'height': 2}]*2, 100, workers=4)
            original = w._write_pair
            def delayed(pair):
                if pair[0].stamp_ns == 110: time.sleep(.03)
                original(pair)
            with patch.object(w, '_write_pair', delayed):
                for t in (110, 120, 130, 140): w.submit((frame(0,t), frame(1,t+1)))
                w.close()
            rows = list(csv.reader(io.StringIO((root/'mav0/cam0/data.csv').read_text())))
            self.assertEqual([int(r[0]) for r in rows[1:]], [10,20,30,40])
            self.assertEqual(w.written, 4)
        with tempfile.TemporaryDirectory() as d:
            w = PairWriter(Path(d), [{'width': 2, 'height': 2}]*2, 0)
            with self.assertRaises(RuntimeError): w.close()
            self.assertFalse(w.thread.is_alive())

    def test_export_requires_calibration_and_never_invents_it(self):
        try: import yaml
        except ImportError: self.skipTest('PyYAML not installed')
        with tempfile.TemporaryDirectory() as d:
            p = Path(d); root = p/'data'; root.mkdir(); cal = p/'cal'
            cameras = [{'width': 2, 'height': 2}]*2
            manifest = {'backend': 'csi-mavlink', 'complete': True, 'replay_ready': False,
                        'config': {'cameras': cameras}}
            (root/'capture_manifest.json').write_text(json.dumps(manifest))
            for sensor in ('cam0', 'cam1', 'imu0'):
                (root/'mav0'/sensor).mkdir(parents=True); (cal/sensor).mkdir(parents=True)
            with self.assertRaises(FileNotFoundError): export_calibration(root, cal)
            for i in range(2):
                matrix = [1,0,0,.1*i,0,1,0,0,0,0,1,0,0,0,0,1]
                data = {'resolution': [2,2], 'camera_model': 'pinhole', 'distortion_model': 'radial-tangential',
                        'intrinsics': [2,2,1,1], 'distortion_coefficients': [0,0,0,0],
                        'T_BS': {'rows': 4, 'cols': 4, 'data': matrix}}
                (cal/('cam'+str(i))/'sensor.yaml').write_text(yaml.safe_dump(data))
            noise = dict.fromkeys(['accelerometer_noise_density', 'gyroscope_noise_density',
                                  'accelerometer_random_walk', 'gyroscope_random_walk'], .01)
            (cal/'imu0/sensor.yaml').write_text(yaml.safe_dump(noise))
            result = export_calibration(root, cal)
            self.assertFalse(result['replay_ready']); self.assertEqual(len(result['calibration']), 3)
            self.assertTrue((root/'mav0/imu0/sensor.yaml').read_text().startswith('accelerometer_'))
            self.assertFalse((root/'mav0/imu0/sensor.yaml').read_text().startswith('{'))
            with self.assertRaises(ValueError): export_calibration(root, cal)
            data['T_BS']['data'][0] = -1
            (cal/'cam1/sensor.yaml').write_text(yaml.safe_dump(data))
            with self.assertRaises(ValueError): validate_calibration(cal, cameras)

    def test_complete_session_with_local_device_fakes(self):
        from csi_mavlink.cli import run_session
        class FakeCameras:
            def __init__(self, c): self.t = 5_000_000_000; self.started = False; self.seq = 0
            def now(self): self.t += 1_000_000; return self.t
            def start(self): self.started = True
            def close(self): self.started = False
            def poll(self):
                self.seq += 1
                return [frame(0, self.t, self.seq), frame(1, self.t+1000, self.seq)]
        class FakeLink:
            ack_result_value = 0
            def __init__(self, c): self.sent = {'HEARTBEAT': 0, 'TIMESYNC': 0, 'COMMAND_LONG': 0}; self.ack_pending = False; self.mav = types.SimpleNamespace(total_receive_errors=0); self.requests=[]; self.n=0
            def heartbeat(self): self.sent['HEARTBEAT'] += 1
            def request_message_interval(self, message_id, interval_us, confirmation=0):
                if not self.sent['HEARTBEAT']: raise AssertionError('interval command before heartbeat')
                self.sent['COMMAND_LONG'] += 1
            def request_imu_rate(self, interval_us, confirmation=0):
                if not self.sent['HEARTBEAT']: raise AssertionError('interval command before heartbeat')
                if interval_us != 5000: raise AssertionError('wrong HIGHRES_IMU interval')
                self.sent['COMMAND_LONG'] += 1; self.ack_pending = True
            def timesync(self,t): self.requests.append(t);self.sent['TIMESYNC'] += 1
            def read(self):
                self.assert_heartbeat()
                self.n += 1; t = 5_000_000_000+self.n*2_000_000
                msgs=[]
                if self.ack_pending:
                    ack=Message();ack.command=511;ack.result=self.ack_result_value;ack.get_type=lambda:'COMMAND_ACK';msgs.append(ack);self.ack_pending=False
                if self.requests:
                    t=self.requests.pop(0)
                    s=Message();s.tc1=t-1_000_000;s.ts1=t;s.get_type=lambda:'TIMESYNC';msgs.append(s)
                m=Message();m.time_usec=(5_000_000_000+self.n*2_000_000-1_000_000)//1000;m.get_type=lambda:'HIGHRES_IMU';msgs.append(m)
                return b'',msgs
            def assert_heartbeat(self):
                if not self.sent['HEARTBEAT']:raise AssertionError('read before heartbeat')
                if not self.sent['COMMAND_LONG']:raise AssertionError('read before HIGHRES_IMU request')
            def close(self): pass
        with tempfile.TemporaryDirectory() as d:
            c=config()
            for cam in c['cameras']:cam.update(width=2,height=2)
            args=types.SimpleNamespace(command='record',out=d,seq='fake',duration=.03,calibration=None)
            with patch('csi_mavlink.linux.Cameras', FakeCameras), patch('csi_mavlink.linux.SerialMavlink', FakeLink), contextlib.redirect_stdout(io.StringIO()):
                run_session(args,c)
            result=json.loads((Path(d)/'fake/capture_manifest.json').read_text())
            self.assertTrue(result['complete']);self.assertFalse(result['replay_ready'])
            self.assertGreater(result['stats']['pairs_written'],0)
            self.assertEqual(set(result['stats']['transmitted']),{'HEARTBEAT','TIMESYNC','COMMAND_LONG'})
            self.assertTrue(result['stats']['highres_imu_request']['ack_accepted'])
            self.assertEqual(result['stats']['highres_imu_request']['interval_us'],5000)
            self.assertFalse((Path(d)/'fake/mav0/imu0/sensor.yaml').exists())
            with self.assertRaises(FileExistsError):run_session(args,c)
            args.seq='denied'
            FakeLink.ack_result_value=3
            with patch('csi_mavlink.linux.Cameras', FakeCameras), patch('csi_mavlink.linux.SerialMavlink', FakeLink), contextlib.redirect_stdout(io.StringIO()):
                with self.assertRaisesRegex(RuntimeError,'interval command rejected'):
                    run_session(args,c)
            denied=json.loads((Path(d)/'denied/capture_manifest.json').read_text())
            self.assertFalse(denied['complete'])
            self.assertEqual(denied['stats']['highres_imu_request']['ack_result'],3)
            self.assertEqual(denied['stats']['pairs_written'],0)
            self.assertEqual(denied['stats']['imu_rows'],0)



if __name__ == '__main__': unittest.main()
