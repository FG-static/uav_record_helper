"""Exercise the real session loop and disk writer with injected sensor/timing faults."""
import contextlib
import errno
import io
import json
from pathlib import Path
import tempfile
import types
import unittest
from unittest.mock import patch

from csi_mavlink import cli, core, linux


class Cameras:
    instance = None
    def __init__(self, config):
        type(self).instance = self
        self.time = 10_000_000_000
        self.started = False
        self.sequence = 0
    def now(self):
        self.time += 20_000_000
        return self.time
    def start(self): self.started = True
    def close(self): self.started = False
    def poll(self):
        self.sequence += 1
        return [core.Frame(side,self.time-2_000_000+side*1_000_000,
                           self.time,self.sequence,self.sequence,self.time,b'1234') for side in (0,1)]


def message(kind, **fields):
    return types.SimpleNamespace(get_type=lambda:kind, get_srcSystem=lambda:1,
        get_srcComponent=lambda:1, **fields)


class Serial:
    instance = None
    def __init__(self, config):
        type(self).instance = self
        self.calls=[];self.sent={'HEARTBEAT':0,'TIMESYNC':0,'COMMAND_LONG':0};self.mav=types.SimpleNamespace(total_receive_errors=0)
    def heartbeat(self): self.calls.append('heartbeat'); self.sent['HEARTBEAT']+=1
    def request_message_interval(self, msgid, interval, confirmation=0): self.calls.append(('511',msgid,interval))
    def request_imu_rate(self, interval, confirmation): self.calls.append(('511',interval))
    def timesync(self, now): pass
    def close(self): pass
    def read(self):
        self.calls.append('read')
        t=Cameras.instance.time-1_000_000
        return b'raw', [message('COMMAND_ACK',command=511,result=0),
            message('TIMESYNC',tc1=t,ts1=t-1000),
            message('HIGHRES_IMU',time_usec=t//1000,id=0,fields_updated=63,
                    xgyro=0.,ygyro=0.,zgyro=0.,xacc=0.,yacc=0.,zacc=9.81)]


class OutlierMapping(core.ClockMapping):
    def __init__(self):
        super().__init__()
        self.offset_ns=0
        self.max_drift_ns=4_000_000
    def map(self, remote): return remote
    def reply(self,*args): raise ValueError('TIMESYNC clock rate outside 2000ppm')


class Controller:
    advisory_quality=True
    def __init__(self):
        self.recording=False;self.stop_requested=False;self.count=0;self.warnings=[];self.finished=False
    def setup(self,root,manifest): manifest['capture_policy']='record_then_analyze'
    def warn(self,text):
        if text not in self.warnings:self.warnings.append(text)
    def frame(self,frame):
        if self.recording:self.writer.submit(frame)
        self.count+=1
    def tick(self,*args):
        self.recording=True
        if self.count>=16:self.stop_requested=True
    def progress(self,*args): pass
    def close(self): pass
    def finish(self,*args): self.finished=True
    def summary(self): return dict(warnings=self.warnings)
    def pair(self,*args): raise AssertionError('raw capture must not require stereo pairing')
    def accept_pair(self,*args): raise AssertionError('raw capture must not require stereo pairing')


class Session(unittest.TestCase):
    def run_capture(self,folder,controller):
        config=json.loads((Path(__file__).resolve().parents[1]/'config/rk3588-csi-mavlink.json').read_text())
        for c in config['cameras']:c.update(width=2,height=2)
        args=types.SimpleNamespace(command='record',out=folder,seq='capture',duration=0,calibration=None)
        with patch.object(linux,'Cameras',Cameras),patch.object(linux,'SerialMavlink',Serial), \
                patch.object(cli,'ClockMapping',OutlierMapping),contextlib.redirect_stdout(io.StringIO()):
            return cli.run_session(args,config,controller)

    def test_clock_outliers_and_low_sensor_rate_do_not_interrupt_raw_capture(self):
        with tempfile.TemporaryDirectory() as d:
            c=Controller();manifest=self.run_capture(d,c)
            self.assertTrue(manifest['complete']);self.assertIsNone(manifest['error'])
            self.assertTrue(c.finished)
            self.assertGreater(manifest['stats']['imu_max_gap_ns'],25_000_000)
            self.assertLess(manifest['stats']['imu_rate_hz'],120)
            self.assertEqual(Serial.instance.calls[0],'heartbeat')
            self.assertIn(('511',5000),Serial.instance.calls)
            self.assertEqual(len(c.warnings),2)
            root=Path(d)/'capture'
            exchanges=[json.loads(line) for line in (root/'raw/timesync.jsonl').read_text().splitlines()]
            self.assertTrue(all(not e['model_update_applied'] for e in exchanges))
            self.assertGreater(len(exchanges),5)
            for side,count in enumerate(manifest['stats']['camera_saved_counts']):
                self.assertGreater(count,2)
                self.assertEqual(len(list((root/('mav0/cam%d/data'%side)).glob('*.png'))),count)

    def test_actual_disk_failure_is_reported_with_incomplete_manifest(self):
        with tempfile.TemporaryDirectory() as d:
            with patch.object(linux.PairWriter,'_write_pair',side_effect=OSError(errno.ENOSPC,'disk full')):
                with self.assertRaisesRegex(RuntimeError,'disk full'):self.run_capture(d,Controller())
            m=json.loads((Path(d)/'capture/capture_manifest.json').read_text())
            self.assertFalse(m['complete'])
            self.assertIn('disk full',m['error'])

    def test_regular_recorder_still_rejects_invalid_clock_model(self):
        with self.assertRaises(ValueError):
            cli.clock_reply(OutlierMapping(),1,2,3)


if __name__=='__main__':unittest.main()
