"""Pure data/timing contracts shared by capture, export and offline tests."""
import hashlib
import json
import math
import os
from pathlib import Path
import statistics
import struct
import zlib
from collections import deque
from dataclasses import dataclass


def atomic_json(path, value):
    path = Path(path)
    temp = path.with_suffix(path.suffix + '.tmp')
    with temp.open('w') as f:
        f.write(json.dumps(value, indent=2, allow_nan=False) + '\n')
        f.flush()
        os.fsync(f.fileno())
    temp.replace(path)
    directory = os.open(str(path.parent), os.O_RDONLY | getattr(os, 'O_DIRECTORY', 0))
    try: os.fsync(directory)
    finally: os.close(directory)


def load_config(path):
    def invalid(value):
        raise ValueError('non-finite JSON constant: ' + value)
    c = json.loads(Path(path).read_text(), parse_constant=invalid)
    required = {'schema_version', 'cameras', 'mavlink', 'pair_tolerance_ms'}
    if set(c) != required or type(c['schema_version']) is not int or c['schema_version'] != 1:
        raise ValueError('config requires schema_version=1, cameras, mavlink, pair_tolerance_ms')
    if not isinstance(c['cameras'], list) or len(c['cameras']) != 2:
        raise ValueError('exactly two cameras required')
    for cam in c['cameras']:
        if set(cam) != {'device', 'width', 'height', 'fps', 'format', 'readout_ns'}:
            raise ValueError('camera keys: device,width,height,fps,format,readout_ns')
        if not isinstance(cam['device'], str) or not cam['device'].startswith('/dev/'):
            raise ValueError('camera device must be an explicit /dev path')
        for key, upper in [('width', 8192), ('height', 8192), ('fps', 240)]:
            if type(cam[key]) is not int or not 1 <= cam[key] <= upper:
                raise ValueError('invalid camera ' + key)
        if cam['format'] not in ('NV12', 'YUY2', 'GRAY8'):
            raise ValueError('unsupported input pixel format')
        if cam['readout_ns'] is not None and (type(cam['readout_ns']) is not int or cam['readout_ns'] < 0):
            raise ValueError('readout_ns must be null or nonnegative integer')
    if c['cameras'][0]['device'] == c['cameras'][1]['device']:
        raise ValueError('left/right devices must differ')
    m = c['mavlink']
    required_mavlink = {'serial', 'baud', 'system_id', 'component_id', 'sensor_id', 'body_frame'}
    if not required_mavlink <= set(m) or set(m)-required_mavlink-{'imu_rate_hz'}:
        raise ValueError('mavlink keys: serial,baud,system_id,component_id,sensor_id,body_frame[,imu_rate_hz]')
    if not isinstance(m['serial'], str) or not m['serial'].startswith('/dev/'):
        raise ValueError('explicit serial device required')
    if type(m['baud']) is not int or m['baud'] not in (57600, 115200, 230400, 460800, 921600):
        raise ValueError('unsupported baud rate')
    for key, low in [('system_id', 1), ('component_id', 1), ('sensor_id', 0)]:
        if type(m[key]) is not int or not low <= m[key] <= 255:
            raise ValueError('invalid MAVLink source ' + key)
    if m['body_frame'] not in ('frd', 'flu'):
        raise ValueError('body_frame must be frd or flu')
    m.setdefault('imu_rate_hz', 200)
    if type(m['imu_rate_hz']) is not int or not 1 <= m['imu_rate_hz'] <= 1000:
        raise ValueError('imu_rate_hz must be an integer in [1,1000]')
    tol = c['pair_tolerance_ms']
    if type(tol) not in (int, float) or not math.isfinite(tol) or not 0 < tol <= 20:
        raise ValueError('pair_tolerance_ms must be in (0,20]')
    return c


# USB onboard/GCS extras that starve HIGHRES_IMU. Keep HEARTBEAT(0),
# COMMAND_ACK(77), HIGHRES_IMU(105), TIMESYNC(111).
CALIB_SILENCE_MSGIDS = (
    331, 31, 30, 32, 116, 26, 74, 141, 83, 36, 230, 241,
    12901, 1, 253, 245, 147, 29, 4, 410, 8, 380, 411,
)
CALIB_KEEP_MSGIDS = (0, 77, 105, 111)


class HighresRequest:
    """Bounded MAV_CMD_SET_MESSAGE_INTERVAL transaction for HIGHRES_IMU only."""
    def __init__(self, rate_hz=200):
        self.rate_hz = rate_hz
        self.interval_us = round(1_000_000/rate_hz)
        self.attempts = 0
        self.deadline_ns = None
        self.ack_result = None
        self.accepted = False
        self.in_progress = False

    def due(self, now_ns):
        if self.accepted or (self.deadline_ns is not None and now_ns < self.deadline_ns):
            return False
        if self.in_progress:
            raise RuntimeError('HIGHRES_IMU interval command remained IN_PROGRESS without final ACK')
        if self.attempts >= 3:
            raise RuntimeError('no accepted COMMAND_ACK for HIGHRES_IMU interval after 3 attempts')
        self.attempts += 1
        self.deadline_ns = now_ns+1_000_000_000
        return True

    def acknowledge(self, command, result, now_ns):
        if command != 511 or self.attempts == 0 or self.accepted:
            return False
        self.ack_result = result
        if result == 0:  # MAV_RESULT_ACCEPTED
            self.accepted = True
        elif result == 5:  # MAV_RESULT_IN_PROGRESS
            self.in_progress = True
            self.deadline_ns = now_ns+5_000_000_000
        elif result == 1:  # MAV_RESULT_TEMPORARILY_REJECTED
            self.in_progress = False
            self.deadline_ns = now_ns+1_000_000_000
        else:
            raise RuntimeError('HIGHRES_IMU interval command rejected, MAV_RESULT='+str(result))
        return True

    def snapshot(self):
        return {'command': 511, 'message_id': 105, 'requested_hz': self.rate_hz,
                'interval_us': self.interval_us, 'attempts': self.attempts,
                'ack_result': self.ack_result, 'ack_accepted': self.accepted}


class ClockMapping:
    """Matched low-RTT TIMESYNC with bounded affine clock-rate tracking."""
    def __init__(self, max_rtt_ns=10_000_000, min_samples=8):
        self.max_rtt_ns = max_rtt_ns
        self.min_samples = min_samples
        self.pending = set()
        self.samples = deque(maxlen=128)
        self.offset_ns = None
        self.max_drift_ns = 0
        self.reference_remote_ns = None
        self.reference_host_ns = None
        self.scale = 1.0
        self.updates = 0
        self.points = deque(maxlen=600)

    def map(self, remote_ns):
        if self.offset_ns is None: return None
        return round(self.reference_host_ns + (remote_ns-self.reference_remote_ns)*self.scale)

    def request(self, host_ns):
        self.pending = {t for t in self.pending if host_ns - t < 1_000_000_000}
        self.pending.add(host_ns)

    def reply(self, remote_ns, echoed_ns, received_ns):
        if echoed_ns not in self.pending:
            return None
        self.pending.remove(echoed_ns)
        rtt = received_ns - echoed_ns
        if remote_ns <= 0 or not 0 <= rtt <= self.max_rtt_ns:
            return None
        offset = (echoed_ns + received_ns) // 2 - remote_ns
        self.samples.append((rtt, offset))
        self.points.append((remote_ns, (echoed_ns+received_ns)//2, rtt))
        if self.offset_ns is None and len(self.samples) >= self.min_samples:
            best = sorted(self.samples)[:max(3, self.min_samples // 2)]
            self.offset_ns = int(statistics.median(x[1] for x in best))
            self.reference_remote_ns = remote_ns
            self.reference_host_ns = remote_ns+self.offset_ns
        if self.offset_ns is not None:
            # One lowest-RTT observation per second avoids packet bursts dominating a fit.
            buckets = {}
            for remote, host, latency in self.points:
                key = remote//1_000_000_000
                if key not in buckets or latency < buckets[key][2]: buckets[key]=(remote,host,latency)
            selected = sorted(buckets.values())
            if len(selected)>=4 and selected[-1][0]-selected[0][0]>=3_000_000_000:
                slopes=[(b[1]-a[1])/(b[0]-a[0]) for i,a in enumerate(selected) for b in selected[i+1:]
                        if b[0]-a[0]>=2_000_000_000]
                target_scale=statistics.median(slopes)
                if not .998<=target_scale<=1.002: raise ValueError('TIMESYNC clock rate outside 2000ppm')
                target=statistics.median(h+(remote_ns-r)*target_scale for r,h,_ in selected)
                previous=self.map(remote_ns)
                residual=target-previous
                self.max_drift_ns=max(self.max_drift_ns,abs(round(residual)))
                # Rebase continuously, with at most 50us correction per reply. Raw exchanges
                # and every applied model remain logged for independent offline refitting.
                correction=max(-50_000,min(50_000,residual))
                self.reference_remote_ns=remote_ns
                self.reference_host_ns=previous+correction
                self.scale=target_scale
                self.updates+=1
        return {'rtt_ns': rtt, 'offset_candidate_ns': offset, 'clock_scale': self.scale,
                'reference_remote_ns': self.reference_remote_ns,
                'reference_host_ns': self.reference_host_ns, 'mapping_updates': self.updates}


@dataclass
class Frame:
    camera: int
    stamp_ns: int
    receive_ns: int
    sequence: int
    driver_sequence: object
    pts_ns: int
    pixels: bytes


class Pairer:
    """Bounded, monotonic, one-to-one software pairing, keeping independent timestamps."""
    def __init__(self, tolerance_ns, capacity=8):
        self.tolerance_ns = tolerance_ns
        self.capacity = capacity
        self.queues = [deque(), deque()]
        self.last = [-1, -1]
        self.dropped = [0, 0]
        self.max_skew_ns = 0

    def push(self, frame):
        side = frame.camera
        if frame.stamp_ns <= self.last[side]:
            raise ValueError('camera timestamp rollback or duplicate')
        self.last[side] = frame.stamp_ns
        q = self.queues[side]
        q.append(frame)
        if len(q) > self.capacity:
            q.popleft()
            self.dropped[side] += 1
        pairs = []
        while all(self.queues):
            left, right = self.queues[0][0], self.queues[1][0]
            delta = right.stamp_ns - left.stamp_ns
            if abs(delta) <= self.tolerance_ns:
                self.queues[0].popleft(); self.queues[1].popleft()
                self.max_skew_ns = max(self.max_skew_ns, abs(delta))
                pairs.append((left, right))
            else:
                early = 0 if delta > 0 else 1
                self.queues[early].popleft(); self.dropped[early] += 1
        return pairs


class ImuDecoder:
    def __init__(self, config):
        self.config = config
        self.last_device_us = None
        self.last_host_ns = None
        self.rejected = 0

    def accept(self, msg, offset_ns, receive_ns):
        c = self.config
        if (msg.get_srcSystem(), msg.get_srcComponent(), getattr(msg, 'id', 0)) != (
                c['system_id'], c['component_id'], c['sensor_id']):
            self.rejected += 1; return None
        if (msg.fields_updated & 0x3f) != 0x3f:
            self.rejected += 1; return None
        values = [getattr(msg, k) for k in ('xgyro', 'ygyro', 'zgyro', 'xacc', 'yacc', 'zacc')]
        if not all(math.isfinite(v) for v in values) or msg.time_usec <= 0:
            self.rejected += 1; return None
        if self.last_device_us is not None:
            if msg.time_usec < self.last_device_us:
                raise ValueError('IMU boot timestamp rollback; stop, do not silently re-anchor')
            if msg.time_usec == self.last_device_us:
                self.rejected += 1; return None
        self.last_device_us = msg.time_usec
        if offset_ns is None:
            return None
        host_ns = offset_ns.map(msg.time_usec*1000) if isinstance(offset_ns, ClockMapping) else msg.time_usec * 1000 + offset_ns
        if host_ns is None: return None
        if host_ns <= 0 or host_ns > receive_ns + 10_000_000:
            self.rejected += 1; return None
        if self.last_host_ns is not None and host_ns <= self.last_host_ns:
            raise ValueError('mapped IMU time not increasing')
        self.last_host_ns = host_ns
        if c['body_frame'] == 'frd':
            values = [values[i] * (-1 if i in (1, 2, 4, 5) else 1) for i in range(6)]
        return host_ns, values


def png_gray(width, height, pixels, level=1):
    if len(pixels) != width * height:
        raise ValueError('GRAY8 pixel size mismatch')
    def chunk(kind, payload):
        return struct.pack('>I', len(payload)) + kind + payload + struct.pack('>I', zlib.crc32(kind + payload))
    rows = b''.join(b'\0' + pixels[y*width:(y+1)*width] for y in range(height))
    return (b'\x89PNG\r\n\x1a\n' + chunk(b'IHDR', struct.pack('>IIBBBBB', width, height, 8, 0, 0, 0, 0))
            + chunk(b'IDAT', zlib.compress(rows, level)) + chunk(b'IEND', b''))


def validate_calibration(directory, cameras):
    import yaml
    paths = [Path(directory)/sensor/'sensor.yaml' for sensor in ('cam0', 'cam1', 'imu0')]
    contents = [p.read_bytes() for p in paths]
    docs = [yaml.safe_load(c) for c in contents]
    def numbers(v, n):
        return (isinstance(v, list) and len(v) == n and
                all(type(x) in (int, float) and math.isfinite(x) for x in v))
    translations = []
    for i, d in enumerate(docs[:2]):
        c = cameras[i]
        if d.get('resolution') != [c['width'], c['height']] or d.get('camera_model') != 'pinhole' or d.get('distortion_model') != 'radial-tangential':
            raise ValueError('calibration dimensions/model differ from captured pixels')
        k = d.get('intrinsics'); distortion = d.get('distortion_coefficients')
        if not numbers(k, 4) or not k[0] > 0 or not k[1] > 0 or not 0 <= k[2] < c['width'] or not 0 <= k[3] < c['height'] or not numbers(distortion, 4):
            raise ValueError('invalid camera intrinsics/distortion')
        matrix = d.get('T_BS')
        if not isinstance(matrix, dict) or matrix.get('rows') != 4 or matrix.get('cols') != 4:
            raise ValueError('T_BS rows and cols must both be 4')
        t = matrix.get('data')
        if not numbers(t, 16) or t[12:] != [0, 0, 0, 1]:
            raise ValueError('T_BS requires finite row-major 4x4 matrix')
        r = [[t[y*4+x] for x in range(3)] for y in range(3)]
        for a in range(3):
            for b in range(3):
                if abs(sum(r[k][a]*r[k][b] for k in range(3)) - (a == b)) > 1e-6:
                    raise ValueError('T_BS rotation is not orthonormal')
        det = sum(r[0][j] * (r[1][(j+1)%3]*r[2][(j+2)%3]-r[1][(j+2)%3]*r[2][(j+1)%3]) for j in range(3))
        if abs(det-1) > 1e-6:
            raise ValueError('T_BS rotation must be proper')
        translations.append([t[3], t[7], t[11]])
        if 'timeshift_to_imu_ms' in d and not numbers([d['timeshift_to_imu_ms']], 1):
            raise ValueError('invalid camera timeshift')
    if math.dist(*translations) <= 1e-9:
        raise ValueError('stereo baseline cannot be zero')
    for k in ('accelerometer_noise_density', 'gyroscope_noise_density', 'accelerometer_random_walk', 'gyroscope_random_walk'):
        v = docs[2].get(k)
        if not numbers([v], 1) or v <= 0:
            raise ValueError('IMU noise density/random walk must be supplied and positive')
    return paths, contents


def export_calibration(root, directory):
    root = Path(root)
    manifest = json.loads((root/'capture_manifest.json').read_text())
    if manifest.get('backend') != 'csi-mavlink' or not manifest.get('complete'):
        raise ValueError('only a cleanly finalized CSI recording can be exported')
    paths, contents = validate_calibration(directory, manifest['config']['cameras'])
    targets = [root/'mav0'/sensor/'sensor.yaml' for sensor in ('cam0', 'cam1', 'imu0')]
    if any(p.exists() for p in targets):
        raise ValueError('sensor.yaml already exists; refusing to overwrite calibration')
    import yaml
    stats = manifest.get('stats', {})
    # Preserve source bytes/hash, but write the measured CSV rate rather than a stale calibration rate.
    exported = []
    for i, content in enumerate(contents):
        d = yaml.safe_load(content)
        count, first, last = ((stats.get('pairs_written'), stats.get('first_pair_ns'), stats.get('last_pair_ns'))
                              if i < 2 else
                              (stats.get('imu_rows'), stats.get('imu_first_ns'), stats.get('imu_last_ns')))
        if count and count > 1 and first is not None and last is not None and last > first:
            d['rate_hz'] = (count-1)*1e9/(last-first)
        exported.append(yaml.safe_dump(d, sort_keys=False, default_flow_style=False).encode())
    written = []
    try:
        for target, content in zip(targets, exported):
            # Exclusive creation prevents races with another export process.
            with target.open('xb') as f: f.write(content)
            written.append(target)
    except BaseException:
        for path in written: path.unlink(missing_ok=True)
        raise
    manifest['calibration'] = {str(p): hashlib.sha256(c).hexdigest() for p, c in zip(paths, contents)}
    manifest['exported_calibration_sha256'] = {str(p.relative_to(root)): hashlib.sha256(c).hexdigest()
                                             for p, c in zip(targets, exported)}
    manifest['calibration_frame'] = 'FLU, same IMU/body origin as recorded imu0'
    atomic_json(root/'capture_manifest.json', manifest)
    return manifest
