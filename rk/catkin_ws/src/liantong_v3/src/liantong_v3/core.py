"""Serial protocol, measurement timing, and scan preparation without ROS imports."""

import math
import re
import select
import threading
from dataclasses import dataclass


NS = 1_000_000_000
MASK32 = (1 << 32) - 1
CLOCK_STEP_NS = 50_000_000


class ProtocolError(RuntimeError):
    pass


class LocalizationSessionEnded(RuntimeError):
    pass


@dataclass(frozen=True)
class Feedback:
    boot: str
    sequence: int
    measurement_ms: int
    reply_ms: int
    valid: bool
    segment: int
    x: float
    y: float
    yaw: float
    v: float
    w: float
    stop_latched: bool
    fault: int


def parse_feedback(line):
    try:
        kind, boot, seq, measured, replied, valid, segment, x, y, yaw, v, w, stopped, fault = (
            line.decode('ascii').rstrip('\r\n').split(','))
        if (kind != 'ODO' or not re.fullmatch(r'[0-9a-fA-F]{16}', boot)
                or valid not in ('0', '1') or stopped not in ('0', '1')):
            raise ValueError('invalid ODO identity or flags')
        integers = (seq, measured, replied, segment, fault)
        if any(not re.fullmatch(r'[0-9]{1,10}', field) or int(field) > MASK32 for field in integers):
            raise ValueError('invalid ODO uint32')
        physical = tuple(map(float, (x, y, yaw, v, w)))
        if not all(map(math.isfinite, physical)):
            raise ValueError('nonfinite ODO data')
        return Feedback(boot.lower(), int(seq), int(measured), int(replied), valid == '1',
                        int(segment), *physical, stopped == '1', int(fault))
    except ValueError as exc:
        raise ProtocolError('Expected valid ODO reply: {}'.format(exc)) from exc


def encode_command(v, w):
    if not math.isfinite(v) or not math.isfinite(w):
        raise ValueError('cmd_vel must be finite')
    v, w = max(-1., min(1., v)), max(-1., min(1., w))
    return 'CMD,{:.6f},{:.6f}\n'.format(v, w).encode('ascii')


def check_ack(line, name):
    if line.rstrip(b'\r\n') != ('ACK,' + name).encode('ascii'):
        raise ProtocolError('Expected ACK,{}, got {!r}'.format(name, line))


class SerialExchange:
    """An already-open port, used by only the RK transaction loop."""

    def __init__(self, port, monotonic_ns):
        self.port = port
        self.monotonic_ns = monotonic_ns

    def transact(self, request, *, start_before_ns=None):
        started = self.monotonic_ns()
        if start_before_ns is not None and started >= start_before_ns:
            return None
        deadline = started + 100_000_000
        if self.port.write(request) != len(request):
            raise ProtocolError('Short serial write')
        reply = bytearray()
        while len(reply) < 256:
            remaining = deadline - self.monotonic_ns()
            if remaining <= 0 or not select.select([self.port], [], [], remaining / NS)[0]:
                break
            reply.extend(self.port.read(min(self.port.in_waiting or 1, 256 - len(reply))))
            if b'\n' in reply:
                if reply.count(b'\n') != 1 or not reply.endswith(b'\n'):
                    raise ProtocolError('Unexpected data after serial reply')
                if self.monotonic_ns() < deadline:
                    return started, bytes(reply)
                break
        raise ProtocolError('No complete serial reply within 100 ms')


class MeasurementClock:
    def __init__(self):
        self.identity = None
        self.last_sequence = None
        self.last_measurement_ms = None
        self.last_source_ns = None
        self.ros_offset_ns = None
        self.last_ros_now = None

    def accept(self, feedback, request_start_ns, monotonic_now_ns, ros_now_ns):
        identity = feedback.boot, feedback.segment
        if self.identity is not None and identity != self.identity:
            raise LocalizationSessionEnded('MCU boot or odometry segment changed; restart localization')
        if ros_now_ns <= 0 or (self.last_ros_now is not None and ros_now_ns < self.last_ros_now):
            raise LocalizationSessionEnded('ROS time moved backward or is unavailable; restart localization')
        self.last_ros_now = ros_now_ns
        offset_ns = ros_now_ns - monotonic_now_ns
        if self.ros_offset_ns is not None and abs(offset_ns - self.ros_offset_ns) >= CLOCK_STEP_NS:
            raise LocalizationSessionEnded('ROS clock offset changed; restart localization')
        if not feedback.valid or feedback.sequence == self.last_sequence:
            return None
        if self.last_sequence is None:
            # Reply formation follows request start, so the initial stamp is conservative.
            source_age = ((feedback.reply_ms - feedback.measurement_ms) & MASK32) * 1_000_000
            source_ns = request_start_ns - source_age
        else:
            seq_step = (feedback.sequence - self.last_sequence) & MASK32
            measurement_step = (feedback.measurement_ms - self.last_measurement_ms) & MASK32
            if not (0 < seq_step < 1 << 31 and 0 < measurement_step < 1 << 31):
                raise LocalizationSessionEnded('MCU odometry time or sequence regressed; restart localization')
            source_ns = self.last_source_ns + measurement_step * 1_000_000
        if self.ros_offset_ns is None:
            self.ros_offset_ns = offset_ns
        stamp_ns = self.ros_offset_ns + source_ns
        if not 0 <= source_ns <= monotonic_now_ns or not 0 < stamp_ns <= ros_now_ns:
            raise LocalizationSessionEnded('Odometry time mapping is out of bounds; restart localization')
        self.identity = identity
        self.last_sequence = feedback.sequence
        self.last_measurement_ms = feedback.measurement_ms
        self.last_source_ns = source_ns
        return stamp_ns


class MotionCommands:
    """Latest local Twist receipt, with a barrier at acknowledged STOP or RESUME."""

    def __init__(self, timeout_ns):
        self.timeout_ns = timeout_ns
        self.received_ns = -1
        self.request = None
        self.lock = threading.Lock()

    def remember(self, request, received_ns):
        with self.lock:
            if received_ns > self.received_ns:
                self.received_ns = received_ns
                self.request = request

    def clear_through(self, ack_ns):
        with self.lock:
            if self.received_ns <= ack_ns:
                self.received_ns = ack_ns
                self.request = None

    def send(self, wire, stop_latched):
        if stop_latched:
            return
        with self.lock:
            request = self.request
            expires_ns = self.received_ns + self.timeout_ns
        if request is not None:
            exchange = wire.transact(request, start_before_ns=expires_ns)
            if exchange is not None:
                check_ack(exchange[1], 'CMD')


@dataclass(frozen=True)
class ScanCrop:
    first: int
    end: int
    stamp_ns: int
    angle_min: float
    angle_max: float
    ranges: list


def crop_front_scan(stamp_ns, angle_min, angle_max, angle_increment, time_increment,
                    range_min, range_max, ranges, *, laser_yaw, min_usable_range):
    count = len(ranges)
    if (stamp_ns <= 0 or count < 2
            or not all(map(math.isfinite, (angle_min, angle_max, angle_increment, time_increment,
                                          range_min, range_max)))
            or not 0 < angle_increment <= math.pi / 2 or time_increment < 0
            or not 0 <= range_min < range_max
            or abs(angle_max - (angle_min + (count - 1)*angle_increment)) > angle_increment*.02 + 1e-6):
        raise ValueError('Invalid laser scan geometry, range, or timestamp')

    # Front is in base_link. The retained angles still belong to the laser frame.
    base_angle = angle_min + laser_yaw
    half = math.pi / 2
    margin = 1e-6
    first = max(0, math.ceil((-half - margin - base_angle) / angle_increment))
    end = min(count, math.floor((half + margin - base_angle) / angle_increment) + 1)
    if (end <= first or base_angle + first*angle_increment > -half + angle_increment + margin
            or base_angle + (end - 1)*angle_increment < half - angle_increment - margin):
        raise ValueError('Laser scan does not cover a continuous front 180 degree sector')

    usable_min = max(range_min, min_usable_range)
    if usable_min >= range_max:
        raise ValueError('Laser has no usable range above the configured floor')
    # Filtered returns are unknown. Keep actual no-returns distinct.
    selected = [value if usable_min <= value < range_max or value == math.inf else math.nan
                for value in ranges[first:end]]
    return ScanCrop(first, end, stamp_ns + round(first*time_increment*NS),
                    angle_min + first*angle_increment, angle_min + (end - 1)*angle_increment,
                    selected)
