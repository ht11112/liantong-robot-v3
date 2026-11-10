import math
import socket
import time
import unittest
from dataclasses import replace

from liantong_v3.core import (
    LocalizationSessionEnded, MASK32, MeasurementClock, MotionCommands, NS, ProtocolError,
    SerialExchange, check_ack, crop_front_scan, encode_command, parse_feedback,
)


ODO_LINE = b'ODO,1234567890abcdef,7,100,104,1,2,1.25,-0.5,0.125,0.03,-0.12,0,0\n'
MS = 1_000_000
ROS_OFFSET = 100 * NS


class ProtocolTest(unittest.TestCase):
    def test_feedback_fields(self):
        feedback = parse_feedback(ODO_LINE)
        self.assertEqual(feedback.boot, '1234567890abcdef')
        self.assertEqual((feedback.sequence, feedback.measurement_ms, feedback.reply_ms), (7, 100, 104))
        self.assertTrue(feedback.valid)
        self.assertEqual(feedback.segment, 2)
        self.assertEqual((feedback.x, feedback.y, feedback.yaw, feedback.v, feedback.w),
                         (1.25, -.5, .125, .03, -.12))
        self.assertFalse(feedback.stop_latched)
        self.assertEqual(feedback.fault, 0)

    def test_rejects_bad_fields(self):
        for line in (b'ACK,CMD\n', ODO_LINE.replace(b'abcdef', b'abcdeg'),
                     ODO_LINE.replace(b',104,1,', b',104,2,'),
                     ODO_LINE.replace(b',7,', b',4294967296,'),
                     ODO_LINE.replace(b',1.25,', b',nan,'), ODO_LINE.rstrip() + b',0\n'):
            with self.subTest(line=line), self.assertRaises(ProtocolError):
                parse_feedback(line)

    def test_command_serialization(self):
        self.assertEqual(encode_command(.03, -.12), b'CMD,0.030000,-0.120000\n')
        self.assertEqual(encode_command(2., -4.), b'CMD,1.000000,-1.000000\n')
        with self.assertRaises(ValueError):
            encode_command(math.inf, 0.)

    def test_control_ack_identity(self):
        check_ack(b'ACK,STOP\n', 'STOP')
        with self.assertRaises(ProtocolError):
            check_ack(b'ACK,RESUME\n', 'STOP')


class LocalSocketPort:
    """A local byte transport for exercising the production read loop without serial hardware."""

    in_waiting = 5

    def __init__(self, endpoint):
        self.endpoint = endpoint

    def fileno(self):
        return self.endpoint.fileno()

    def write(self, data):
        return self.endpoint.send(data)

    def read(self, size):
        return self.endpoint.recv(size)


class SerialExchangeTest(unittest.TestCase):
    def test_reads_reply_in_chunks(self):
        host, peer = socket.socketpair()
        with host, peer:
            peer.sendall(ODO_LINE)
            transport = SerialExchange(LocalSocketPort(host), time.monotonic_ns)
            _, reply = transport.transact(b'QUERY\n')
            self.assertEqual(reply, ODO_LINE)
            self.assertEqual(peer.recv(16), b'QUERY\n')

    def test_rejects_incomplete_reply(self):
        host, peer = socket.socketpair()
        with host, peer:
            peer.sendall(b'ACK,ST')
            transport = SerialExchange(LocalSocketPort(host), time.monotonic_ns)
            with self.assertRaisesRegex(ProtocolError, 'within 100 ms'):
                transport.transact(b'STOP\n')


class MotionCommandsTest(unittest.TestCase):
    def setUp(self):
        self.host, self.peer = socket.socketpair()
        self.addCleanup(self.host.close)
        self.addCleanup(self.peer.close)
        self.peer.setblocking(False)
        self.commands = MotionCommands(300 * MS)
        self.old = encode_command(.03, 0.)
        self.new = encode_command(0., .07)

    def wire_at(self, now_ns):
        return SerialExchange(LocalSocketPort(self.host), lambda: now_ns)

    def assert_no_write(self):
        with self.assertRaises(BlockingIOError):
            self.peer.recv(128)

    def test_no_cmd_while_latched_or_at_write_expiry(self):
        self.commands.remember(self.old, NS)
        self.commands.send(self.wire_at(NS + MS), stop_latched=True)
        self.assert_no_write()
        self.commands.send(self.wire_at(NS + 300 * MS), stop_latched=False)
        self.assert_no_write()
        self.peer.sendall(b'ACK,CMD\n')
        self.commands.send(self.wire_at(NS + 299 * MS), stop_latched=False)
        self.assertEqual(self.peer.recv(128), self.old)

    def test_pre_ack_callback_cannot_restore_cleared_command(self):
        ack_ns = NS + 100 * MS
        self.commands.remember(self.old, NS)
        self.commands.clear_through(ack_ns)
        self.commands.remember(self.old, ack_ns - 1)
        self.commands.remember(self.old, ack_ns)
        self.commands.send(self.wire_at(ack_ns + MS), stop_latched=False)
        self.assert_no_write()
        self.commands.remember(self.new, ack_ns + 2 * MS)
        self.peer.sendall(b'ACK,CMD\n')
        self.commands.send(self.wire_at(ack_ns + 3 * MS), stop_latched=False)
        self.assertEqual(self.peer.recv(128), self.new)

    def test_newer_callback_survives_ack_handling(self):
        ack_ns = NS + 100 * MS
        self.commands.remember(self.new, ack_ns + 1)
        self.commands.clear_through(ack_ns)
        self.commands.remember(self.old, ack_ns - 1)
        self.peer.sendall(b'ACK,CMD\n')
        self.commands.send(self.wire_at(ack_ns + 2), stop_latched=False)
        self.assertEqual(self.peer.recv(128), self.new)


class MeasurementClockTest(unittest.TestCase):
    def setUp(self):
        self.feedback = parse_feedback(ODO_LINE)
        self.clock = MeasurementClock()

    def accept(self, feedback, started_ms=1000, now_ms=1020, offset_ns=ROS_OFFSET):
        return self.clock.accept(feedback, started_ms * MS, now_ms * MS, offset_ns + now_ms * MS)

    def test_uses_real_ticks_after_first_anchor_despite_query_jitter(self):
        first = self.accept(self.feedback)
        second = self.accept(replace(self.feedback, sequence=8, measurement_ms=150, reply_ms=221),
                             started_ms=1050, now_ms=1080)
        self.assertEqual(first, ROS_OFFSET + 996 * MS)
        self.assertEqual(second - first, 50 * MS)

    def test_duplicates_and_invalid_feedback_do_not_publish(self):
        self.assertIsNone(self.accept(replace(self.feedback, valid=False)))
        self.assertIsNotNone(self.accept(self.feedback, 1050, 1070))
        self.assertIsNone(self.accept(replace(self.feedback, reply_ms=180), 1100, 1120))
        self.assertIsNone(self.accept(replace(self.feedback, sequence=8, valid=False), 1150, 1170))

    def test_uint32_rollover(self):
        feedback = replace(self.feedback, sequence=MASK32, measurement_ms=MASK32-24, reply_ms=4)
        first = self.accept(feedback)
        second = self.accept(replace(feedback, sequence=0, measurement_ms=25, reply_ms=32), 1050, 1070)
        self.assertEqual(second - first, 50 * MS)

    def test_session_ends_on_new_boot_or_segment(self):
        for changed in (replace(self.feedback, boot='0000000000000001'),
                        replace(self.feedback, segment=3, valid=False)):
            with self.subTest(changed=changed):
                self.clock = MeasurementClock()
                self.accept(self.feedback)
                with self.assertRaisesRegex(LocalizationSessionEnded, 'boot or odometry segment'):
                    self.accept(changed, 1050, 1070)

    def test_clock_steps_end_session(self):
        for offset in (ROS_OFFSET - NS, ROS_OFFSET + NS):
            with self.subTest(offset=offset):
                self.clock = MeasurementClock()
                self.accept(self.feedback)
                with self.assertRaises(LocalizationSessionEnded):
                    self.accept(self.feedback, 1050, 1070, offset)

    def test_measurement_regression_does_not_make_fresh_tf(self):
        self.accept(self.feedback)
        with self.assertRaisesRegex(LocalizationSessionEnded, 'regressed'):
            self.accept(replace(self.feedback, sequence=8, measurement_ms=99), 1050, 1070)


class ScanTest(unittest.TestCase):
    def crop(self, ranges, *, laser_yaw=0.):
        return crop_front_scan(NS, -math.pi, math.pi, math.pi/4, .001, .1, 10., ranges,
                               laser_yaw=laser_yaw, min_usable_range=1.)

    def test_retains_only_front_rays_and_moves_stamp(self):
        raw = [2., 2., .9, 1., math.inf, math.nan, 10., 2., 2.]
        cropped = self.crop(raw)
        self.assertEqual((cropped.first, cropped.end), (2, 7))
        self.assertEqual(cropped.stamp_ns, NS + 2 * MS)
        self.assertEqual((cropped.angle_min, cropped.angle_max), (-math.pi/2, math.pi/2))
        self.assertTrue(math.isnan(cropped.ranges[0]))
        self.assertEqual(cropped.ranges[1:3], [1., math.inf])
        self.assertTrue(all(map(math.isnan, cropped.ranges[3:])))
        self.assertEqual(raw[2], .9)

    def test_laser_mounting_yaw_changes_selected_rays_not_their_frame(self):
        cropped = self.crop([2.] * 9, laser_yaw=math.pi/4)
        self.assertEqual((cropped.first, cropped.end), (1, 6))
        self.assertEqual(cropped.stamp_ns, NS + MS)
        self.assertAlmostEqual(cropped.angle_min, -3*math.pi/4)
        self.assertAlmostEqual(cropped.angle_max, math.pi/4)

    def test_rejects_scan_with_missing_front_sector(self):
        with self.assertRaisesRegex(ValueError, 'front 180'):
            crop_front_scan(NS, 0., math.pi/2, math.pi/8, .001, .1, 10., [2.] * 5,
                            laser_yaw=0., min_usable_range=1.)
