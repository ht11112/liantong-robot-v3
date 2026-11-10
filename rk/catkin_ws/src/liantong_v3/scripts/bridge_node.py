#!/usr/bin/env python3

import math
import queue
import time

import rospy
import serial
import tf2_ros
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from std_srvs.srv import Trigger, TriggerResponse

from liantong_v3.core import (
    LocalizationSessionEnded, MeasurementClock, MotionCommands, NS, ProtocolError,
    SerialExchange, check_ack, encode_command, parse_feedback,
)


class MotionInputs:
    def __init__(self, timeout_s):
        self.commands = MotionCommands(round(timeout_s * NS))
        self.actions = queue.Queue(maxsize=1)
        self.subscriber = rospy.Subscriber('/cmd_vel', Twist, self.receive, queue_size=1,
                                           tcp_nodelay=True)
        self.stop_service = rospy.Service('~stop', Trigger, lambda _: self.request('STOP'))
        self.resume_service = rospy.Service('~resume', Trigger, lambda _: self.request('RESUME'))

    def receive(self, message):
        received = time.monotonic_ns()
        try:
            command = encode_command(message.linear.x, message.angular.z)
        except ValueError as error:
            rospy.logwarn_throttle(2, str(error))
            command = None
        self.commands.remember(command, received)

    def request(self, name):
        finished = queue.Queue(maxsize=1)
        try:
            self.actions.put_nowait((name, finished))
        except queue.Full:
            return TriggerResponse(False, 'Control request is already queued')
        while not rospy.is_shutdown():
            try:
                return finished.get(timeout=.1)
            except queue.Empty:
                pass
        return TriggerResponse(False, 'Bridge stopped before ACK was confirmed')

    def send(self, wire, stopped):
        try:
            name, finished = self.actions.get_nowait()
        except queue.Empty:
            self.commands.send(wire, stopped)
            return

        # Neither STOP nor RESUME carries a cached Twist into the next cycle.
        try:
            _, reply = wire.transact((name + '\n').encode('ascii'))
            acknowledged_ns = time.monotonic_ns()
            check_ack(reply, name)
        except (ProtocolError, OSError) as error:
            finished.put_nowait(TriggerResponse(False, str(error)))
            raise
        self.commands.clear_through(acknowledged_ns)
        finished.put_nowait(TriggerResponse(True, 'ACK,{} (MCU parsed command)'.format(name)))


def publish_odom(publisher, broadcaster, feedback, stamp_ns):
    stamp = rospy.Time(*divmod(stamp_ns, NS))
    transform = TransformStamped()
    transform.header.stamp = stamp
    transform.header.frame_id = 'odom'
    transform.child_frame_id = 'base_link'
    transform.transform.translation.x = feedback.x
    transform.transform.translation.y = feedback.y
    transform.transform.rotation.z = math.sin(feedback.yaw / 2)
    transform.transform.rotation.w = math.cos(feedback.yaw / 2)

    message = Odometry()
    message.header = transform.header
    message.child_frame_id = transform.child_frame_id
    message.pose.pose.position.x = feedback.x
    message.pose.pose.position.y = feedback.y
    message.pose.pose.orientation = transform.transform.rotation
    message.twist.twist.linear.x = feedback.v
    message.twist.twist.angular.z = feedback.w
    broadcaster.sendTransform(transform)
    publisher.publish(message)


def run(wire, motion):
    clock = MeasurementClock()
    publisher = rospy.Publisher('/odom', Odometry, queue_size=1)
    broadcaster = tf2_ros.TransformBroadcaster()
    while not rospy.is_shutdown():
        started, line = wire.transact(b'QUERY\n')
        feedback = parse_feedback(line)
        before = time.monotonic_ns()
        ros_now = rospy.Time.now().to_nsec()
        monotonic_now = (before + time.monotonic_ns()) // 2
        stamp = clock.accept(feedback, started, monotonic_now, ros_now)
        if stamp is not None:
            publish_odom(publisher, broadcaster, feedback, stamp)
        if feedback.fault:
            rospy.logwarn_throttle(2, 'MCU fault %s, odometry valid=%s', feedback.fault, feedback.valid)
        if motion is not None and not rospy.is_shutdown():
            motion.send(wire, feedback.stop_latched)
        # A slow reply skips the unused wait. It does not create a backlog of polls.
        delay = started + 50_000_000 - time.monotonic_ns()
        if delay > 0:
            time.sleep(delay / NS)


def main():
    rospy.init_node('liantong_v3_bridge')
    if rospy.get_param('/use_sim_time', False):
        raise ValueError('The physical serial bridge requires wall-clock ROS time')
    port_name = rospy.get_param('~port')
    enable_motion = rospy.get_param('~enable_motion', False)
    timeout = float(rospy.get_param('~cmd_timeout_s', .3))
    if type(enable_motion) is not bool or not math.isfinite(timeout) or timeout <= 0:
        raise ValueError('enable_motion must be boolean and cmd_timeout_s must be finite and positive')
    try:
        with serial.Serial(port_name, 115200, timeout=0, write_timeout=.1, exclusive=True) as port:
            motion = MotionInputs(timeout) if enable_motion else None
            run(SerialExchange(port, time.monotonic_ns), motion)
    except (LocalizationSessionEnded, ProtocolError, OSError) as error:
        rospy.logfatal('V3 bridge stopped: %s', error)
        rospy.signal_shutdown(str(error))
        return 1
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
