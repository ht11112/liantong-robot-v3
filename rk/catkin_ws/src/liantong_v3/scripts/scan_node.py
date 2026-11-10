#!/usr/bin/env python3

import math

import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import LaserScan

from liantong_v3.core import NS, crop_front_scan


class ScanRelay:
    def __init__(self):
        self.yaw = float(rospy.get_param('~laser_yaw', 0.0))
        self.minimum = float(rospy.get_param('~min_usable_range', 1.0))
        laser_xyz = [float(rospy.get_param('~laser_' + axis, default))
                     for axis, default in (('x', .13), ('y', 0.), ('z', .20))]
        if not all(map(math.isfinite, (*laser_xyz, self.yaw, self.minimum))) or self.minimum < 0:
            raise ValueError('Invalid laser mounting or minimum range')

        self.publisher = rospy.Publisher('/scan', LaserScan, queue_size=1)
        self.static_tf = tf2_ros.StaticTransformBroadcaster()
        tf = TransformStamped()
        tf.header.stamp = rospy.Time.now()
        tf.header.frame_id = 'base_link'
        tf.child_frame_id = 'laser'
        tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z = laser_xyz
        tf.transform.rotation.z = math.sin(self.yaw / 2)
        tf.transform.rotation.w = math.cos(self.yaw / 2)
        self.static_tf.sendTransform(tf)
        self.subscriber = rospy.Subscriber(rospy.get_param('~scan_topic', '/lidar1/laser_fan'),
                                           LaserScan, self.receive, queue_size=1, tcp_nodelay=True)

    def receive(self, raw):
        try:
            crop = crop_front_scan(raw.header.stamp.to_nsec(), raw.angle_min, raw.angle_max,
                                   raw.angle_increment, raw.time_increment, raw.range_min,
                                   raw.range_max, raw.ranges, laser_yaw=self.yaw,
                                   min_usable_range=self.minimum)
            if raw.intensities and len(raw.intensities) != len(raw.ranges):
                raise ValueError('Intensity count does not match measured rays')
        except ValueError as error:
            rospy.logwarn_throttle(2, str(error))
            return

        scan = LaserScan()
        scan.header.stamp = rospy.Time(*divmod(crop.stamp_ns, NS))
        scan.header.frame_id = 'laser'
        scan.angle_min, scan.angle_max = crop.angle_min, crop.angle_max
        scan.angle_increment, scan.time_increment = raw.angle_increment, raw.time_increment
        scan.scan_time = raw.scan_time
        scan.range_min, scan.range_max = max(raw.range_min, self.minimum), raw.range_max
        scan.ranges = crop.ranges
        scan.intensities = raw.intensities[crop.first:crop.end]
        self.publisher.publish(scan)


if __name__ == '__main__':
    rospy.init_node('liantong_v3_scan')
    relay = ScanRelay()
    rospy.spin()
