#!/usr/bin/env python3
import copy
import math

import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry


def yaw_from_quaternion(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def quaternion_from_yaw(yaw):
    half = 0.5 * yaw
    return 0.0, 0.0, math.sin(half), math.cos(half)


class StrictOdomFilter:
    def __init__(self):
        self.last_stamp = rospy.Time(0)
        self.dropped = 0
        self.input_topic = rospy.get_param("~input_topic", "/odom")
        self.output_topic = rospy.get_param("~output_topic", "/cartographer/odom")
        self.planarize = rospy.get_param("~planarize", True)
        self.publish_tf = rospy.get_param("~publish_tf", True)
        self.odom_frame = rospy.get_param("~odom_frame", "odom")
        self.base_frame = rospy.get_param("~base_frame", "base_footprint")
        self.publisher = rospy.Publisher(self.output_topic, Odometry, queue_size=50)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()
        self.subscriber = rospy.Subscriber(
            self.input_topic, Odometry, self.callback, queue_size=100
        )

    def callback(self, msg):
        if msg.header.stamp <= self.last_stamp:
            self.dropped += 1
            rospy.logwarn_throttle(
                5.0,
                "odom filter dropped %d duplicate or out-of-order messages",
                self.dropped,
            )
            return
        self.last_stamp = msg.header.stamp
        odom = self.filtered_odom(msg)
        self.publisher.publish(odom)
        if self.publish_tf:
            self.publish_transform(odom)

    def filtered_odom(self, msg):
        if not self.planarize:
            return msg

        odom = copy.deepcopy(msg)
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.z = 0.0

        yaw = yaw_from_quaternion(msg.pose.pose.orientation)
        x, y, z, w = quaternion_from_yaw(yaw)
        odom.pose.pose.orientation.x = x
        odom.pose.pose.orientation.y = y
        odom.pose.pose.orientation.z = z
        odom.pose.pose.orientation.w = w
        odom.twist.twist.linear.z = 0.0
        odom.twist.twist.angular.x = 0.0
        odom.twist.twist.angular.y = 0.0
        return odom

    def publish_transform(self, odom):
        transform = TransformStamped()
        transform.header.stamp = odom.header.stamp
        transform.header.frame_id = odom.header.frame_id
        transform.child_frame_id = odom.child_frame_id
        transform.transform.translation.x = odom.pose.pose.position.x
        transform.transform.translation.y = odom.pose.pose.position.y
        transform.transform.translation.z = odom.pose.pose.position.z
        transform.transform.rotation = odom.pose.pose.orientation
        self.tf_broadcaster.sendTransform(transform)


if __name__ == "__main__":
    rospy.init_node("indoor_odom_filter")
    StrictOdomFilter()
    rospy.spin()
