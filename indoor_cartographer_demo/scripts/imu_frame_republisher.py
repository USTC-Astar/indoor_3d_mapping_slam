#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Imu


class ImuFrameRepublisher:
    def __init__(self):
        self.input_topic = rospy.get_param("~input_topic", "/imu_raw")
        self.output_topic = rospy.get_param("~output_topic", "/imu")
        self.frame_id = rospy.get_param("~frame_id", "base_link")
        self.planar_motion = rospy.get_param("~planar_motion", False)
        self.gravity_magnitude = rospy.get_param("~gravity_magnitude", 9.80665)
        self.publisher = rospy.Publisher(self.output_topic, Imu, queue_size=50)
        self.subscriber = rospy.Subscriber(
            self.input_topic, Imu, self._callback, queue_size=50
        )

    def _callback(self, msg):
        msg.header.frame_id = self.frame_id
        if self.planar_motion:
            msg.angular_velocity.x = 0.0
            msg.angular_velocity.y = 0.0
            msg.linear_acceleration.x = 0.0
            msg.linear_acceleration.y = 0.0
            msg.linear_acceleration.z = self.gravity_magnitude
        self.publisher.publish(msg)


def main():
    rospy.init_node("imu_frame_republisher")
    ImuFrameRepublisher()
    rospy.spin()


if __name__ == "__main__":
    main()
