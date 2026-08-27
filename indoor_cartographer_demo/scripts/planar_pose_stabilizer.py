#!/usr/bin/env python3

import math
import threading

import rospy
from gazebo_msgs.msg import ModelState, ModelStates


def yaw_from_quaternion(quaternion):
    siny_cosp = 2.0 * (
        quaternion.w * quaternion.z + quaternion.x * quaternion.y
    )
    cosy_cosp = 1.0 - 2.0 * (
        quaternion.y * quaternion.y + quaternion.z * quaternion.z
    )
    return math.atan2(siny_cosp, cosy_cosp)


def quaternion_from_yaw(yaw):
    half_yaw = 0.5 * yaw
    return 0.0, 0.0, math.sin(half_yaw), math.cos(half_yaw)


class PlanarPoseStabilizer:
    def __init__(self):
        self.model_name = rospy.get_param("~model_name", "indoor_mapper_bot")
        self.target_z = rospy.get_param("~target_z", 0.025)
        self.state_lock = threading.Lock()
        self.latest_state = None

        self.publisher = rospy.Publisher(
            "/gazebo/set_model_state", ModelState, queue_size=1
        )
        self.subscriber = rospy.Subscriber(
            "/gazebo/model_states", ModelStates, self.states_callback, queue_size=1
        )
        rate = rospy.get_param("~update_rate", 100.0)
        self.timer = rospy.Timer(rospy.Duration(1.0 / rate), self.update)

    def states_callback(self, message):
        try:
            index = message.name.index(self.model_name)
        except ValueError:
            return
        with self.state_lock:
            self.latest_state = (message.pose[index], message.twist[index])

    def update(self, _event):
        with self.state_lock:
            if self.latest_state is None:
                return
            pose, twist = self.latest_state

        state = ModelState()
        state.model_name = self.model_name
        state.reference_frame = "world"
        state.pose.position.x = pose.position.x
        state.pose.position.y = pose.position.y
        state.pose.position.z = self.target_z
        quaternion = quaternion_from_yaw(yaw_from_quaternion(pose.orientation))
        state.pose.orientation.x = quaternion[0]
        state.pose.orientation.y = quaternion[1]
        state.pose.orientation.z = quaternion[2]
        state.pose.orientation.w = quaternion[3]
        state.twist.linear.x = twist.linear.x
        state.twist.linear.y = twist.linear.y
        state.twist.angular.z = twist.angular.z
        self.publisher.publish(state)


def main():
    rospy.init_node("planar_pose_stabilizer")
    PlanarPoseStabilizer()
    rospy.spin()


if __name__ == "__main__":
    main()
