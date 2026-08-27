#!/usr/bin/env python3
import math
import os

import rospy
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import DeleteModel, GetWorldProperties, SetModelState, SpawnModel
from geometry_msgs.msg import Pose


DEFAULT_WAYPOINTS = [
    (-3.35, -0.15),
    (-2.15, -1.55),
    (-0.45, -1.75),
    (1.10, -1.18),
    (1.42, -0.28),
    (0.20, 0.42),
    (-1.18, 0.22),
    (-2.70, 0.08),
]


def yaw_to_quaternion(yaw):
    return 0.0, 0.0, math.sin(yaw * 0.5), math.cos(yaw * 0.5)


def make_pose(x, y, z, yaw):
    pose = Pose()
    pose.position.x = x
    pose.position.y = y
    pose.position.z = z
    qx, qy, qz, qw = yaw_to_quaternion(yaw)
    pose.orientation.x = qx
    pose.orientation.y = qy
    pose.orientation.z = qz
    pose.orientation.w = qw
    return pose


def kitten_sdf(model_name, package_dir):
    material_scripts_uri = "file://" + os.path.join(package_dir, "media", "materials", "scripts")
    material_textures_uri = "file://" + os.path.join(package_dir, "media", "materials", "textures")
    return """<?xml version="1.0"?>
<sdf version="1.6">
  <model name="{model_name}">
    <static>false</static>
    <allow_auto_disable>false</allow_auto_disable>
    <link name="kitten_link">
      <gravity>false</gravity>
      <kinematic>true</kinematic>
      <self_collide>false</self_collide>

      <collision name="body_collision">
        <pose>0 0 0.17 0 0 0</pose>
        <geometry><cylinder><radius>0.17</radius><length>0.34</length></cylinder></geometry>
      </collision>
      <collision name="head_collision">
        <pose>0.23 0 0.24 0 0 0</pose>
        <geometry><sphere><radius>0.10</radius></sphere></geometry>
      </collision>

      <visual name="yolo_kitten_photo_visual">
        <pose>0.46 0 0.48 0 0 0</pose>
        <geometry><box><size>0.025 1.00 0.96</size></box></geometry>
        <material>
          <script>
            <uri>{material_scripts_uri}</uri>
            <uri>{material_textures_uri}</uri>
            <name>IndoorDemo/YoloKitten</name>
          </script>
        </material>
      </visual>

      <visual name="body_visual">
        <pose>0 0 0.18 0 0 0</pose>
        <geometry><sphere><radius>0.17</radius></sphere></geometry>
        <material><ambient>0.92 0.50 0.22 1</ambient><diffuse>1.00 0.60 0.27 1</diffuse></material>
      </visual>
      <visual name="belly_visual">
        <pose>0.03 0 0.145 0 0 0</pose>
        <geometry><sphere><radius>0.115</radius></sphere></geometry>
        <material><ambient>0.95 0.82 0.64 1</ambient><diffuse>1.00 0.88 0.70 1</diffuse></material>
      </visual>
      <visual name="head_visual">
        <pose>0.23 0 0.25 0 0 0</pose>
        <geometry><sphere><radius>0.105</radius></sphere></geometry>
        <material><ambient>0.92 0.50 0.22 1</ambient><diffuse>1.00 0.60 0.27 1</diffuse></material>
      </visual>
      <visual name="left_ear_visual">
        <pose>0.24 0.07 0.36 0.55 0 0.35</pose>
        <geometry><box><size>0.055 0.035 0.095</size></box></geometry>
        <material><ambient>0.88 0.43 0.18 1</ambient><diffuse>0.96 0.52 0.22 1</diffuse></material>
      </visual>
      <visual name="right_ear_visual">
        <pose>0.24 -0.07 0.36 -0.55 0 -0.35</pose>
        <geometry><box><size>0.055 0.035 0.095</size></box></geometry>
        <material><ambient>0.88 0.43 0.18 1</ambient><diffuse>0.96 0.52 0.22 1</diffuse></material>
      </visual>
      <visual name="left_eye_visual">
        <pose>0.315 0.038 0.285 0 0 0</pose>
        <geometry><sphere><radius>0.014</radius></sphere></geometry>
        <material><ambient>0.02 0.02 0.02 1</ambient><diffuse>0.02 0.02 0.02 1</diffuse></material>
      </visual>
      <visual name="right_eye_visual">
        <pose>0.315 -0.038 0.285 0 0 0</pose>
        <geometry><sphere><radius>0.014</radius></sphere></geometry>
        <material><ambient>0.02 0.02 0.02 1</ambient><diffuse>0.02 0.02 0.02 1</diffuse></material>
      </visual>
      <visual name="nose_visual">
        <pose>0.335 0 0.25 0 0 0</pose>
        <geometry><sphere><radius>0.012</radius></sphere></geometry>
        <material><ambient>0.72 0.22 0.28 1</ambient><diffuse>0.88 0.32 0.38 1</diffuse></material>
      </visual>
      <visual name="tail_visual">
        <pose>-0.25 0 0.27 0 1.5708 0</pose>
        <geometry><cylinder><radius>0.026</radius><length>0.42</length></cylinder></geometry>
        <material><ambient>0.88 0.43 0.18 1</ambient><diffuse>0.96 0.52 0.22 1</diffuse></material>
      </visual>
      <visual name="front_left_leg_visual">
        <pose>0.09 0.09 0.065 0 0 0</pose>
        <geometry><cylinder><radius>0.024</radius><length>0.13</length></cylinder></geometry>
        <material><ambient>0.70 0.35 0.16 1</ambient><diffuse>0.85 0.45 0.20 1</diffuse></material>
      </visual>
      <visual name="front_right_leg_visual">
        <pose>0.09 -0.09 0.065 0 0 0</pose>
        <geometry><cylinder><radius>0.024</radius><length>0.13</length></cylinder></geometry>
        <material><ambient>0.70 0.35 0.16 1</ambient><diffuse>0.85 0.45 0.20 1</diffuse></material>
      </visual>
      <visual name="rear_left_leg_visual">
        <pose>-0.09 0.09 0.065 0 0 0</pose>
        <geometry><cylinder><radius>0.024</radius><length>0.13</length></cylinder></geometry>
        <material><ambient>0.70 0.35 0.16 1</ambient><diffuse>0.85 0.45 0.20 1</diffuse></material>
      </visual>
      <visual name="rear_right_leg_visual">
        <pose>-0.09 -0.09 0.065 0 0 0</pose>
        <geometry><cylinder><radius>0.024</radius><length>0.13</length></cylinder></geometry>
        <material><ambient>0.70 0.35 0.16 1</ambient><diffuse>0.85 0.45 0.20 1</diffuse></material>
      </visual>
    </link>
  </model>
</sdf>
""".format(
        model_name=model_name,
        material_scripts_uri=material_scripts_uri,
        material_textures_uri=material_textures_uri,
    )


class MovingKittenObstacle:
    def __init__(self):
        self.model_name = rospy.get_param("~model_name", "moving_kitten")
        self.package_dir = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))
        self.speed = max(0.0, rospy.get_param("~speed", 0.38))
        self.update_rate = max(1.0, rospy.get_param("~update_rate", 15.0))
        self.z = rospy.get_param("~z", 0.0)
        self.waypoints = self.load_waypoints()
        self.segments = self.build_segments(self.waypoints)
        self.path_length = sum(segment[4] for segment in self.segments)

        rospy.wait_for_service("/gazebo/get_world_properties")
        rospy.wait_for_service("/gazebo/delete_model")
        rospy.wait_for_service("/gazebo/spawn_sdf_model")
        rospy.wait_for_service("/gazebo/set_model_state")

        self.get_world = rospy.ServiceProxy("/gazebo/get_world_properties", GetWorldProperties)
        self.delete_model = rospy.ServiceProxy("/gazebo/delete_model", DeleteModel)
        self.spawn_model = rospy.ServiceProxy("/gazebo/spawn_sdf_model", SpawnModel)
        self.set_model_state = rospy.ServiceProxy("/gazebo/set_model_state", SetModelState)

    def load_waypoints(self):
        raw = rospy.get_param("~waypoints", DEFAULT_WAYPOINTS)
        waypoints = [(float(point[0]), float(point[1])) for point in raw]
        if len(waypoints) < 2:
            raise rospy.ROSException("moving kitten needs at least two waypoints")
        return waypoints

    def build_segments(self, waypoints):
        segments = []
        for index, start in enumerate(waypoints):
            end = waypoints[(index + 1) % len(waypoints)]
            dx = end[0] - start[0]
            dy = end[1] - start[1]
            length = math.hypot(dx, dy)
            if length > 1e-6:
                yaw = math.atan2(dy, dx)
                segments.append((start, end, dx, dy, length, yaw))
        if not segments:
            raise rospy.ROSException("moving kitten waypoint path has zero length")
        return segments

    def delete_existing_model(self):
        try:
            world = self.get_world()
        except rospy.ServiceException as exc:
            rospy.logwarn("Could not query Gazebo world before spawning kitten: %s", exc)
            return
        if self.model_name in world.model_names:
            try:
                self.delete_model(self.model_name)
                rospy.sleep(0.3)
            except rospy.ServiceException as exc:
                rospy.logwarn("Could not delete existing kitten model %s: %s", self.model_name, exc)

    def spawn(self):
        self.delete_existing_model()
        x, y = self.waypoints[0]
        yaw = self.segments[0][5]
        result = self.spawn_model(
            self.model_name,
            kitten_sdf(self.model_name, self.package_dir),
            "",
            make_pose(x, y, self.z, yaw),
            "world",
        )
        if not result.success:
            raise rospy.ROSException("moving kitten spawn failed: %s" % result.status_message)
        rospy.loginfo("Spawned moving kitten obstacle '%s' on a %.2f m loop.", self.model_name, self.path_length)

    def pose_at_distance(self, distance):
        distance = distance % self.path_length
        for start, _, dx, dy, length, yaw in self.segments:
            if distance <= length:
                ratio = distance / length
                return start[0] + dx * ratio, start[1] + dy * ratio, yaw
            distance -= length
        start, _, _, _, _, yaw = self.segments[-1]
        return start[0], start[1], yaw

    def publish_state(self, x, y, yaw):
        state = ModelState()
        state.model_name = self.model_name
        state.pose = make_pose(x, y, self.z, yaw)
        state.twist.linear.x = self.speed * math.cos(yaw)
        state.twist.linear.y = self.speed * math.sin(yaw)
        state.reference_frame = "world"
        self.set_model_state(state)

    def run(self):
        self.spawn()
        start_time = rospy.Time.now()
        rate = rospy.Rate(self.update_rate)
        while not rospy.is_shutdown():
            elapsed = (rospy.Time.now() - start_time).to_sec()
            x, y, yaw = self.pose_at_distance(elapsed * self.speed)
            try:
                self.publish_state(x, y, yaw)
            except rospy.ServiceException as exc:
                rospy.logwarn_throttle(5.0, "Could not update moving kitten pose: %s", exc)
            try:
                rate.sleep()
            except rospy.ROSInterruptException:
                break


def main():
    rospy.init_node("moving_kitten_obstacle")
    MovingKittenObstacle().run()


if __name__ == "__main__":
    main()
