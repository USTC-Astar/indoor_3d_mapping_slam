#!/usr/bin/env python3

import math
import threading

import rospy
from gazebo_msgs.msg import LinkStates
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Bool, String


def quaternion_conjugate(quaternion):
    if hasattr(quaternion, "x"):
        return (-quaternion.x, -quaternion.y, -quaternion.z, quaternion.w)
    return (-quaternion[0], -quaternion[1], -quaternion[2], quaternion[3])


def rotate_vector(quaternion, point):
    quaternion_x, quaternion_y, quaternion_z, quaternion_w = quaternion
    point_x, point_y, point_z = point
    temp_x = 2.0 * (quaternion_y * point_z - quaternion_z * point_y)
    temp_y = 2.0 * (quaternion_z * point_x - quaternion_x * point_z)
    temp_z = 2.0 * (quaternion_x * point_y - quaternion_y * point_x)
    return (
        point_x + quaternion_w * temp_x + quaternion_y * temp_z - quaternion_z * temp_y,
        point_y + quaternion_w * temp_y + quaternion_z * temp_x - quaternion_x * temp_z,
        point_z + quaternion_w * temp_z + quaternion_x * temp_y - quaternion_y * temp_x,
    )


class OctomapCloudFilter:
    def __init__(self):
        self.input_topic = rospy.get_param("~input_topic", "/points2")
        self.output_topic = rospy.get_param("~output_topic", "/octomap/points_filtered")
        self.completion_topic = rospy.get_param("~completion_topic", "/active_slam/completed")
        self.link_states_topic = rospy.get_param("~link_states_topic", "/gazebo/link_states")
        self.sensor_link = rospy.get_param("~sensor_link", "indoor_mapper_bot::laser3d_link")
        self.base_link = rospy.get_param("~base_link", "indoor_mapper_bot::base_footprint")
        self.sensor_offset = (
            rospy.get_param("~sensor_offset_x", 0.03),
            rospy.get_param("~sensor_offset_y", 0.0),
            rospy.get_param("~sensor_offset_z", 0.285),
        )
        self.dynamic_link_tokens = rospy.get_param("~dynamic_link_tokens", ["moving_kitten"])
        self.min_range = rospy.get_param("~min_range", 0.22)
        self.max_range = rospy.get_param("~max_range", 5.45)
        self.min_z = rospy.get_param("~min_z", -0.16)
        self.max_z = rospy.get_param("~max_z", 2.20)
        self.min_world_z = rospy.get_param("~min_world_z", 0.02)
        self.max_world_z = rospy.get_param("~max_world_z", 2.48)
        self.ground_filter_max_z = rospy.get_param("~ground_filter_max_z", 0.20)
        self.vertical_support_min_z = rospy.get_param("~vertical_support_min_z", 0.45)
        self.ground_column_size = rospy.get_param("~ground_column_size", 0.15)
        self.voxel_size = rospy.get_param("~voxel_size", 0.075)
        self.dynamic_filter_radius = rospy.get_param("~dynamic_filter_radius", 0.72)
        self.freeze_on_completion = rospy.get_param("~freeze_on_completion", True)

        self.state_lock = threading.Lock()
        self.sensor_pose = None
        self.base_pose = None
        self.dynamic_positions = []
        self.frozen = False
        self.received_clouds = 0
        self.published_clouds = 0
        self.filtered_dynamic_points = 0

        self.publisher = rospy.Publisher(self.output_topic, PointCloud2, queue_size=1)
        self.status_publisher = rospy.Publisher(
            "/octomap_filter/status", String, queue_size=1, latch=True
        )
        self.cloud_subscriber = rospy.Subscriber(
            self.input_topic, PointCloud2, self.cloud_callback, queue_size=1
        )
        self.link_states_subscriber = rospy.Subscriber(
            self.link_states_topic, LinkStates, self.link_states_callback, queue_size=1
        )
        self.completion_subscriber = rospy.Subscriber(
            self.completion_topic, Bool, self.completion_callback, queue_size=1
        )

    def completion_callback(self, message):
        if self.freeze_on_completion and message.data:
            self.frozen = True
            self.status_publisher.publish(
                "octomap input frozen after autonomous exploration completion"
            )

    def link_states_callback(self, message):
        sensor_pose = None
        base_pose = None
        dynamic_positions = []
        for name, pose in zip(message.name, message.pose):
            if name == self.sensor_link:
                sensor_pose = pose
            if name == self.base_link:
                base_pose = pose
            if any(token in name for token in self.dynamic_link_tokens):
                dynamic_positions.append(
                    (pose.position.x, pose.position.y, pose.position.z)
                )
        with self.state_lock:
            self.sensor_pose = sensor_pose
            self.base_pose = base_pose
            self.dynamic_positions = dynamic_positions

    def dynamic_positions_in_sensor_frame(self):
        sensor_pose, dynamic_positions = self.sensor_pose_and_dynamic_positions()
        if sensor_pose is None or not dynamic_positions:
            return []

        sensor_position, sensor_orientation = sensor_pose
        inverse_orientation = quaternion_conjugate(sensor_orientation)
        transformed = []
        for position in dynamic_positions:
            relative = (
                position[0] - sensor_position[0],
                position[1] - sensor_position[1],
                position[2] - sensor_position[2],
            )
            transformed.append(rotate_vector(inverse_orientation, relative))
        return transformed

    def sensor_pose_and_dynamic_positions(self):
        with self.state_lock:
            sensor_pose = self.sensor_pose
            base_pose = self.base_pose
            dynamic_positions = list(self.dynamic_positions)

        if sensor_pose is not None:
            sensor_position = (
                sensor_pose.position.x,
                sensor_pose.position.y,
                sensor_pose.position.z,
            )
            sensor_orientation = sensor_pose.orientation
        elif base_pose is not None:
            base_orientation = (
                base_pose.orientation.x,
                base_pose.orientation.y,
                base_pose.orientation.z,
                base_pose.orientation.w,
            )
            rotated_offset = rotate_vector(base_orientation, self.sensor_offset)
            sensor_position = (
                base_pose.position.x + rotated_offset[0],
                base_pose.position.y + rotated_offset[1],
                base_pose.position.z + rotated_offset[2],
            )
            sensor_orientation = base_pose.orientation
        else:
            return None, dynamic_positions
        orientation = (
            sensor_orientation.x,
            sensor_orientation.y,
            sensor_orientation.z,
            sensor_orientation.w,
        )
        return (sensor_position, orientation), dynamic_positions

    def near_dynamic_object(self, point, dynamic_positions):
        radius_squared = self.dynamic_filter_radius * self.dynamic_filter_radius
        for dynamic_position in dynamic_positions:
            delta_x = point[0] - dynamic_position[0]
            delta_y = point[1] - dynamic_position[1]
            delta_z = point[2] - dynamic_position[2]
            if delta_x * delta_x + delta_y * delta_y + delta_z * delta_z <= radius_squared:
                return True
        return False

    def voxel_key(self, point):
        return tuple(
            int(math.floor(coordinate / self.voxel_size)) for coordinate in point
        )

    def cloud_callback(self, message):
        self.received_clouds += 1
        if self.frozen or not message.header.frame_id:
            return

        sensor_pose, _ = self.sensor_pose_and_dynamic_positions()
        if sensor_pose is None:
            return
        sensor_position, sensor_orientation = sensor_pose
        dynamic_positions = self.dynamic_positions_in_sensor_frame()
        min_range_squared = self.min_range * self.min_range
        max_range_squared = self.max_range * self.max_range
        candidates = []
        column_max_z = {}
        voxels = {}
        filtered_dynamic = 0

        for point in point_cloud2.read_points(
            message, field_names=("x", "y", "z"), skip_nans=True
        ):
            point_x, point_y, point_z = point
            if not (
                math.isfinite(point_x)
                and math.isfinite(point_y)
                and math.isfinite(point_z)
            ):
                continue
            if point_z < self.min_z or point_z > self.max_z:
                continue
            world_offset = rotate_vector(
                sensor_orientation, (point_x, point_y, point_z)
            )
            world_z = sensor_position[2] + world_offset[2]
            if world_z < self.min_world_z or world_z > self.max_world_z:
                continue
            range_squared = point_x * point_x + point_y * point_y + point_z * point_z
            if range_squared < min_range_squared or range_squared > max_range_squared:
                continue
            point_tuple = (point_x, point_y, point_z)
            if self.near_dynamic_object(point_tuple, dynamic_positions):
                filtered_dynamic += 1
                continue
            world_x = sensor_position[0] + world_offset[0]
            world_y = sensor_position[1] + world_offset[1]
            column_key = (
                int(math.floor(world_x / self.ground_column_size)),
                int(math.floor(world_y / self.ground_column_size)),
            )
            candidates.append((point_tuple, world_z, column_key))
            column_max_z[column_key] = max(column_max_z.get(column_key, world_z), world_z)

        for point_tuple, world_z, column_key in candidates:
            if (
                world_z <= self.ground_filter_max_z
                and column_max_z[column_key] < self.vertical_support_min_z
            ):
                continue
            voxels[self.voxel_key(point_tuple)] = point_tuple

        if not voxels:
            return

        self.filtered_dynamic_points += filtered_dynamic
        filtered_cloud = point_cloud2.create_cloud_xyz32(
            message.header, list(voxels.values())
        )
        self.publisher.publish(filtered_cloud)
        self.published_clouds += 1
        self.status_publisher.publish(
            "octomap clouds=%d points=%d dynamic_filtered=%d"
            % (
                self.published_clouds,
                len(voxels),
                self.filtered_dynamic_points,
            )
        )


def main():
    rospy.init_node("octomap_cloud_filter")
    OctomapCloudFilter()
    rospy.spin()


if __name__ == "__main__":
    main()
