#!/usr/bin/env python3

import math

import rospy
import tf2_ros
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Bool
from std_msgs.msg import Header


def clamp(value, low, high):
    return max(low, min(high, value))


def rotate_vector(q, point):
    x, y, z = point
    qx = q.x
    qy = q.y
    qz = q.z
    qw = q.w

    tx = 2.0 * (qy * z - qz * y)
    ty = 2.0 * (qz * x - qx * z)
    tz = 2.0 * (qx * y - qy * x)

    return (
        x + qw * tx + (qy * tz - qz * ty),
        y + qw * ty + (qz * tx - qx * tz),
        z + qw * tz + (qx * ty - qy * tx),
    )


class PointCloudMapAccumulator:
    def __init__(self):
        self.input_topic = rospy.get_param("~input_topic", "/points2")
        self.output_topic = rospy.get_param("~output_topic", "/map_points")
        self.target_frame = rospy.get_param("~target_frame", "map")
        self.voxel_size = rospy.get_param("~voxel_size", 0.08)
        self.publish_rate = rospy.get_param("~publish_rate", 1.0)
        self.max_points = int(rospy.get_param("~max_points", 220000))
        self.min_z = rospy.get_param("~min_z", -0.10)
        self.max_z = rospy.get_param("~max_z", 2.80)
        self.min_x = self.optional_float_param("~min_x")
        self.max_x = self.optional_float_param("~max_x")
        self.min_y = self.optional_float_param("~min_y")
        self.max_y = self.optional_float_param("~max_y")
        self.allow_latest_tf_fallback = rospy.get_param("~allow_latest_tf_fallback", False)
        self.freeze_on_completion = rospy.get_param("~freeze_on_completion", False)
        self.completion_topic = rospy.get_param("~completion_topic", "/active_slam/completed")
        self.frozen = False

        self.points = {}
        self.tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(30.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.publisher = rospy.Publisher(self.output_topic, PointCloud2, queue_size=1, latch=True)
        self.subscriber = rospy.Subscriber(
            self.input_topic, PointCloud2, self.cloud_callback, queue_size=1
        )
        if self.freeze_on_completion:
            self.completion_subscriber = rospy.Subscriber(
                self.completion_topic, Bool, self.completion_callback, queue_size=1
            )
        rospy.Timer(rospy.Duration(1.0 / max(0.1, self.publish_rate)), self.publish)

    def optional_float_param(self, name):
        value = rospy.get_param(name, None)
        if value is None or value == "":
            return None
        return float(value)

    def voxel_key(self, x, y, z):
        size = self.voxel_size
        return (
            int(math.floor(x / size)),
            int(math.floor(y / size)),
            int(math.floor(z / size)),
        )

    def lookup_transform(self, stamp, frame_id):
        if stamp != rospy.Time(0):
            try:
                return self.tf_buffer.lookup_transform(
                    self.target_frame, frame_id, stamp, rospy.Duration(0.12)
                )
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                    tf2_ros.ExtrapolationException):
                if not self.allow_latest_tf_fallback:
                    raise
        return self.tf_buffer.lookup_transform(
            self.target_frame, frame_id, rospy.Time(0), rospy.Duration(0.08)
        )

    def transform_point(self, transform, point):
        rotated = rotate_vector(transform.transform.rotation, point)
        translation = transform.transform.translation
        return (
            rotated[0] + translation.x,
            rotated[1] + translation.y,
            rotated[2] + translation.z,
        )

    def completion_callback(self, msg):
        if msg.data and not self.frozen:
            self.frozen = True
            rospy.loginfo(
                "Freezing accumulated 3D map at %d voxels after exploration completion",
                len(self.points),
            )

    def in_bounds(self, x, y, z):
        if z < self.min_z or z > self.max_z:
            return False
        if self.min_x is not None and x < self.min_x:
            return False
        if self.max_x is not None and x > self.max_x:
            return False
        if self.min_y is not None and y < self.min_y:
            return False
        if self.max_y is not None and y > self.max_y:
            return False
        return True

    def cloud_callback(self, msg):
        if self.frozen:
            return
        if not msg.header.frame_id:
            return

        try:
            transform = self.lookup_transform(msg.header.stamp, msg.header.frame_id)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as exc:
            rospy.logwarn_throttle(2.0, "Waiting for %s -> %s transform: %s",
                                   msg.header.frame_id, self.target_frame, exc)
            return

        added = 0
        for point in point_cloud2.read_points(
                msg, field_names=("x", "y", "z"), skip_nans=True):
            x, y, z = self.transform_point(transform, point)
            if not (math.isfinite(x) and math.isfinite(y) and math.isfinite(z)):
                continue
            if not self.in_bounds(x, y, z):
                continue
            intensity = clamp((z - self.min_z) / max(0.01, self.max_z - self.min_z), 0.0, 1.0)
            self.points[self.voxel_key(x, y, z)] = (x, y, z, intensity)
            added += 1

        while len(self.points) > self.max_points:
            self.points.pop(next(iter(self.points)))

        if added:
            rospy.loginfo_throttle(
                5.0, "Accumulated 3D map points: %d voxels", len(self.points)
            )

    def publish(self, _event):
        if not self.points:
            return
        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = self.target_frame
        fields = [
            PointField("x", 0, PointField.FLOAT32, 1),
            PointField("y", 4, PointField.FLOAT32, 1),
            PointField("z", 8, PointField.FLOAT32, 1),
            PointField("intensity", 12, PointField.FLOAT32, 1),
        ]
        self.publisher.publish(
            point_cloud2.create_cloud(header, fields, list(self.points.values()))
        )


def main():
    rospy.init_node("pointcloud_map_accumulator")
    PointCloudMapAccumulator()
    rospy.spin()


if __name__ == "__main__":
    main()
