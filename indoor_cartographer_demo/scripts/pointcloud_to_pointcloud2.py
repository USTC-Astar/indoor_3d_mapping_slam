#!/usr/bin/env python3

import math

import rospy
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud, PointCloud2


class PointCloudToPointCloud2:
    def __init__(self):
        self.input_topic = rospy.get_param("~input_topic", "/block_laser_3d")
        self.output_topic = rospy.get_param("~output_topic", "/points2")
        self.min_range = rospy.get_param("~min_range", 0.18)
        self.max_range = rospy.get_param("~max_range", 8.0)
        self.max_range_margin = rospy.get_param("~max_range_margin", 0.0)
        self.min_z = rospy.get_param("~min_z", float("-inf"))
        self.max_z = rospy.get_param("~max_z", float("inf"))
        self.publisher = rospy.Publisher(
            self.output_topic, PointCloud2, queue_size=1
        )
        self.subscriber = rospy.Subscriber(
            self.input_topic, PointCloud, self._callback, queue_size=1
        )

    def _callback(self, msg):
        points = []
        min_range_sq = self.min_range * self.min_range
        max_range_sq = self.max_range * self.max_range
        max_range_cutoff = self.max_range - max(0.0, self.max_range_margin)
        for point in msg.points:
            if not (
                math.isfinite(point.x)
                and math.isfinite(point.y)
                and math.isfinite(point.z)
            ):
                continue
            if point.z < self.min_z or point.z > self.max_z:
                continue
            range_sq = point.x * point.x + point.y * point.y + point.z * point.z
            if range_sq < min_range_sq or range_sq > max_range_sq:
                continue
            if self.max_range_margin > 0.0 and math.sqrt(range_sq) >= max_range_cutoff:
                continue
            points.append((point.x, point.y, point.z))

        if not points:
            return

        cloud = point_cloud2.create_cloud_xyz32(msg.header, points)
        self.publisher.publish(cloud)


def main():
    rospy.init_node("pointcloud_to_pointcloud2")
    PointCloudToPointCloud2()
    rospy.spin()


if __name__ == "__main__":
    main()
