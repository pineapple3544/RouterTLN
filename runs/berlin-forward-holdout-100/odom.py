"""Capture ground-truth odometry for the ROS obstacle clearance measurement."""
import csv
import math
import sys

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data


def main():
    path = sys.argv[1]
    rclpy.init(args=[])
    node = Node("clearance_logger")
    with open(path, "x", newline="") as stream:
        writer = csv.writer(stream)
        writer.writerow(("stamp","x","y","yaw"))
        def on_odom(msg):
            pose = msg.pose.pose
            q = pose.orientation
            yaw = math.atan2(2*(q.w*q.z + q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
            stamp = msg.header.stamp
            writer.writerow((stamp.sec + 1e-9*stamp.nanosec, pose.position.x, pose.position.y, yaw))
        node.create_subscription(Odometry, "/car_state/odom_GT", on_odom, qos_profile_sensor_data)
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            pass
        finally:
            node.destroy_node()
            rclpy.try_shutdown()


if __name__ == "__main__":
    main()
