#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from geometry_msgs.msg import PoseStamped
from px4_msgs.msg import VehicleOdometry
import math

class MocapPx4Bridge(Node):
    def __init__(self):
        super().__init__('mocap_px4_bridge')

        # Declare Parameters
        self.declare_parameter("vehicle_id", "/pv162")


        # Configure QoS profile for publishing and subscribing
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Publisher: Send to PX4 via uXRCE-DDS
        self.odom_pub = self.create_publisher(
            VehicleOdometry, 
            '/fmu/in/vehicle_visual_odometry', 
            qos_profile
        )

        # Subscriber: Listen to Qualisys Mocap (Change topic name as needed!)
        self.pose_topic = self.get_parameter("vehicle_id").get_parameter_value().string_value + '/pose'
        self.pose_sub = self.create_subscription(
            PoseStamped, 
            self.pose_topic, 
            self.pose_callback, 
            qos_profile
        )
        self.get_logger().info(f"Subscribed to Mocap Pose topic: {self.pose_topic}")

        self.get_logger().info("Mocap to PX4 Bridge Node Started.")

    def pose_callback(self, msg: PoseStamped):
        out_msg = VehicleOdometry()

        # Timestamps
        # current_time = int(self.get_clock().now().nanoseconds / 1000)
        out_msg = VehicleOdometry()

        # 1. Use the Zero-Timestamp Trick (Crucial for Mavlink Inspector to show it)
        out_msg.timestamp = 0 
        out_msg.timestamp_sample = 0

        # 2. Frame IDs (MUST be set for 1.16.1)
        out_msg.pose_frame = VehicleOdometry.POSE_FRAME_NED
        out_msg.velocity_frame = VehicleOdometry.VELOCITY_FRAME_UNKNOWN

        # 3. Coordinate Swizzle (ENU to NED)
        out_msg.position[0] =  msg.pose.position.y # North
        out_msg.position[1] =  msg.pose.position.x # East
        out_msg.position[2] = -msg.pose.position.z # Down (Negative of Mocap Z)

        # 4. Orientation Swizzle (ENU to NED)
        out_msg.q[0] =  msg.pose.orientation.w
        out_msg.q[1] =  msg.pose.orientation.y
        out_msg.q[2] =  msg.pose.orientation.x
        out_msg.q[3] = -msg.pose.orientation.z

        # 5. Populate Variances (EKF won't trust 0.0 variance)
        out_msg.position_variance = [0.01, 0.01, 0.01]
        out_msg.orientation_variance = [0.02, 0.02, 0.02]

        self.odom_pub.publish(out_msg)

def main(args=None):
    rclpy.init(args=args)
    node = MocapPx4Bridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()