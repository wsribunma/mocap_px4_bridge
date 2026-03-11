#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleLocalPosition, VehicleStatus
import time

class OffboardControl(Node):
    def __init__(self):
        super().__init__('offboard_node')

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Publishers
        self.offboard_pub = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', 10)
        self.trajectory_pub = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', 10)

        # Subscribers
        self.local_pos_sub = self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position', self.position_callback, qos_profile)
        self.status_sub = self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status', self.status_callback, qos_profile)

        # --- WAYPOINT MISSION PLAN ---
        # Format: [target_x, target_y, target_z, duration_in_seconds]
        # Remember: Z is negative for UP in NED frame.
        self.waypoints = [
            [0.0, 0.0, -1.0, 3.0],  # WP 0: Climb to 1m over 3 seconds
            [1.5, 0.0, -1.0, 4.0],  # WP 1: Move Forward 1.5m over 4 seconds
            [1.5, 1.5, -1.0, 4.0],  # WP 2: Move Right 1.5m over 4 seconds
            [0.0, 0.0, -1.0, 5.0]   # WP 3: Return to home over 5 seconds
        ]
        
        # State Variables
        self.current_pos = [0.0, 0.0, 0.0]
        self.start_pos = [0.0, 0.0, 0.0]
        self.nav_state = 0
        self.has_local_pos = False
        
        self.mission_started = False
        self.current_wp_index = 0
        self.wp_start_time = 0.0

        # 10Hz Timer
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.get_logger().info("Node Started. Waiting for Offboard mode switch...")

    def position_callback(self, msg):
        self.current_pos = [msg.x, msg.y, msg.z]
        self.has_local_pos = True

    def status_callback(self, msg):
        self.nav_state = msg.nav_state
        # nav_state 14 corresponds to Offboard mode in PX4
        if self.nav_state == 14 and not self.mission_started and self.has_local_pos:
            self.get_logger().info("Offboard Mode detected! Starting waypoint sequence.")
            self.mission_started = True
            self.start_next_waypoint()

        elif self.nav_state != 14 and self.mission_started:
            self.get_logger().warn("Pilot took back control! Mission paused.")
            self.mission_started = False
            self.current_wp_index = 0 # Reset mission (optional)

    def start_next_waypoint(self):
        if self.current_wp_index < len(self.waypoints):
            self.start_pos = list(self.current_pos)
            target = self.waypoints[self.current_wp_index]
            self.wp_start_time = time.time()
            self.get_logger().info(f"Moving to WP {self.current_wp_index}: {target[0:3]} over {target[3]}s")
        else:
            self.get_logger().info("Mission Complete. Holding final position.")

    def timer_callback(self):
        if not self.has_local_pos:
            return

        now = int(self.get_clock().now().nanoseconds / 1000)

        # 1. Maintain Offboard Heartbeat
        offboard_msg = OffboardControlMode()
        offboard_msg.position = True
        offboard_msg.velocity = True 
        offboard_msg.timestamp = now
        self.offboard_pub.publish(offboard_msg)

        # 2. Calculate Trajectory
        setpoint_msg = TrajectorySetpoint()
        setpoint_msg.timestamp = now
        setpoint_msg.yaw = 0.0

        if not self.mission_started:
            # If not in Offboard yet, just command it to hold exactly where it currently is.
            # This ensures a completely smooth transition when you flip the switch.
            setpoint_msg.position = self.current_pos
            setpoint_msg.velocity = [0.0, 0.0, 0.0]
        
        elif self.current_wp_index < len(self.waypoints):
            # We are actively executing the mission
            target = self.waypoints[self.current_wp_index]
            target_pos = target[0:3]
            duration = target[3]
            
            elapsed = time.time() - self.wp_start_time
            
            if elapsed >= duration:
                # Time is up for this waypoint! Move to the next one.
                self.current_wp_index += 1
                self.start_next_waypoint()
            else:
                # Interpolate
                fraction = elapsed / duration
                setpoint_msg.position = [
                    self.start_pos[i] + (target_pos[i] - self.start_pos[i]) * fraction for i in range(3)
                ]
                setpoint_msg.velocity = [
                    (target_pos[i] - self.start_pos[i]) / duration for i in range(3)
                ]
        else:
            # Mission finished, hold the final waypoint
            setpoint_msg.position = self.waypoints[-1][0:3]
            setpoint_msg.velocity = [0.0, 0.0, 0.0]

        self.trajectory_pub.publish(setpoint_msg)

def main(args=None):
    rclpy.init(args=args)
    node = OffboardControl()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()