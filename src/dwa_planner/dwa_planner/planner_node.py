#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point, PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker, MarkerArray
import math
import numpy as np
from tf_transformations import euler_from_quaternion
import random

class DWAPlanner(Node):
    def __init__(self):
        super().__init__('dwa_planner')
        
        # Robot parameters (TurtleBot3)
        self.max_vel_x = 0.22
        self.min_vel_x = -0.22
        self.max_vel_theta = 2.84
        self.max_accel_x = 2.5
        self.max_accel_theta = 3.2
        
        # DWA parameters
        self.vx_samples = 20
        self.vtheta_samples = 20
        self.dt = 0.1
        self.predict_time = 1.5
        self.goal_tolerance = 0.1
        
        # Cost function weights
        self.obstacle_weight = 0.4
        self.goal_weight = 0.3
        self.velocity_weight = 0.2
        self.smoothness_weight = 0.1
        
        # Current state
        self.current_pose = None
        self.current_velocity = [0.0, 0.0]
        self.scan_data = None
        self.goal_point = None
        
        # Publishers and Subscribers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/trajectory_markers', 10)
        
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        
        self.goal_sub = self.create_subscription(PoseStamped, '/goal_pose', self.goal_callback, 10)
        
        # Control timer
        self.control_timer = self.create_timer(0.1, self.control_loop)
        
        self.get_logger().info("Custom DWA Planner initialized")
        self.odom_received = False
        self.scan_received = False
        self.goal_received = False

    def odom_callback(self, msg):
        """Update current pose and velocity from odometry"""
        self.current_pose = msg.pose.pose
        self.current_velocity[0] = msg.twist.twist.linear.x
        self.current_velocity[1] = msg.twist.twist.angular.z
        if not self.odom_received:
            self.odom_received = True
            self.get_logger().info(f"Odom data received from topic: {self.odom_sub.topic_name}")


    def scan_callback(self, msg):
        """Update laser scan data"""
        self.scan_data = msg
        if not self.scan_received:
            self.scan_received = True
            self.get_logger().info(f"Laser scan received from topic: {self.scan_sub.topic_name}")


    def goal_callback(self, msg):
        """Update goal position"""
        self.goal_point = msg.pose.position
        if not self.goal_received:
            self.goal_received = True
            self.get_logger().info(f"Goal received from topic: {self.goal_sub.topic_name}") 
        self.get_logger().info(f"New goal received: ({self.goal_point.x}, {self.goal_point.y})")

    def generate_velocity_samples(self, vx_current, vtheta_current):
        """Generate velocity samples within dynamic window"""
        vx_samples = np.linspace(
            max(self.min_vel_x, vx_current - self.max_accel_x * self.dt),
            min(self.max_vel_x, vx_current + self.max_accel_x * self.dt),
            self.vx_samples
        )
        
        vtheta_samples = np.linspace(-self.max_vel_theta, self.max_vel_theta, self.vtheta_samples)

        
        return vx_samples, vtheta_samples
    
    def quaternion_to_yaw(self, q):
        _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
        return yaw

    def predict_trajectory(self, vx, vtheta, x, y, yaw):
        """
        Correct trajectory prediction:
        - yaw updates every step
        - returns full list of (x,y)
        """
        trajectory = []
        px, py, pyaw = x, y, yaw

        steps = int(self.predict_time / self.dt)
        for _ in range(steps):
            px += vx * math.cos(pyaw) * self.dt
            py += vx * math.sin(pyaw) * self.dt
            pyaw += vtheta * self.dt
            trajectory.append((px, py))

        return trajectory, pyaw


    def calculate_obstacle_cost(self, trajectory):
        if self.scan_data is None:
            return 0.0

        ranges = np.array(self.scan_data.ranges)
        angle_min = self.scan_data.angle_min
        angle_inc = self.scan_data.angle_increment

        min_dist = float('inf')

        # Precompute obstacles in global frame
        obstacles = []
        for i, r in enumerate(ranges):
            if np.isinf(r) or np.isnan(r):
                continue
            
            angle = angle_min + i * angle_inc
            ox = self.current_pose.position.x + r * math.cos(angle)
            oy = self.current_pose.position.y + r * math.sin(angle)
            obstacles.append((ox, oy))

        if not obstacles:
            return 0.0

        # Compute trajectory → obstacle clearance
        for (px, py) in trajectory:
            for (ox, oy) in obstacles:
                d = math.sqrt((px - ox)**2 + (py - oy)**2)
                min_dist = min(min_dist, d)

        if min_dist < 0.15:
            return 1e6

        return 1.0 / (min_dist + 0.01)

    def calculate_goal_cost(self, trajectory, end_yaw):
        """
        Goal cost = distance to goal + heading error
        """
        if self.goal_point is None or len(trajectory) == 0:
            return 0.0

        # End point of predicted trajectory
        end_x, end_y = trajectory[-1]

        # --- distance to goal ---
        dx = self.goal_point.x - end_x
        dy = self.goal_point.y - end_y
        dist_cost = math.sqrt(dx*dx + dy*dy)

        # --- heading alignment ---
        goal_yaw = math.atan2(dy, dx)

        heading_error = abs(math.atan2(
            math.sin(goal_yaw - end_yaw),
            math.cos(goal_yaw - end_yaw)
        ))

        return dist_cost + 1.5 * heading_error


    def calculate_velocity_cost(self, vx, vtheta):
        # Penalize backward motion heavily
        if vx < 0:
            return 1000.0

        # Encourage forward velocity
        return (self.max_vel_x - vx) / self.max_vel_x

    def calculate_smoothness_cost(self, vx, vtheta, prev_vx, prev_vtheta):
        """Calculate cost based on smoothness of control"""
        return abs(vx - prev_vx) / (2 * self.max_vel_x) + abs(vtheta - prev_vtheta) / (2 * self.max_vel_theta)

    def evaluate_trajectory(self, trajectory, vx, vtheta, end_yaw):
        """Evaluate trajectory using cost function"""
        trajectory, end_yaw = self.predict_trajectory(vx, vtheta, 
                                                    self.current_pose.position.x,
                                                    self.current_pose.position.y,
                                                    self.quaternion_to_yaw(self.current_pose.orientation))

        obstacle_cost = self.calculate_obstacle_cost(trajectory)
        goal_cost = self.calculate_goal_cost(trajectory, end_yaw)
        velocity_cost = self.calculate_velocity_cost(vx, vtheta)
        smoothness_cost = self.calculate_smoothness_cost(vx, vtheta, 
                                                        self.current_velocity[0], 
                                                        self.current_velocity[1])
        
        total_cost = (self.obstacle_weight * obstacle_cost +
                     self.goal_weight * goal_cost +
                     self.velocity_weight * velocity_cost +
                     self.smoothness_weight * smoothness_cost)
        
        return total_cost

    def control_loop(self):
        """Main control loop"""
        if not self.current_pose or not self.scan_data or not self.goal_point:
            missing_data = []
            if not self.current_pose: missing_data.append("odom")
            if not self.scan_data: missing_data.append("scan") 
            if not self.goal_point: missing_data.append("goal")
            if len(missing_data) > 0:
                self.get_logger().info(f"Waiting for: {', '.join(missing_data)}")
            return
            
        # Check if goal is reached
        current_x = self.current_pose.position.x
        current_y = self.current_pose.position.y
        q = self.current_pose.orientation
        current_theta = math.atan2(2*(q.w*q.z + q.x*q.y),
                                1 - 2*(q.y*q.y + q.z*q.z))

        distance_to_goal = math.sqrt(
            (current_x - self.goal_point.x)**2 + 
            (current_y - self.goal_point.y)**2
        )
        
        if distance_to_goal < self.goal_tolerance:
            # Stop the robot
            stop_cmd = Twist()
            self.cmd_vel_pub.publish(stop_cmd)
            self.get_logger().info("Goal reached!")
            return
        
        # Generate velocity samples
        vx_samples, vtheta_samples = self.generate_velocity_samples(
            self.current_velocity[0], self.current_velocity[1]
        )
        
        best_cost = float('inf')
        best_vx = 0.0
        best_vtheta = 0.0
        self.last_vx = 0.0
        
        # Evaluate all velocity samples
        for vx in vx_samples:
            for vtheta in vtheta_samples:
                trajectory, end_yaw = self.predict_trajectory(vx, vtheta, current_x, current_y, current_theta)
                cost = self.evaluate_trajectory(trajectory, vx, vtheta, end_yaw)
                
                if cost < best_cost:
                    best_cost = cost
                    best_vx = vx
                    best_vtheta = vtheta
                    # --- Oscillation detection ---
                    if abs(best_vx) < 0.03 and abs(self.current_velocity[0]) > 0.05:
                        best_vx = 0.0
                        best_vtheta = random.choice([-0.8, 0.8])
                        self.get_logger().warn("Oscillation detected — forcing rotation!")

                    self.last_vx = best_vx

        
        # Publish best velocity command
        cmd_vel = Twist()
        cmd_vel.linear.x = best_vx
        cmd_vel.angular.z = best_vtheta
        self.cmd_vel_pub.publish(cmd_vel)
        
        self.get_logger().info(f"Best velocity: vx={best_vx:.2f}, vtheta={best_vtheta:.2f}, cost={best_cost:.2f}, distance_to_goal={distance_to_goal:.2f}")

def main():
    rclpy.init()
    node = DWAPlanner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard interrupt, shutting down")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()