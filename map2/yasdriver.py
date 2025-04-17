#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import PoseStamped
import math
import time

class F1TenthDriver(Node):
    """
    A comprehensive driver for F1Tenth that combines safety, wall following,
    and reactive gap following to navigate the Yas Marina track.
    """
    
    def __init__(self):
        super().__init__('f1tenth_driver')
        
        # Publishers
        self.drive_publisher = self.create_publisher(
            AckermannDriveStamped,
            '/drive',
            10
        )
        
        # Subscribers
        self.scan_subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )
        
        self.odom_subscription = self.create_subscription(
            Odometry,
            '/ego_racecar/odom',
            self.odom_callback,
            10
        )
        
        # Initialize state variables
        self.current_speed = 0.0
        self.current_position = [0.0, 0.0]
        self.current_orientation = 0.0
        self.last_time = time.time()
        
        # PID control parameters for wall following
        self.kp = 1.0  # Proportional gain (increased for more responsive steering)
        self.kd = 0.1  # Derivative gain
        self.ki = 0.01  # Integral gain
        
        # PID history
        self.error_integral = 0.0
        self.prev_error = 0.0
        
        # Safety parameters
        self.safety_threshold = 0.5  # Time threshold (in seconds) before collision
        
        # Track following parameters
        self.target_distance_from_wall = 0.8  # Target distance from wall (meters)
        self.follow_left_wall = True  # Whether to follow left or right wall (will switch as needed)
        
        # Reactive gap parameters
        self.bubble_radius = 100  # Radius around obstacles (in points)
        
        # Drive parameters
        self.max_speed = 3.0  # Maximum speed
        self.min_speed = 0.5   # Minimum speed
        self.max_steering_angle = 0.4  # Maximum steering angle
        
        # Racing line waypoints (pre-computed optimal path coordinates)
        # These would ideally be more precise points from the racing line
        self.get_logger().info('F1Tenth Driver initialized for Yas Marina track!')
    
    def odom_callback(self, odom_msg):
        """
        Update position, orientation, and speed from odometry data
        """
        # Extract position
        self.current_position = [
            odom_msg.pose.pose.position.x,
            odom_msg.pose.pose.position.y
        ]
        
        # Extract orientation (yaw) - converting quaternion to euler ourselves
        # Formula to convert quaternion to yaw (z-axis rotation)
        orientation_q = odom_msg.pose.pose.orientation
        x, y, z, w = orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w
        
        # Calculate yaw (z-axis rotation) from quaternion
        # This is equivalent to euler_from_quaternion but doesn't require the tf_transformations package
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        
        self.current_orientation = yaw
        
        # Extract speed
        self.current_speed = odom_msg.twist.twist.linear.x
    
    def scan_callback(self, scan_msg):
        """
        Main callback processing laser scan data to control the car
        """
        # Check safety first (emergency braking)
        if not self.check_safety(scan_msg):
            return
        
        # Process scan data for navigation
        self.process_scan_and_drive(scan_msg)
    
    def check_safety(self, scan_msg):
        """
        Safety check to prevent collisions (from Lab 4)
        Returns True if safe, False if emergency braking needed
        """
        ranges = scan_msg.ranges
        num_rays = len(ranges)
        angle_min = scan_msg.angle_min
        angle_increment = scan_msg.angle_increment
        v = self.current_speed
        
        # Look at frontal rays (within ±30 degrees)
        front_indices = [i for i in range(num_rays) if abs(angle_min + i * angle_increment) < np.deg2rad(30)]
        
        for i in front_indices:
            # Calculate angle for current ray
            angle = angle_min + i * angle_increment
            
            # Compute longitudinal velocity component
            v_long = v * np.cos(angle)
            
            # Only consider rays where we're moving toward the obstacle
            if v_long > 0:
                # Compute time to collision (iTTC) with safety against division by zero
                iTTC = ranges[i] / max(v_long, 1e-5)
                
                # If collision is imminent, apply emergency braking
                if iTTC < self.safety_threshold:
                    self.emergency_brake()
                    self.get_logger().warning(
                        f'Emergency braking! iTTC: {iTTC:.2f} seconds, Speed: {v:.2f} m/s'
                    )
                    return False
        
        return True
    
    def emergency_brake(self):
        """
        Emergency braking function
        """
        drive_msg = AckermannDriveStamped()
        drive_msg.drive.speed = 0.0
        self.drive_publisher.publish(drive_msg)
    
    def process_scan_and_drive(self, scan_msg):
        """
        Combined algorithm using both wall following and reactive gap following
        """
        ranges = np.array(scan_msg.ranges)
        num_rays = len(ranges)
        angle_min = scan_msg.angle_min
        angle_increment = scan_msg.angle_increment
        
        # Clean the scan data (replace NaN and Inf with appropriate values)
        for i in range(num_rays):
            if np.isinf(ranges[i]) or np.isnan(ranges[i]):
                ranges[i] = scan_msg.range_max
        
        # Determine if we should follow left or right wall based on track curvature
        # This simplified approach uses the difference between left and right distances
        left_avg = np.mean(ranges[int(num_rays*0.7):int(num_rays*0.8)])  # Left side rays
        right_avg = np.mean(ranges[int(num_rays*0.2):int(num_rays*0.3)])  # Right side rays
        
        # Switch wall following side when appropriate (when one side has much more space)
        if left_avg > right_avg * 1.5 and self.follow_left_wall:
            self.follow_left_wall = False
            self.get_logger().info('Switching to follow right wall')
        elif right_avg > left_avg * 1.5 and not self.follow_left_wall:
            self.follow_left_wall = True
            self.get_logger().info('Switching to follow left wall')
        
        # Get approximate indices for 0°, 45° left, and 45° right
        center_idx = num_rays // 2
        left_idx = int(center_idx * 1.25)
        right_idx = int(center_idx * 0.75)
        
        # Ensure indices are in range
        left_idx = min(left_idx, num_rays - 1)
        right_idx = max(right_idx, 0)
        
        # Choose which side to follow
        if self.follow_left_wall:
            a_idx = left_idx
            b_idx = center_idx
        else:
            a_idx = center_idx
            b_idx = right_idx
        
        # Extract beam measurements for wall following
        a = ranges[a_idx]
        b = ranges[b_idx]
        
        # Calculate the angle between the two points
        theta = abs(a_idx - b_idx) * angle_increment
        
        # Wall following error calculation
        future_distance = self.get_range([a, b], theta)
        error = future_distance - self.target_distance_from_wall
        
        # Check if there's an obstacle that needs reactive approach
        # Find the closest obstacle in the front area
        front_start = int(num_rays * 0.4)
        front_end = int(num_rays * 0.6)
        front_ranges = ranges[front_start:front_end]
        
        if np.min(front_ranges) < 1.5:  # If obstacle is close
            # Use reactive gap following for this section
            steering_angle = self.find_best_gap(ranges, angle_min, angle_increment)
        else:
            # Use PID wall following
            steering_angle = self.pid_control(error)
        
        # Publish drive command
        self.publish_drive_command(steering_angle)
    
    def get_range(self, range_data, angle):
        """
        Calculate the predicted distance to the wall
        Adapted from Lab 5
        """
        alpha = np.arctan(
            (range_data[0] * np.cos(angle) - range_data[1]) /
            (range_data[0] * np.sin(angle))
        )
        
        Dt = range_data[1] * np.cos(angle)
        L = 0.5  # Look-ahead distance
        Dt1 = Dt + L * np.sin(alpha)
        
        return Dt1
    
    def pid_control(self, error):
        """
        PID control for wall following
        Adapted from Lab 5
        """
        current_time = time.time()
        delta_time = current_time - self.last_time
        self.last_time = current_time
        
        # Update integral and handle windup
        self.error_integral += error * delta_time
        self.error_integral = np.clip(self.error_integral, -5.0, 5.0)  # Prevent windup
        
        # Calculate PID terms
        p_term = self.kp * error
        i_term = self.ki * self.error_integral
        d_term = self.kd * (error - self.prev_error) / max(delta_time, 0.001)
        
        # Calculate steering angle
        angle = p_term + i_term + d_term
        
        # Clamp steering angle to valid range
        angle = np.clip(angle, -self.max_steering_angle, self.max_steering_angle)
        
        # Update previous error
        self.prev_error = error
        
        return angle
    
    def find_best_gap(self, ranges, angle_min, angle_increment):
        """
        Find the best gap using the reactive gap following approach
        Adapted from Lab 6
        """
        # Get front 180 degrees of scan
        num_points = len(ranges)
        center_idx = num_points // 2
        start_idx = center_idx - int(num_points * 0.25)
        end_idx = center_idx + int(num_points * 0.25)
        
        # Ensure indices are within range
        start_idx = max(0, start_idx)
        end_idx = min(num_points, end_idx)
        
        # Extract relevant ranges
        relevant_ranges = ranges[start_idx:end_idx]
        
        # Find the closest point
        min_idx = np.argmin(relevant_ranges) + start_idx
        min_dist = ranges[min_idx]
        
        # Create a bubble around the closest obstacle
        bubble_radius_scaled = min(self.bubble_radius, int(num_points * 0.1))
        
        # Create a copy of ranges to modify
        processed_ranges = np.copy(ranges)
        
        # Set the bubble area to zero (treated as obstacles)
        for i in range(max(0, min_idx - bubble_radius_scaled), min(num_points, min_idx + bubble_radius_scaled + 1)):
            processed_ranges[i] = 0.0
        
        # Find the largest gap
        gaps = []
        in_gap = False
        gap_start = 0
        
        for i in range(start_idx, end_idx):
            if processed_ranges[i] > 0.5 and not in_gap:
                in_gap = True
                gap_start = i
            elif (processed_ranges[i] <= 0.5 or i == end_idx - 1) and in_gap:
                in_gap = False
                gap_end = i
                gap_width = gap_end - gap_start
                gap_center = gap_start + gap_width // 2
                furthest_point = np.argmax(processed_ranges[gap_start:gap_end]) + gap_start
                gaps.append((gap_width, gap_center, furthest_point))
        
        if not gaps:
            # If no gaps found, steer toward the longest range
            best_idx = np.argmax(processed_ranges[start_idx:end_idx]) + start_idx
        else:
            # Sort gaps by width (largest first)
            gaps.sort(reverse=True)
            # Choose the center of the largest gap
            _, _, best_idx = gaps[0]
        
        # Calculate the steering angle
        steering_angle = angle_min + best_idx * angle_increment
        
        # Normalize steering angle
        if steering_angle > np.pi:
            steering_angle -= 2 * np.pi
        elif steering_angle < -np.pi:
            steering_angle += 2 * np.pi
            
        return steering_angle
    
    def publish_drive_command(self, steering_angle):
        """
        Publish drive command with speed based on steering angle
        """
        drive_msg = AckermannDriveStamped()
        drive_msg.drive.steering_angle = steering_angle
        
        # Adjust speed based on steering angle - slower in turns, faster on straights
        speed_factor = 1.0 - (abs(steering_angle) / self.max_steering_angle) * 0.7
        drive_msg.drive.speed = self.min_speed + speed_factor * (self.max_speed - self.min_speed)
        
        # If we're turning sharply, reduce speed more
        if abs(steering_angle) > self.max_steering_angle * 0.8:
            drive_msg.drive.speed = self.min_speed
        
        self.drive_publisher.publish(drive_msg)

def main(args=None):
    rclpy.init(args=args)
    f1tenth_driver = F1TenthDriver()
    
    try:
        rclpy.spin(f1tenth_driver)
    except KeyboardInterrupt:
        pass
    finally:
        # Perform clean shutdown
        stop_msg = AckermannDriveStamped()
        stop_msg.drive.speed = 0.0
        f1tenth_driver.drive_publisher.publish(stop_msg)
        f1tenth_driver.get_logger().info('Stopping vehicle and shutting down')
        f1tenth_driver.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()