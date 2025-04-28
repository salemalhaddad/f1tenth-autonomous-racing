#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped

class DisparityExtender:
    CAR_WIDTH = 0.31
    DIFFERENCE_THRESHOLD = 2.0
    SPEED = 5.0
    SAFETY_PERCENTAGE = 300.0

    def preprocess_lidar(self, ranges):
        eighth = int(len(ranges) / 8)
        return np.array(ranges[eighth:-eighth])

    def get_differences(self, ranges):
        differences = [0.]
        for i in range(1, len(ranges)):
            differences.append(abs(ranges[i] - ranges[i-1]))
        return differences

    def get_disparities(self, differences, threshold):
        disparities = []
        for index, difference in enumerate(differences):
            if difference > threshold:
                disparities.append(index)
        return disparities

    def get_num_points_to_cover(self, dist, width):
        angle = 2 * np.arcsin(width / (2 * dist))
        num_points = int(np.ceil(angle / self.radians_per_point))
        return num_points

    def cover_points(self, num_points, start_idx, cover_right, ranges):
        new_dist = ranges[start_idx]
        if cover_right:
            for i in range(num_points):
                next_idx = start_idx + 1 + i
                if next_idx >= len(ranges):
                    break
                if ranges[next_idx] > new_dist:
                    ranges[next_idx] = new_dist
        else:
            for i in range(num_points):
                next_idx = start_idx - 1 - i
                if next_idx < 0:
                    break
                if ranges[next_idx] > new_dist:
                    ranges[next_idx] = new_dist
        return ranges

    def extend_disparities(self, disparities, ranges, car_width, extra_pct):
        width_to_cover = (car_width / 2) * (1 + extra_pct / 100)
        for index in disparities:
            first_idx = index - 1
            points = ranges[first_idx:first_idx + 2]
            close_idx = first_idx + np.argmin(points)
            far_idx = first_idx + np.argmax(points)
            close_dist = ranges[close_idx]
            num_points_to_cover = self.get_num_points_to_cover(close_dist, width_to_cover)
            cover_right = close_idx < far_idx
            ranges = self.cover_points(num_points_to_cover, close_idx, cover_right, ranges)
        return ranges

    def process_lidar(self, ranges):
        self.radians_per_point = (2 * np.pi) / len(ranges)
        proc_ranges = self.preprocess_lidar(ranges)
        differences = self.get_differences(proc_ranges)
        disparities = self.get_disparities(differences, self.DIFFERENCE_THRESHOLD)
        proc_ranges = self.extend_disparities(disparities, proc_ranges, self.CAR_WIDTH, self.SAFETY_PERCENTAGE)
        return proc_ranges


class SimpleGapFollower(Node):
    def __init__(self):
        super().__init__('simple_gap_follower')
        # Constants
        self.sector_deg = 70  # look ±35°
        self.max_speed = 1.45
        self.min_speed = 1.0

        # Create publisher
        self.pub = self.create_publisher(AckermannDriveStamped, '/drive', 10)
        # Create subscriber
        self.create_subscription(LaserScan, '/scan', self.scan_cb, 10)

        # Instantiate DisparityExtender
        self.disparity_extender = DisparityExtender()

        self.get_logger().info('SimpleGapFollower with DisparityExtender ready.')

    def scan_cb(self, scan: LaserScan):
        # 1) Clean up
        r = np.nan_to_num(scan.ranges, nan=scan.range_max, posinf=scan.range_max)

        # 2) Sector indices
        a_min, a_inc = scan.angle_min, scan.angle_increment
        half = np.deg2rad(self.sector_deg)
        margin = 160
        i_min = 570 - margin
        i_max = 570 + margin
        sector = r[i_min:i_max + 1]

        if len(sector) == 0:
            return

        # Use DisparityExtender
        sector_proc = self.disparity_extender.process_lidar(sector)

        bub = 200
        # Mask the closest obstacle with a bubble
        closest_rel_idx = np.argmin(sector_proc)
        closest_abs_idx = i_min + closest_rel_idx

        bubble_start = max(i_min, closest_abs_idx - bub)
        bubble_end = min(i_max, closest_abs_idx + bub)

        sector_masked = sector_proc.copy()
        mask_start = bubble_start - i_min
        mask_end = bubble_end - i_min
        sector_masked[mask_start:mask_end + 1] = 0.0

        # Find best direction
        best_rel_idx = np.argmax(sector_masked)
        best_abs_idx = i_min + best_rel_idx

        if sector_proc[best_rel_idx] < 0.2:
            self.get_logger().info("CRASHED INTO WALL")
            return

        # === FIXED EDGE STICKING ===
        if best_abs_idx <= i_min + 5:
            self.get_logger().info("Best ray near LEFT EDGE → sticking left.")
        elif best_abs_idx >= i_max - 5:
            self.get_logger().info("Best ray near RIGHT EDGE → sticking right.")

        # Pick the best ray in the remaining sector
        rel_best = np.argmax(sector_masked)
        best_i = rel_best + i_min
        steer = a_min + best_i * a_inc

        self.get_logger().info(f"Best ray: {best_i} | Steer: {steer:.2f} rad")

        # Adjust speed based on edge sticking
        margin = 20
        if best_i <= i_min + margin or best_i >= i_max - margin:
            speed = self.min_speed * 0.6
        else:
            speed = self.max_speed

        # Publish drive command
        drive_msg = AckermannDriveStamped()
        drive_msg.drive.steering_angle = steer
        drive_msg.drive.speed = speed
        self.pub.publish(drive_msg)

def main(args=None):
    rclpy.init(args=args)
    node = SimpleGapFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
