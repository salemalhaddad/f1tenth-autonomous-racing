#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped

class SimpleGapFollower(Node):
    def __init__(self):
        super().__init__('simple_gap_follower')
        # Constants you can tweak
        self.sector_deg = 70  # look ±35°
        self.max_speed = 1.45  # top speed (m/s)
        self.min_speed = 1.0   # slowest (m/s)

        self.pub = self.create_publisher(AckermannDriveStamped, '/drive', 10)
        self.create_subscription(LaserScan, '/scan', self.scan_cb, 10)
        self.get_logger().info('SimpleGapFollower ready.')

    def disparity_extender(self, ranges, car_width=0.31, threshold=2.0, safety_pct=300.0):
        """Extends disparities in LIDAR data so the car doesn't try to drive through gaps too small for its width."""
        ranges = np.array(ranges)
        radians_per_point = (2 * np.pi) / len(ranges)
        width_to_cover = (car_width / 2) * (1 + safety_pct / 100)
        for i in range(len(ranges) - 1):
            diff = abs(ranges[i + 1] - ranges[i])
            if diff > threshold:
                close_idx = i if ranges[i] < ranges[i + 1] else i + 1
                close_dist = ranges[close_idx]
                if close_dist == 0:  # avoid division by zero
                    continue
                angle = 2 * np.arcsin(min(1.0, width_to_cover / (2 * close_dist)))
                num_points = int(np.ceil(angle / radians_per_point))
                if close_idx == i:
                    start = close_idx + 1
                    end = min(len(ranges), close_idx + 1 + num_points)
                    ranges[start:end] = close_dist
                else:
                    start = max(0, close_idx - num_points)
                    end = close_idx
                    ranges[start:end] = close_dist
        return ranges

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

        # Disparity extender step (pre-bubble)
        sector = self.disparity_extender(sector)

        print("Sector is the: ", sector)
        bub = 200
       
        # Mask the closest obstacle with a bubble
        closest_rel_idx = np.argmin(sector)
        closest_abs_idx = i_min + closest_rel_idx

        bubble_start = max(i_min, closest_abs_idx - bub)
        bubble_end = min(i_max, closest_abs_idx + bub)

        sector_masked = sector.copy()
        mask_start = bubble_start - i_min
        mask_end = bubble_end - i_min
        sector_masked[mask_start:mask_end + 1] = 0.0

        print('sector range is: [', sector_masked, ']')

        # Find best direction
        best_rel_idx = np.argmax(sector_masked)
        best_abs_idx = i_min + best_rel_idx
        print('best index is: ', best_abs_idx)

        if sector[best_rel_idx] < 0.2:
            self.get_logger().info("CRASHED INTO WALL")
            return

        # === FIXED EDGE STICKING ===
        if best_abs_idx <= i_min + 5:
            self.get_logger().info("Best ray near LEFT EDGE → sticking left.")
        elif best_abs_idx >= i_max - 5:
            self.get_logger().info("Best ray near RIGHT EDGE → sticking right.")

        # 4) Pick the best ray in the remaining sector
        rel_best = np.argmax(sector_masked)
        best_i = rel_best + i_min
        steer = a_min + best_i * a_inc

        self.get_logger().info(f"Best ray: {best_i} | Steer: {steer:.2f} rad")

        # FORCE EDGE STICKING TURN if needed
        margin = 20
        if best_i <= i_min + margin:
            # steer = -1.0
            speed = self.min_speed * 0.6
            self.get_logger().info("FORCING LEFT STICK TURN")
        elif best_i >= i_max - margin:
            # steer = 1.0
            speed = self.min_speed * 0.6
            self.get_logger().info("FORCING RIGHT STICK TURN")
            if best_i <= i_min + margin and sector[best_rel_idx] < sector[best_rel_idx + (i_max - i_min)//2]:
                # steer = 0.0
                speed = self.min_speed * 0.6
        
        else:
            norm = min(abs(steer) / half, 1.0)
            speed = self.max_speed * (1 - norm) + self.min_speed * norm

        # 6) Publish
        self.publish_control(steer, speed)

    def publish_control(self, steer, speed):
        msg = AckermannDriveStamped()
        msg.drive.steering_angle = float(steer)
        msg.drive.speed = float(speed)
        self.pub.publish(msg)
        self.get_logger().info(f'STEER {steer:.2f} rad | SPD {speed:.2f} m/s')

def main(args=None):
    rclpy.init(args=args)
    node = SimpleGapFollower()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()