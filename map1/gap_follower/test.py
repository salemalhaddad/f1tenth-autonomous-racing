import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
import numpy as np

class ReactiveGapFollower(Node):
    def __init__(self):
        super().__init__('reactive_gap_follower')
        self.publisher_ = self.create_publisher(AckermannDriveStamped, '/drive', 10)
        self.subscription = self.create_subscription(LaserScan, '/scan', self.lidar_callback, 10)
        self.subscription  # prevent unused variable warning
        self.angle = 0.0
        self.ranges = []

    def lidar_callback(self, lidar_info):
        # Step 1: preprocess LIDAR data
        self.ranges = np.array(lidar_info.ranges)
        # Define the angles of interest (e.g., -70 to +70 degrees)
        min_angle = -70 * np.pi/180
        max_angle = 70 * np.pi/180
        min_indx = int((min_angle - lidar_info.angle_min) / lidar_info.angle_increment)
        max_indx = int((max_angle - lidar_info.angle_min) / lidar_info.angle_increment)

        # Clip ranges to [0, range_max]
        self.ranges = np.clip(self.ranges, 0, lidar_info.range_max)

        # Replace inf/nan with max range
        self.ranges[np.isinf(self.ranges)] = lidar_info.range_max
        self.ranges[np.isnan(self.ranges)] = lidar_info.range_max

        # Step 2: find closest obstacle
        closest_distance = float('inf')
        closest_indx = -1
        for i in range(min_indx, max_indx + 1):
            # Sum over a window of 5 elements
            window_indices = np.arange(max(i-2, 0), min(i+3, len(self.ranges)))
            distance = np.sum(self.ranges[window_indices])
            if distance < closest_distance:
                closest_distance = distance
                closest_indx = i

        # Step 3: create bubble around closest obstacle
        radius = 150
        start_idx = max(0, closest_indx - radius)
        end_idx = min(len(self.ranges) - 1, closest_indx + radius)
        self.ranges[start_idx:end_idx+1] = 0.0

        # Step 4: find largest gap
        start, end = 0, 0
        current_start = -1
        longest_duration = 0
        duration = 0

        for i in range(min_indx, max_indx + 1):
            if self.ranges[i] > 0:
                if current_start == -1:
                    current_start = i
                duration += 1
            else:
                if current_start != -1:
                    if duration > longest_duration:
                        longest_duration = duration
                        start, end = current_start, i - 1
                    current_start = -1
                    duration = 0
        # Check last gap
        if current_start != -1:
            if duration > longest_duration:
                longest_duration = duration
                start, end = current_start, max_indx

        # Step 5: find the best point in the largest gap
        max_distance = -1
        best_idx = start
        for i in range(start, end + 1):
            if self.ranges[i] > max_distance:
                max_distance = self.ranges[i]
                best_idx = i
            elif self.ranges[i] == max_distance:
                # Tie-breaker: choose the angle with the lowest absolute value
                current_angle = lidar_info.angle_min + i * lidar_info.angle_increment
                best_angle = lidar_info.angle_min + best_idx * lidar_info.angle_increment
                if abs(current_angle) < abs(best_angle):
                    best_idx = i

        # Compute steering angle
        self.angle = lidar_info.angle_min + best_idx * lidar_info.angle_increment

        # Actuate control
        self.reactive_control()

    def reactive_control(self):
        ackermann_drive = AckermannDriveStamped()
        ackermann_drive.drive.steering_angle = self.angle

        # Speed control based on steering angle
        if abs(self.angle) < 0.2:
            ackermann_drive.drive.speed = 2.0  # fast
        elif abs(self.angle) < 0.5:
            ackermann_drive.drive.speed = 1.0  # moderate
        else:
            ackermann_drive.drive.speed = 0.5  # slow

        self.publisher_.publish(ackermann_drive)
        self.get_logger().info(f'Steering Angle: {self.angle:.2f} rad, Speed: {ackermann_drive.drive.speed:.2f} m/s')

def main(args=None):
    rclpy.init(args=args)
    node = ReactiveGapFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

