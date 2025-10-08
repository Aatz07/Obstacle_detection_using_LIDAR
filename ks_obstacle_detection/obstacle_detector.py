import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
import math
import numpy as np  # For handling NaN values


class ObstacleDetector(Node):
    def __init__(self, log_capture=None):
        super().__init__('obstacle_detector')

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )

        # Subscription to LaserScan
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            qos_profile
        )

        # Publishers
        self.masked_scan_publisher = self.create_publisher(LaserScan, '/masked_scan', qos_profile)

        self.obstacle_status_publisher = self.create_publisher(Bool, '/obstacle_status', qos_profile)

        # Log capture for testing
        self.logger = log_capture or self.get_logger()


        self.logger.info("Obstacle Detector Node has started.")

    def scan_callback(self, msg):
        ranges = msg.ranges

        try:
            # Calculate indices for -90° to -30° relative to the forward direction
            angle_min = msg.angle_min
            angle_increment = msg.angle_increment

            index_min = math.ceil((-math.pi / 2 - angle_min) / angle_increment)
            index_max = math.floor((-math.pi / 6 - angle_min) / angle_increment)

            self.logger.info(f"Checking ranges between indices {index_min} and {index_max}")

            # Masked ranges for ROI visualization
            masked_ranges = [float('inf')] * len(ranges)

            # Obstacle detection flag
            obstacle_detected = False

            # Check ranges within the specified angle range
            for i in range(index_min, index_max + 1):
                distance = ranges[i]
                if 0 < distance < 0.65:  # Check within detection range
                    masked_ranges[i] = distance
                    obstacle_detected = True

            # Publish masked LaserScan
            masked_scan = LaserScan()
            masked_scan.header = msg.header
            masked_scan.angle_min = msg.angle_min
            masked_scan.angle_max = msg.angle_max
            masked_scan.angle_increment = msg.angle_increment
            masked_scan.time_increment = msg.time_increment
            masked_scan.scan_time = msg.scan_time
            masked_scan.range_min = msg.range_min
            masked_scan.range_max = msg.range_max
            masked_scan.ranges = masked_ranges
            self.masked_scan_publisher.publish(masked_scan)

            # Publish obstacle status
            obstacle_status = Bool()
            obstacle_status.data = obstacle_detected
            self.obstacle_status_publisher.publish(obstacle_status)

            # Log detection
            if obstacle_detected:
                self.logger.warn("Obstacle detected within 0.6m in the -90° to -30° range.")
            else:
                self.logger.info("No obstacles detected within 0.6m in the -90° to -30° range.")
        except Exception as e:
            self.logger.error(f"Error processing LaserScan message: {e}")



def main(args=None):
    rclpy.init(args=args)
    node = ObstacleDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
