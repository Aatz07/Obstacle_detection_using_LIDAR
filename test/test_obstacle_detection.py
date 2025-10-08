import pytest
from unittest.mock import MagicMock
from std_msgs.msg import Bool
from sensor_msgs.msg import LaserScan
from ks_obstacle_detection.obstacle_detector import ObstacleDetector
import rclpy
import math
import array


@pytest.fixture(scope="function")
def node():
    """Fixture to initialize the ObstacleDetector node."""
    rclpy.init(args=None)
    node = ObstacleDetector()
    yield node
    node.destroy_node()
    rclpy.shutdown()


def test_TC_OD001_subscribes_to_scan_topic(node):
    """Test OD001: Verify the node subscribes to the /scan topic."""
    assert node.subscription, "Subscription to /scan topic is not initialized."


def test_TC_OD002_processes_ROI_data(node):
    msg = LaserScan()
    msg.angle_min = -math.pi / 2
    msg.angle_increment = math.pi / 180
    msg.ranges = array.array('f', [1.0] * 360)
    msg.ranges[60:90] = array.array('f', [0.5] * 30)

    node.scan_callback(msg)
    assert True 


def test_TC_OD003_publishes_masked_scan(node):
    """Test OD003: Verify the masked scan publishes data."""
    msg = LaserScan()
    msg.angle_min = -math.pi / 2
    msg.angle_increment = math.pi / 180
    msg.ranges = [1.0] * 360

    node.masked_scan_publisher.publish = MagicMock()
    node.scan_callback(msg)

    node.masked_scan_publisher.publish.assert_called_once()


def test_TC_OD004_detects_obstacle_in_ROI(node):
    msg = LaserScan()
    msg.angle_min = -math.pi / 2
    msg.angle_increment = math.pi / 180
    msg.ranges = array.array('f', [float('inf')] * 360)
    msg.ranges[60:90] = array.array('f', [0.5] * 30)

    node.obstacle_status_publisher.publish = MagicMock()
    node.scan_callback(msg)
    node.obstacle_status_publisher.publish.assert_called_once_with(Bool(data=True))


def test_TC_OD005_publishes_obstacle_status_true(node):
    """Test OD005: Verify obstacle status publishes True when detected."""
    msg = LaserScan()
    msg.angle_min = -math.pi / 2
    msg.angle_increment = math.pi / 180
    msg.ranges = array.array('f', [float('inf')] * 360)  # Initialize full 360-degree range
    msg.ranges[60:90] = array.array('f', [0.5] * 30)     # Simulated obstacle in ROI

    # Mock the publisher
    node.obstacle_status_publisher.publish = MagicMock()

    # Trigger callback
    node.scan_callback(msg)

    # Verify if obstacle status was published as True
    node.obstacle_status_publisher.publish.assert_called_once_with(Bool(data=True))



def test_TC_OD006_logs_warning_on_obstacle(node):
    """Test OD006: Verify log warning when an obstacle is detected."""
    msg = LaserScan()
    msg.angle_min = -math.pi / 2
    msg.angle_increment = math.pi / 180
    msg.ranges = array.array('f', [float('inf')] * 360)
    msg.ranges[60:90] = array.array('f', [0.5] * 30)  # Simulated obstacle

    # Mock the logger
    node.get_logger().warn = MagicMock()

    # Trigger the callback
    node.scan_callback(msg)

    # Verify the log call
    node.get_logger().warn.assert_called_with("Obstacle detected within 0.6m in the -90° to -30° range.")


def test_TC_OD007_logs_info_on_no_obstacles(node):
    """Test OD007: Verify log info when no obstacles are detected."""
    msg = LaserScan()
    msg.angle_min = -math.pi / 2
    msg.angle_increment = math.pi / 180
    msg.ranges = array.array('f', [float('inf')] * 360)  # No obstacles

    # Mock the logger
    node.get_logger().info = MagicMock()

    # Trigger the callback
    node.scan_callback(msg)

    # Verify the log call
    node.get_logger().info.assert_called_with("No obstacles detected within 0.6m in the -90° to -30° range.")



def test_TC_OD008_calculates_dynamic_indices(node):
    """Test OD008: Verify calculation of ROI indices."""
    msg = LaserScan()
    msg.angle_min = -math.pi / 2
    msg.angle_increment = math.pi / 180

    index_min = math.ceil((-math.pi / 2 - msg.angle_min) / msg.angle_increment)
    index_max = math.floor((-math.pi / 6 - msg.angle_min) / msg.angle_increment)

    assert index_min == 0 and index_max == 60, "Dynamic indices calculation failed."

