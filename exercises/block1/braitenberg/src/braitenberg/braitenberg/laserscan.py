
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class LidarLogger(Node):
    def __init__(self):
        super().__init__('laserscan')
        self.get_logger().info("LaserScan Node has been started")

        # Create a subscription to the lidar topic
        self.create_subscription(
            LaserScan,
            'lidar_right',
            self.lidar_callback,
            10)

    def lidar_callback(self, msg):
        self.get_logger().info(f"Lidar Data: {msg.ranges}")


def main():
    rclpy.init()
    node = LidarLogger()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
