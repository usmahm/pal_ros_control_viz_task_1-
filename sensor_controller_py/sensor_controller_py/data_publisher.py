import rclpy
from rclpy.node import Node

from std_msgs.msg import Int64


class DataPublisher(Node):

    def __init__(self):
        super().__init__('data_publisher')
        self.publisher_ = self.create_publisher(Int64, 'sensor_data', 10)

        self.timer = self.create_timer(2, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = Int64()
        msg.data = self.i
        self.publisher_.publish(msg)
        
        self.get_logger().info(f"Publishing: '{msg.data}'")
        self.i += 1


def main(args=None):
    rclpy.init(args=args)

    data_publisher = DataPublisher()

    rclpy.spin(data_publisher)

    data_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()