from std_msgs.msg import Int64
from example_interfaces.srv import SetBool

import rclpy
from rclpy.node import Node

class SensorControllerClient(Node):
    def __init__(self):
        super().__init__("sensor_controller_client")

        self.subscriber_ = self.create_subscription(Int64, 'sensor_data', self.sub_cb, 10)

        self.client_ = self.create_client(SetBool, "command_service")

        while not self.client_.wait_for_service(1.0):
            self.get_logger().info('service not available, waiting again...')

        self.request = SetBool.Request()

    def sub_cb(self, msg):
        self.send_request(msg.data % 2 == 0)

    def send_request(self, toggle: bool):
        self.request.data = toggle

        future = self.client_.call_async(self.request)
        future.add_done_callback(self.handle_response)

    def handle_response(self, future):
        response = future.result()
        self.get_logger().info(f"Success: {response.success}")

def main(args=None):
    rclpy.init(args=args)

    sensor_controller = SensorControllerClient()

    rclpy.spin(sensor_controller)

    sensor_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()