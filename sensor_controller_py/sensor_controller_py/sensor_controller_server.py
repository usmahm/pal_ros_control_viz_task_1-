from std_msgs.msg import Int64
from example_interfaces.srv import SetBool

import rclpy
from rclpy.node import Node

class SensorControllerServer(Node):
    def __init__(self):
        super().__init__("sensor_controller_server")

        self.service_ = self.create_service(SetBool, "command_service", self.server_cb)

    def server_cb(self, request, response):
        self.get_logger().info(f"Actuating signal is {request.data}")

        response.success = True
        return response

def main(args=None):
    rclpy.init(args=args)

    sensor_controller = SensorControllerServer()

    rclpy.spin(sensor_controller)

    sensor_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()