#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/srv/set_bool.hpp"

using namespace std::placeholders;

class SensorControllerServer : public rclcpp::Node
{
  public:
    SensorControllerServer() : Node("sensor_controller_server")
    {
      service_ = this->create_service<example_interfaces::srv::SetBool>(
        "command_service", std::bind(&SensorControllerServer::server_cb, this, _1, _2));

    }

  private:
    void server_cb(const std::shared_ptr<example_interfaces::srv::SetBool::Request> request, std::shared_ptr<example_interfaces::srv::SetBool::Response> response)
    {
      RCLCPP_INFO(this->get_logger(), "Actuating signal is %d", request->data);

      response->success = true;
    }

    rclcpp::Service<example_interfaces::srv::SetBool>::SharedPtr service_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  rclcpp::spin(std::make_shared<SensorControllerServer>());

  rclcpp::shutdown();
  return 0;
}