#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int64.hpp"
#include "example_interfaces/srv/set_bool.hpp"
#include <chrono>

#include "interfaces/msg/custom_message.hpp" 

using namespace std::chrono_literals;
using namespace std::placeholders;

class SensorControllerClient : public rclcpp::Node
{
  public:
    SensorControllerClient() : Node("sensor_controller_client")
    {
      subscriber_ = this->create_subscription<std_msgs::msg::Int64>(
        "sensor_data", 10, std::bind(&SensorControllerClient::sub_cb, this, _1));
      subscriber_2_ = this->create_subscription<interfaces::msg::CustomMessage>(
        "custom_data", 10, std::bind(&SensorControllerClient::sub_cb_2, this, _1));
      client_ = this->create_client<example_interfaces::srv::SetBool>("command_service");

      while (!client_->wait_for_service(1s)) {
        if (!rclcpp::ok()) {
          RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
          return;
        }
        RCLCPP_INFO(this->get_logger(), "service not available, waiting again...");
      }
    }
  
  private:
    void sub_cb(const std_msgs::msg::Int64::SharedPtr msg)
    {
      this->send_request(msg->data % 2 == 0);
    }
    
    void sub_cb_2(const interfaces::msg::CustomMessage::SharedPtr msg)
    {
      RCLCPP_INFO(this->get_logger(), "Success: %d", msg->is_hardware);
    }

    void send_request(bool toggle)
    {
      auto request = std::make_shared<example_interfaces::srv::SetBool::Request>();
      request->data = toggle;

      auto result = client_->async_send_request(
        request,
        std::bind(&SensorControllerClient::handle_response, this, _1));
    }

    void handle_response(rclcpp::Client<example_interfaces::srv::SetBool>::SharedFuture future)
    {
      auto response = future.get();
      RCLCPP_INFO(this->get_logger(), "Success: %d", response->success);
    }

    rclcpp::Subscription<std_msgs::msg::Int64>::SharedPtr subscriber_;
    rclcpp::Subscription<interfaces::msg::CustomMessage>::SharedPtr subscriber_2_;
    rclcpp::Client<example_interfaces::srv::SetBool>::SharedPtr client_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  rclcpp::spin(std::make_shared<SensorControllerClient>());

  rclcpp::shutdown();
  return 0;
}