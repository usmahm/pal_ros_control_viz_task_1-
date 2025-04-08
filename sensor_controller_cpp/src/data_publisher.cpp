#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int64.hpp"

using namespace std::chrono_literals;

class DataPublisher : public rclcpp::Node
{
  public:
    DataPublisher()
    : Node("data_publisher")
    {
      publisher_ = this->create_publisher<std_msgs::msg::Int64>("sensor_data", 10);
      timer_ = this->create_wall_timer(2s, std::bind(&DataPublisher::timer_callback, this));
    }

  private:
    void timer_callback()
    {
      auto message = std_msgs::msg::Int64();
      message.data = count_++;
      
      RCLCPP_INFO(this->get_logger(), "Publishing: '%ld'", message.data);
      publisher_->publish(message);
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::Int64>::SharedPtr publisher_;
    size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  rclcpp::spin(std::make_shared<DataPublisher>());
  
  rclcpp::shutdown();
  return 0;
}