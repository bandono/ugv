#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "ugv/srv/command.hpp"

using namespace std::chrono_literals;

using Command = ugv::srv::Command;

class Vehicle : public rclcpp::Node
{
public:
  Vehicle()
  : Node("vehicle"), count_(0)
  {
    service_ = this->create_service<Command>("send_command", 
      std::bind(&Vehicle::handle_command, this, std::placeholders::_1, std::placeholders::_2));

    publisher_ = this->create_publisher<std_msgs::msg::String>("respati/ugv/commander", 10);
    timer_ = this->create_wall_timer(
      500ms, std::bind(&Vehicle::timer_callback, this));
  }

private:

  void handle_command(const std::shared_ptr<Command::Request> request,
                      std::shared_ptr<Command::Response> response)
  {
    RCLCPP_INFO(this->get_logger(), "Received command: %s", request->command.c_str());

    if (request->command == "MOVE_FORWARD")
    {
      response->message = "Command executed: MOVE_FORWARD";
      RCLCPP_INFO(this->get_logger(), "Executing command: MOVE_FORWARD");
      // Add your movement logic here
    }
    else
    {
      response->message = "Unknown command";
    }
  }

  
  void timer_callback()
  {
    auto message = std_msgs::msg::String();
    message.data = "Hello, world! " + std::to_string(count_++);
    publisher_->publish(message);
  }
  
  rclcpp::Service<Command>::SharedPtr service_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Vehicle>());
  rclcpp::shutdown();
  return 0;
}
