#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <sstream>
#include <cstddef>
#include <sys/ioctl.h>
#include <cstdlib>
#include <cstdio>

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
  void motor(const int ch1, const int ch2, const int throttle){
    std::ostringstream oss;
    oss << "python3 scripts/motor.py --ch1=" << ch1 << " --ch2=" << ch2 << " --throttle=" << throttle;

    int err = std::system(oss.str().c_str());
  }
  void motor_l(const int throttle){
    motor(8, 9, throttle);
  }
  void motor_r(const int throttle){
    motor(10, 11, throttle);
  }
  void move_forward(){
    motor_l(1);
    motor_r(1);
  }
  void stop(){
    motor_l(0);
    motor_r(0);
  }
  void handle_command(const std::shared_ptr<Command::Request> request,
                      std::shared_ptr<Command::Response> response)
  {
    RCLCPP_INFO(this->get_logger(), "Received command: %s", request->command.c_str());

    if (request->command == "MOVE_FORWARD")
    {
      response->message = "Command executed: MOVE_FORWARD";
      RCLCPP_INFO(this->get_logger(), "Executing command: MOVE_FORWARD");
      move_forward();
    }
    else if (request->command == "STOP"){
      response->message = "Command executed: STOP";
      RCLCPP_INFO(this->get_logger(), "Executing command: STOP");
      stop();
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
