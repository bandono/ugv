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
#include "ugv/srv/command.hpp"
#include "ugv/msg/pwm.hpp"

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

    pwm_publisher_ = this->create_publisher<ugv::msg::PWM>("pwm", 10);

    RCLCPP_INFO(this->get_logger(), "Commander Ready!");
  }

private:
  void motor(const int ch1, const int ch2, const int throttle){
    auto message = ugv::msg::PWM();
    message.ch1 = ch1;
    message.ch2 = ch2;
    message.throttle = throttle;
    pwm_publisher_->publish(message);
  }
  void motor_l(const int throttle){
    motor(9, 8, throttle);
  }
  void motor_r(const int throttle){
    motor(11, 10, throttle);
  }
  void move_forward(){
    motor_l(1);
    motor_r(1);
  }
  void move_back(){
    motor_l(-1);
    motor_r(-1);
  }
  void turn_right(){
    motor_l(1);
    motor_r(-1);
  }
  void turn_left(){
    motor_l(-1);
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

    std::istringstream iss(request->command);
    std::string action;
    
    if (iss >> action)
    {
      if (action == "FORWARD")
      {
        int timer = -1;
        iss >> timer;

        move_forward();
        if (timer != -1){
          reset_timer_ = this->create_wall_timer(
            std::chrono::seconds(timer), std::bind(&Vehicle::reset_timer_callback, this));
        }

        std::ostringstream oss;
        oss << "move FORWARD for " << timer << "s" ;
        response->message = oss.str().c_str();
      }
      else if (action == "BACK"){
        int timer = -1;
        iss >> timer;

        move_back();
        if (timer != -1){
          reset_timer_ = this->create_wall_timer(
            std::chrono::seconds(timer), std::bind(&Vehicle::reset_timer_callback, this));
        }

        std::ostringstream oss;
        oss << "move BACK for " << timer << "s" ;
        response->message = oss.str().c_str();
      }
      else if (action == "RIGHT"){
        int timer = 1;
        iss >> timer;

        turn_right();
        if (timer != -1){
          reset_timer_ = this->create_wall_timer(
            std::chrono::seconds(timer), std::bind(&Vehicle::reset_timer_callback, this));
        }

        std::ostringstream oss;
        oss << "turn RIGHT for " << timer << "s" ;
        response->message = oss.str().c_str();
      }
      else if (action == "LEFT"){
        int timer = 1;
        iss >> timer;

        turn_left();
        if (timer != -1){
          reset_timer_ = this->create_wall_timer(
            std::chrono::seconds(timer), std::bind(&Vehicle::reset_timer_callback, this));
        }

        std::ostringstream oss;
        oss << "turn LEFT for " << timer << "s" ;
        response->message = oss.str().c_str();
      }
      else if (action == "STOP"){
        stop();

        std::ostringstream oss;
        oss << "Stopped";
        response->message = oss.str().c_str();
      }
      else if(action == "MOTOR"){
        int ch1 = 0;
        int ch2 = 0;
        float throttle = 0;
        iss >> ch1 >> ch2 >> throttle;

        motor(ch1, ch2, throttle);
        
        std::ostringstream oss;
        oss << "Set Channel " << ch1 << " And Channel " << ch2 << " To " << throttle;
        response->message = oss.str().c_str();
      }
      else
      {
        response->message = "Unknown command";
      }
    }
    else
    {
      response->message = "Invalid command format";
    }
  }

  void reset_timer_callback(){
    stop();
    reset_timer_->cancel();
  }
  
  rclcpp::Service<Command>::SharedPtr service_;
  rclcpp::TimerBase::SharedPtr reset_timer_;
  rclcpp::Publisher<ugv::msg::PWM>::SharedPtr pwm_publisher_;
  size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Vehicle>());
  rclcpp::shutdown();
  return 0;
}
