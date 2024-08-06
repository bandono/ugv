#include <rclcpp/rclcpp.hpp>
#include <signal.h>
#include <termios.h>
#include <stdio.h>
#include <memory.h>
#include <unistd.h>

#include "ugv/srv/command.hpp"

using Command = ugv::srv::Command;
using namespace std::chrono_literals;

#define KEYCODE_R 0x43 
#define KEYCODE_L 0x44
#define KEYCODE_U 0x41
#define KEYCODE_D 0x42
#define KEYCODE_Q 0x71

class Ground : public rclcpp::Node
{
public:
  Ground() : Node("ground")
  {
    client_ = this->create_client<Command>("send_command");
  }

  void send_request(const std::string &command)
  {
    while (!client_->wait_for_service(1s))
    {
      RCLCPP_INFO(this->get_logger(), "Service not available, waiting again...");
    }

    auto request = std::make_shared<Command::Request>();
    request->command = command;

    using ServiceResponseFuture = rclcpp::Client<Command>::SharedFuture;
    auto response_received_callback = [this](ServiceResponseFuture future)
    {
      auto response = future.get();
      //std::cout << "Response: " << response->message;
    };

    auto result = client_->async_send_request(request, response_received_callback);
    rclcpp::spin_until_future_complete(this->get_node_base_interface(), result) ;
  }

private:
  rclcpp::Client<Command>::SharedPtr client_;
};

int kfd = 0;
struct termios cooked, raw;

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Ground>();
  
  char c;
  bool dirty=false;


  // get the console in raw mode                                                              
  tcgetattr(kfd, &cooked);
  memcpy(&raw, &cooked, sizeof(struct termios));
  raw.c_lflag &=~ (ICANON | ECHO);
  // Setting a new line, then end of file                         
  raw.c_cc[VEOL] = 1;
  raw.c_cc[VEOF] = 2;
  tcsetattr(kfd, TCSANOW, &raw);

  std::string command;

  for(;;)
  {
    // get the next event from the keyboard  
    if(::read(kfd, &c, 1) < 0)
    {
      perror("read():");
      exit(-1);
    }
    
    switch(c)
    {
      case KEYCODE_L:
        std::cout << "LEFT" << std::endl;
        dirty = true;
        command = "LEFT -1";
        break;
      case KEYCODE_R:
        // ROS_DEBUG("RIGHT");
        std::cout << "RIGHT" << std::endl;
        dirty = true;
        command = "RIGHT -1";
        break;
      case KEYCODE_U:
        // ROS_DEBUG("UP");
        std::cout << "UP" << std::endl;
        dirty = true;
        command = "FORWARD -1";
        break;
      case KEYCODE_D:
        // ROS_DEBUG("DOWN");
        std::cout << "DOWN" << std::endl;
        dirty = true;
        command = "BACK -1";
        break;
      case KEYCODE_Q:
        std::cout << "STOP" << std::endl;
        dirty = true;
        command = "STOP";
        break;
    }
   
    if(dirty ==true)
    {
      node->send_request(command);   
      dirty=false;
    }
  }
  
  rclcpp::shutdown();
  return 0;
}
