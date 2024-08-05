#include <rclcpp/rclcpp.hpp>
#include "ugv/srv/command.hpp"

using Command = ugv::srv::Command;
using namespace std::chrono_literals;

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
    };

    auto future_result = client_->async_send_request(request, response_received_callback);
  }

private:
  rclcpp::Client<Command>::SharedPtr client_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Ground>();
  node->send_request("MOVE_FORWARD");
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
