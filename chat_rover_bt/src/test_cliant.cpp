#include "rclcpp/rclcpp.hpp"
#include "chatrover_msgs/srv/text_text.hpp"
#include <chrono>
#include <cstdlib>
#include <memory>

using namespace std::chrono_literals;

class SimpleClient : public rclcpp::Node{
  public:
  SimpleClient() : Node("simple_client"){
      client_ = create_client<chatrover_msgs::srv::TextText>("set_text");
      while (!client_->wait_for_service(1s)) {
          if (!rclcpp::ok()) {
              break;
          }
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "set_text service not available");
      }
  }

  std::string sync_set_text(std::string data){
     //同期
     auto request = std::make_shared<chatrover_msgs::srv::TextText::Request>();
     request->text = data;
     auto future_result = client_->async_send_request(request);
     if (rclcpp::spin_until_future_complete(this->shared_from_this(), future_result) == rclcpp::FutureReturnCode::SUCCESS) {
        return future_result.get()->text;
     }
     std::cout << "can't get future_result" << std::endl;
     return NULL;
  }

  private:
  rclcpp::Client<chatrover_msgs::srv::TextText>::SharedPtr client_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SimpleClient>();
  std::string result = node->sync_set_text("hello");
  std::cout << result << std::endl;
  rclcpp::shutdown();
  return 0;
}