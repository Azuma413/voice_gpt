#include <chrono>
#include <cstdio>
#include <string>
#include "chatrover_msgs/srv/text_text.hpp"
#include "rclcpp/rclcpp.hpp"
#include <memory>

using namespace std::chrono_literals;

class SimpleServer : public rclcpp::Node{
    public:
    SimpleServer() : Node("simple_server"){
        auto service_callback = [this](const std::shared_ptr<chatrover_msgs::srv::TextText::Request> request, std::shared_ptr<chatrover_msgs::srv::TextText::Response> response) -> void{
            std::cout << request->text << std::endl;
            response->text = request->text + " world!";
        };
        srv_ = create_service<chatrover_msgs::srv::TextText>("set_text", service_callback);
    }
    private:
    rclcpp::Service<chatrover_msgs::srv::TextText>::SharedPtr srv_;
};

int main(int argc, char **argv){
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SimpleServer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}