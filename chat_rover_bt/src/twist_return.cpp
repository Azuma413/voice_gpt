#include <chrono>
#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

using namespace std::chrono_literals;

class MinimalPubSub : public rclcpp::Node{
    public:
    MinimalPubSub() : Node("minimal_pubsub"){
        auto topic_callback = [this](const geometry_msgs::msg::Twist& msg) -> void{
            auto message = geometry_msgs::msg::Twist();
            message = msg;
            std::cout << "X:" << message.linear.x << "/nY:" << message.linear.y << std::endl;
            publisher_->publish(message);
        };

        rclcpp::QoS qos(rclcpp::KeepLast(10));
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", qos);
        subscription_ = this->create_subscription<geometry_msgs::msg::Twist>("joy_data", qos, topic_callback);
    }
    private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;
};

int main(int argc, char* argv[]){
    rclcpp::init(argc,argv);
    rclcpp::spin(std::make_shared<MinimalPubSub>());
    rclcpp::shutdown();
    return 0;
}