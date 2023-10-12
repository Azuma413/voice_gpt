#pragma once
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "chatrover_msgs/srv/object_position.hpp"
#include "chatrover_msgs/srv/text_text.hpp"
#include "std_srvs/srv/set_bool.hpp"

/*
vosk_node
service /get_voice text.srv

yolo_node
service /get_object object_position.srv

gpt1_node
service /gpt1_service text.srv

gpt2_node
service /gpt2_service text.srv

qr_node
topic /robot_state twist

main_node
service motor_power
topic cmd_vel
*/

using namespace std::chrono_literals;

class BtRosNode : public rclcpp::Node{
    public:
    BtRosNode() : Node("main_node"){
        std::cout << "main_node is called" << std::endl;
        auto robot_state_cb = [this](const geometry_msgs::msg::Twist& msg) -> void{robot_state_data = msg;};

        rclcpp::QoS qos(rclcpp::KeepLast(10));

        cmd_vel_pub = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", qos);
        robot_state_sub = this->create_subscription<geometry_msgs::msg::Twist>("/robot_state", qos, robot_state_cb);
        motor_power_cli = create_client<std_srvs::srv::SetBool>("motor_power");
        while(!motor_power_cli->wait_for_service(1s)){
            if(!rclcpp::ok()){
                return;
            }
            std::cout << "motor_power service not available" << std::endl;
        }
        get_voice_cli = create_client<chatrover_msgs::srv::TextText>("/get_voice");
        while(!get_voice_cli->wait_for_service(1s)){
            if(!rclcpp::ok()){
                return;
            }
            std::cout << "get_voice service not available" << std::endl;
        }
        get_object_cli = create_client<chatrover_msgs::srv::TextText>("/get_object");
        while(!get_object_cli->wait_for_service(1s)){
            if(!rclcpp::ok()){
                return;
            }
            std::cout << "get_object service not available" << std::endl;
        }
        gpt1_service_cli = create_client<chatrover_msgs::srv::TextText>("/gpt1_service");
        while(!gpt1_service_cli->wait_for_service(1s)){
            if(!rclcpp::ok()){
                return;
            }
            std::cout << "gpt1_service service not available" << std::endl;
        }
        gpt2_service_cli = create_client<chatrover_msgs::srv::TextText>("/gpt2_service");
        while(!gpt2_service_cli->wait_for_service(1s)){
            if(!rclcpp::ok()){
                return;
            }
            std::cout << "gpt2_service service not available" << std::endl;
        }

        auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
        request->data = true;
        auto future_result = motor_power_cli->async_send_request(request);
        if (rclcpp::spin_until_future_complete(this->shared_from_this(), future_result) == rclcpp::FutureReturnCode::SUCCESS) {
            bool result = future_result.get()->success;
            if(result){
                std::cout << "motor on!" << std::endl;
            }else{
                std::cout << "Motor fails to start!" << std::endl;
            }
        }
    }
    
    void pub_cmd_vel(double vel, double rad_vel){
        auto pub_msg = geometry_msgs::msg::Twist();
        pub_msg.linear.x = vel;
        pub_msg.angular.z = rad_vel;
        this->cmd_vel_pub->publish(pub_msg);
    }
    void sub_robot_state(geometry_msgs::msg::Twist& data){
        data = robot_state_data;
    }
    void send_get_object(std::string& string_data){
        auto request = std::make_shared<chatrover_msgs::srv::TextText::Request>();
        request->text = "";
        auto future_result = get_object_cli->async_send_request(request);
        if (rclcpp::spin_until_future_complete(this->shared_from_this(), future_result) == rclcpp::FutureReturnCode::SUCCESS) {
            string_data = future_result.get()->text;
            return;
        }
        std::cout << "can't get future_result" << std::endl;
        return;
    }
    void send_get_voice(std::string& string_data){
        auto request = std::make_shared<chatrover_msgs::srv::TextText::Request>();
        request->text = "";
        auto future_result = get_voice_cli->async_send_request(request);
        if (rclcpp::spin_until_future_complete(this->shared_from_this(), future_result) == rclcpp::FutureReturnCode::SUCCESS) {
            string_data = future_result.get()->text;
            return;
        }
        std::cout << "can't get future_result" << std::endl;
        return;
    }
    void send_gpt1_service(std::string send_data, std::string& get_data){
        auto request = std::make_shared<chatrover_msgs::srv::TextText::Request>();
        request->text = send_data;
        auto future_result = gpt1_service_cli->async_send_request(request);
        if (rclcpp::spin_until_future_complete(this->shared_from_this(), future_result) == rclcpp::FutureReturnCode::SUCCESS) {
            get_data = future_result.get()->text;
            return;
        }
        std::cout << "can't get future_result" << std::endl;
        return;
    }
    void send_gpt2_service(std::string send_data, std::string& get_data){
        auto request = std::make_shared<chatrover_msgs::srv::TextText::Request>();
        request->text = send_data;
        auto future_result = gpt2_service_cli->async_send_request(request);
        if (rclcpp::spin_until_future_complete(this->shared_from_this(), future_result) == rclcpp::FutureReturnCode::SUCCESS) {
            get_data = future_result.get()->text;
            return;
        }
        std::cout << "can't get future_result" << std::endl;
        return;
    }

    private:
    geometry_msgs::msg::Twist robot_state_data;
    
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr robot_state_sub;
    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr motor_power_cli;
    rclcpp::Client<chatrover_msgs::srv::TextText>::SharedPtr get_object_cli;
    rclcpp::Client<chatrover_msgs::srv::TextText>::SharedPtr get_voice_cli;
    rclcpp::Client<chatrover_msgs::srv::TextText>::SharedPtr gpt1_service_cli;
    rclcpp::Client<chatrover_msgs::srv::TextText>::SharedPtr gpt2_service_cli;
};

std::shared_ptr<BtRosNode> global_node;