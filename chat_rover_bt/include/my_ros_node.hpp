#pragma once
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "chatrover_msgs/srv/text_text.hpp"
#include "chatrover_msgs/srv/send_pos.hpp"
#include "std_srvs/srv/set_bool.hpp"

using namespace std::chrono_literals;

struct Point{
    int x;
    int y;
};

class BtRosNode : public rclcpp::Node{
    public:
    BtRosNode() : Node("main_node"){
        std::cout << "main_node is called" << std::endl;
        auto robot_state_cb = [this](const geometry_msgs::msg::Twist& msg) -> void{robot_state_data = msg;};

        rclcpp::QoS qos(rclcpp::KeepLast(10));

        robot_state_sub = this->create_subscription<geometry_msgs::msg::Twist>("/robot_state", qos, robot_state_cb);

        bool flag = true;
        
        get_voice_cli = create_client<chatrover_msgs::srv::TextText>("/get_voice");
        while(!get_voice_cli->wait_for_service(1s)){
            if(!rclcpp::ok()){
                std::cout << "get_voice service not available" << std::endl;
                flag = true;
                break;
            }
            if(flag){
                std::cout << "get_voice service not available" << std::endl;
                flag = false;
            }
        }
        std::cout << "get_voice service available" << std::endl;

        send_pos_cli = create_client<chatrover_msgs::srv::SendPos>("/send_pos");
        while(!send_pos_cli->wait_for_service(1s)){
            if(!rclcpp::ok()){
                std::cout << "send_pos service not available" << std::endl;
                flag = true;
                break;
            }
            if(flag){
                std::cout << "send_pos service not available" << std::endl;
                flag = false;
            }
        }
        std::cout << "send_pos service available" << std::endl;

        get_object_cli = create_client<chatrover_msgs::srv::TextText>("/get_object");
        while(!get_object_cli->wait_for_service(1s)){
            if(!rclcpp::ok()){
                std::cout << "get_object service not available" << std::endl;
                flag = true;
                break;
            }
            if(flag){
                std::cout << "get_object service not available" << std::endl;
                flag = false;
            }
        }
        std::cout << "get_object service available" << std::endl;

        gpt1_service_cli = create_client<chatrover_msgs::srv::TextText>("/gpt1_service");
        while(!gpt1_service_cli->wait_for_service(1s)){
            if(!rclcpp::ok()){
                std::cout << "gpt1_service service not available" << std::endl;
                flag = true;
                break;
            }
            if(flag){
                std::cout << "gpt1_service service not available" << std::endl;
                flag = false;
            }
        }
        std::cout << "gpt1_service service available" << std::endl;

        gpt2_service_cli = create_client<chatrover_msgs::srv::TextText>("/gpt2_service");
        while(!gpt2_service_cli->wait_for_service(1s)){
            if(!rclcpp::ok()){
                std::cout << "gpt2_service service not available" << std::endl;
                flag = true;
                break;
            }
            if(flag){
                std::cout << "gpt2_service service not available" << std::endl;
                flag = false;
            }
        }
        std::cout << "gpt2_service service available" << std::endl;
    }

    void sub_robot_state(geometry_msgs::msg::Twist& data){
        data = robot_state_data;
    }

    bool send_pos(Point point){
        auto request = std::make_shared<chatrover_msgs::srv::SendPos::Request>();
        request->x = point.x;
        request->y = point.y;
        auto future_result = send_pos_cli->async_send_request(request);
        if (rclcpp::spin_until_future_complete(this->shared_from_this(), future_result) == rclcpp::FutureReturnCode::SUCCESS){
            return future_result.get()->success;
        }
        std::cout << "can't get future_result" << std::endl;
        return false;     
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

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr robot_state_sub;
    rclcpp::Client<chatrover_msgs::srv::SendPos>::SharedPtr send_pos_cli;
    rclcpp::Client<chatrover_msgs::srv::TextText>::SharedPtr get_object_cli;
    rclcpp::Client<chatrover_msgs::srv::TextText>::SharedPtr get_voice_cli;
    rclcpp::Client<chatrover_msgs::srv::TextText>::SharedPtr gpt1_service_cli;
    rclcpp::Client<chatrover_msgs::srv::TextText>::SharedPtr gpt2_service_cli;
};

std::shared_ptr<BtRosNode> global_node;