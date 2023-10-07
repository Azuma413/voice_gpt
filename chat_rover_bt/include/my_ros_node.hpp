#pragma once
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
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
    BtRosNode() : Node("bt_ros_node"){
        std::cout << "bt_ros_node is called" << std::endl;
        auto robot_state_cb = [this](const geometry_msgs::msg::Twist& msg) -> void{robot_state_data = msg.data;};

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
        get_object_cli = create_client<chatrover_msgs::srv::ObjectPosition>("get_object");
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
        // motor on
    }
    
    void pub_cmd_vel(double vel, double rad_vel){
        auto pub_msg = geometry_msgs::msg::Twist();
        pub_msg.linear.x = vel;
        pub_msg.angular.z = rad_vel;
        this->cmd_vel_pub->publish(pub_msg);
    }

    void sub_gpt_command(std::string& msg){
        msg = this->gpt_command_data;
    }
    void sub_object_pos(std::vector<std::string>& list, std::vector<float>& pos){
        list = this->detect_object_list;
        pos = this->detect_object_pos;
    }
    void sub_light_sensor(bool& msg){
        msg = this->light_sensor_data;
    }

    bool send_motor_power(bool msg){
        auto req = std::make_shared<std_srvs::srv::SetBool::Request>();
        bool result = false;
        req->data = msg;
        auto res_received_cb = [&result](rclcpp::Client<std_srvs::srv::SetBool>::SharedFuture future){
            auto res = future.get();
            result = res->success;
        };
        auto future_result = linear_cli->async_send_request(req, res_received_cb);
        return result;
    }

    private:
    geometry_msgs::msg::Twist robot_state_data;
    
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr gpt_command_sub;
    rclcpp::Subscription<chatrover_msgs::msg::ObjectPosition>::SharedPtr object_pos_sub;
    rclcpp::Subscription<raspimouse_msgs::msg::LightSensors>::SharedPtr light_sensors_sub;
    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr motor_power_cli;
};

std::shared_ptr<BtRosNode> global_node;