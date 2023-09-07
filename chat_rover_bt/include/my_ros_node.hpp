#pragma once
#include <chrono> //計測用，xxmsという書き方ができるようにする
#include <functional> //ラムダ式などを使えるようにする
#include <memory> //メモリを扱えるようにする
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/string.hpp"
#include "chatrover_msgs/msg/object_position.hpp"
#include "raspimouse_msgs/msg/light_sensors.hpp"
#include "std_srvs/srv/set_bool.hpp"

using namespace std::chrono_literals;

class BtRosNode : public rclcpp::Node{ //rclcpp::Nodeを継承してMinimalPubSubクラスを作成
    public:

    BtRosNode() : Node("bt_ros_node"){ //Node関数をオーバーライド
        std::cout << "bt_ros_node is called" << std::endl;
        auto gpt_command_cb = [this](const std_msgs::msg::String& msg) -> void{gpt_command_data = msg.data;};
        auto object_pos_cb = [this](const chatrover_msgs::msg::ObjectPosition& msg) -> void {
            detect_object_list = msg.name;
            detect_object_pos = msg.position;
            };
        auto light_sensors_cb = [this](const raspimouse_msgs::msg::LightSensors& msg) -> void{light_sensors_data = msg.data;};

        rclcpp::QoS qos(rclcpp::KeepLast(10));

        cmd_vel_pub = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", qos);

        gpt_command_sub = this->create_subscription<std_msgs::msg::String>("/gpt_command", qos, gpt_command_cb);
        object_pos_sub = this->create_subscription<chatrover_msgs::msg::ObjectPosition>("/object_pos", qos, object_pos_cb);
        light_sensors_sub = this->create_subscription<raspimouse_msgs::msg::LightSensors>("light_sensors", qos, light_sensors_cb);

        motor_power_cli = create_client<std_srvs::srv::SetBool>("motor_power");
        while(!motor_power_cli->wait_for_service(1s)){
            if(!rclcpp::ok()){
                return;
            }
            std::cout << "motor_power service not available" << std::endl;
        }
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
    std::string gpt_command_data = "";
    chatrover_msgs::msg::ObjectPosition object_pos_data;
    raspimouse_msgs::msg::LightSensors light_sensors_data;
    
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr gpt_command_sub;
    rclcpp::Subscription<chatrover_msgs::msg::ObjectPosition>::SharedPtr object_pos_sub;
    rclcpp::Subscription<raspimouse_msgs::msg::LightSensors>::SharedPtr light_sensors_sub;
    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr motor_power_cli;
};

std::shared_ptr<BtRosNode> global_node;