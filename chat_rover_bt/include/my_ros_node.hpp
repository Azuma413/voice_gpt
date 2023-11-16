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
#include <inttypes.h>
#include "chatrover_msgs/action/send_pos.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

using namespace std::chrono_literals;
using SendPosAction = chatrover_msgs::action::SendPos;
using GoalHandleSendPos = rclcpp_action::ClientGoalHandle<SendPosAction>;

struct Point{
    int x;
    int y;
};

class BtRosNode : public rclcpp::Node{
    public:
    BtRosNode() : Node("main_node"), goal_done(false){
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

    //------------------------ action setting -------------------------------
        this->client_ptr = rclcpp_action::create_client<SendPosAction>(this->get_node_base_interface(),this->get_node_graph_interface(),this->get_node_logging_interface(),this->get_node_waitables_interface(),"send_pos");
    }

    //ゲッター関数
    bool is_goal_done() const{
        return this->goal_done;
    }

    bool send_pos(Point point){
        using namespace std::placeholders;
        // flagをfalseに設定
        this->goal_done = false;
        //clientが設定されていない場合の例外処理
        if (!this->client_ptr) {
            RCLCPP_ERROR(this->get_logger(), "Action client not initialized");
        }
        //サーバーの起動を待つ
        if (!this->client_ptr->wait_for_action_server(std::chrono::seconds(10))) {
            RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
            this->goal_done = true;
            return false;
        }
        // 目標値を設定
        auto goal_msg = SendPosAction::Goal();
        goal_msg.x = point.x;
        goal_msg.y = point.y;
        RCLCPP_INFO(this->get_logger(), "Sending goal");
        // 目標値のアクションサーバー送信とコールバックメソッドの登録
        auto send_goal_options = rclcpp_action::Client<SendPosAction>::SendGoalOptions();
        send_goal_options.goal_response_callback = std::bind(&BtRosNode::goal_response_callback, this, _1);
        send_goal_options.feedback_callback = std::bind(&BtRosNode::feedback_callback, this, _1, _2);
        send_goal_options.result_callback = std::bind(&BtRosNode::result_callback, this, _1);
        auto goal_handle_future = this->client_ptr->async_send_goal(goal_msg, send_goal_options);
        return false;     
    }

    // -----------------------------------------------------------------------------------------------

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

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr robot_state_sub;
    rclcpp::Client<chatrover_msgs::srv::TextText>::SharedPtr get_object_cli;
    rclcpp::Client<chatrover_msgs::srv::TextText>::SharedPtr get_voice_cli;
    rclcpp::Client<chatrover_msgs::srv::TextText>::SharedPtr gpt1_service_cli;
    rclcpp::Client<chatrover_msgs::srv::TextText>::SharedPtr gpt2_service_cli;

    // ------------------------------- action setting ---------------------------------------
    rclcpp_action::Client<SendPosAction>::SharedPtr client_ptr;
    bool goal_done;
    // 目標設定の受信コールバック関数
    void goal_response_callback(const GoalHandleSendPos::SharedPtr & goal_handle)
    {
        if (!goal_handle) {
        RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
        } else {
        RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
        }
    }

    // フィードバックの受信コールバック関数
    void feedback_callback(GoalHandleSendPos::SharedPtr, const std::shared_ptr<const SendPosAction::Feedback> feedback){
        int x_data = feedback->x;
        int y_data = feedback->y;
        std::cout << "(" << x_data << ", " << y_data << ")" << std::endl;
    }

    // 実行結果の受信コールバック関数
    void result_callback(const GoalHandleSendPos::WrappedResult & result){
        this->goal_done = true;
        switch (result.code) {
        case rclcpp_action::ResultCode::SUCCEEDED:
            break;
        case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
            return;
        case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
            return;
        default:
            RCLCPP_ERROR(this->get_logger(), "Unknown result code");
            return;
        }
        RCLCPP_INFO(this->get_logger(), "Result received");
    }
};

std::shared_ptr<BtRosNode> global_node;