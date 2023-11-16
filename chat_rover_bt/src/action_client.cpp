//こちらはROS2 Action通信の実証のためのコードである。

//参考にすべきコード
//https://docs.ros.org/en/humble/Tutorials/Intermediate/Writing-an-Action-Server-Client/Cpp.html

#include <inttypes.h>
#include <memory>
#include <string>
#include <iostream>
#include "chatrover_msgs/action/send_pos.hpp"
#include "rclcpp/rclcpp.hpp"
// TODO(jacobperron): Remove this once it is included as part of 'rclcpp.hpp'
#include "rclcpp_action/rclcpp_action.hpp"

using SendPos = chatrover_msgs::action::SendPos;
using GoalHandleSendPos = rclcpp_action::ClientGoalHandle<SendPos>;

class SendPosActionClient : public rclcpp::Node
{
public:
  explicit SendPosActionClient() : Node("minimal_action_client"), goal_done(false){
    this->client_ptr = rclcpp_action::create_client<SendPos>(this->get_node_base_interface(),this->get_node_graph_interface(),this->get_node_logging_interface(),this->get_node_waitables_interface(),"send_pos");
    this->timer = this->create_wall_timer(std::chrono::milliseconds(500),std::bind(&SendPosActionClient::send_goal, this));
  }

  //ゲッター関数
  bool is_goal_done() const{
    return this->goal_done;
  }

  // timerで実行される
  void send_goal(){
    using namespace std::placeholders;
    //タイマーを一旦止める
    this->timer->cancel();
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
      return;
    }

  // 目標値を設定
    auto goal_msg = chatrover_msgs::action::SendPos::Goal();
    goal_msg.x = 10;
    goal_msg.y = 20;
    RCLCPP_INFO(this->get_logger(), "Sending goal");
  // 目標値のアクションサーバー送信とコールバックメソッドの登録
    auto send_goal_options = rclcpp_action::Client<SendPos>::SendGoalOptions();
    send_goal_options.goal_response_callback = std::bind(&SendPosActionClient::goal_response_callback, this, _1);
    send_goal_options.feedback_callback = std::bind(&SendPosActionClient::feedback_callback, this, _1, _2);
    send_goal_options.result_callback = std::bind(&SendPosActionClient::result_callback, this, _1);
    auto goal_handle_future = this->client_ptr->async_send_goal(goal_msg, send_goal_options);
  }

private:
  rclcpp_action::Client<SendPos>::SharedPtr client_ptr;
  rclcpp::TimerBase::SharedPtr timer;
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
  /*
  void goal_response_callback(std::shared_future<GoalHandleSendPos::SharedPtr> future){
    auto goal_handle = future.get();
    if (!goal_handle) {
      RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
    } else {
      RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
    }
  }
  */

  // フィードバックの受信コールバック関数
  void feedback_callback(GoalHandleSendPos::SharedPtr, const std::shared_ptr<const SendPos::Feedback> feedback){
    int x_data = feedback->x;
    int y_data = feedback->y;
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

int main(int argc, char ** argv){
  rclcpp::init(argc, argv);
  auto action_client = std::make_shared<SendPosActionClient>();
  while (!action_client->is_goal_done()) {
    rclcpp::spin_some(action_client);
  }

  rclcpp::shutdown();
  return 0;
}