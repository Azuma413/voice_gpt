#include <inttypes.h>
#include <memory>
#include "chatrover_msgs/action/send_pos.hpp"
#include "rclcpp/rclcpp.hpp"
// TODO(jacobperron): Remove this once it is included as part of 'rclcpp.hpp'
#include "rclcpp_action/rclcpp_action.hpp"

using SendPos = chatrover_msgs::action::SendPos;
using GoalHandleSendPos = rclcpp_action::ServerGoalHandle<SendPos>;

class SendPosActionServer : public rclcpp::Node
{
public:
  explicit SendPosActionServer() : Node("send_pos_action_server")
  {
    using namespace std::placeholders;
    this->action_server = rclcpp_action::create_server<SendPos>(this->get_node_base_interface(),this->get_node_clock_interface(),this->get_node_logging_interface(),this->get_node_waitables_interface(),"fibonacci",std::bind(&SendPosActionServer::handle_goal, this, _1, _2),std::bind(&SendPosActionServer::handle_cancel, this, _1),std::bind(&SendPosActionServer::handle_accepted, this, _1));
  }

private:
  rclcpp_action::Server<SendPos>::SharedPtr action_server;
  // 目標値の設定時に呼び出されるハンドラ
  rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const SendPos::Goal> goal){
    RCLCPP_INFO(this->get_logger(), "Received goal request with order (%ld, %ld)", goal->x, goal->y);
    (void)uuid;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  // アクションのキャンセル時に呼び出されるハンドラ
  rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleSendPos> goal_handle){
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  // アクション実行の中身
  void execute(const std::shared_ptr<GoalHandleSendPos> goal_handle){
    RCLCPP_INFO(this->get_logger(), "Executing goal");
    rclcpp::Rate loop_rate(1);
    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<SendPos::Feedback>();
    auto& x = feedback->x;
    auto& y = feedback->y;
    x = 10;
    y = 20;
    auto result = std::make_shared<SendPos::Result>();

    //for (int i = 1; (i < goal->order) && rclcpp::ok(); ++i) {
      // アクションがキャンセルされていないか確認
      if (goal_handle->is_canceling()) {
        result->success = false;
        goal_handle->canceled(result);
        RCLCPP_INFO(this->get_logger(), "Goal Canceled");
        return;
      }
      // フィードバックの送信
      goal_handle->publish_feedback(feedback);
      RCLCPP_INFO(this->get_logger(), "Publish Feedback");
     // loop_rate.sleep();
    //}

    // 目標到達の確認
    if (rclcpp::ok()) {
      result->success = true;
      goal_handle->succeed(result);
      RCLCPP_INFO(this->get_logger(), "Goal Succeeded");
    }
  }

  // アクションの実行開始時に呼び出されるハンドラ
  void handle_accepted(const std::shared_ptr<GoalHandleSendPos> goal_handle)
  {
    using namespace std::placeholders;
    // executeメソッドの実行でExecutorの処理がブロックされないように、
    // スレッド実行
    std::thread{std::bind(&SendPosActionServer::execute, this, _1), goal_handle}.detach();
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto action_server = std::make_shared<SendPosActionServer>();
  rclcpp::spin(action_server);
  rclcpp::shutdown();
  return 0;
}
