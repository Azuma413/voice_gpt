#include <inttypes.h>
#include <memory>
#include "chatrover_msgs/action/send_pos.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/point.hpp"
#include <thread>

using SendPos = chatrover_msgs::action::SendPos;
using GoalHandleSendPos = rclcpp_action::ServerGoalHandle<SendPos>;
using namespace std::chrono_literals;

struct Point{
    int x;
    int y;
};

constexpr double EPSILON = 50.0;
constexpr double KP_L = 0.0004;
constexpr double KP_A = 0.1;
constexpr double KI_A = 0.001;
constexpr double KD_A = 1.0;
constexpr double MAX_LIN_VEL = 0.1;  // 速度の最大値
constexpr double MAX_ANG_VEL = 1.0;  // 角速度の最大値 0.6くらいだとなめらかに回転する

class SendPosActionServer : public rclcpp::Node
{
public:
  explicit SendPosActionServer() : Node("pos2vel_node")
  {
    std::cout << "サーバー起動" << std::endl;
    //ARからロボットの状態を取得
    auto robot_state_cb = [this](const geometry_msgs::msg::Twist& msg) -> void{robot_state = msg;};

    using namespace std::placeholders;
    
    // Actionサーバーの初期化
    this->action_server = rclcpp_action::create_server<SendPos>(
      this->get_node_base_interface(),
      this->get_node_clock_interface(),
      this->get_node_logging_interface(),
      this->get_node_waitables_interface(),
      "send_pos",
      std::bind(&SendPosActionServer::handle_goal, this, _1, _2),
      std::bind(&SendPosActionServer::handle_cancel, this, _1),
      std::bind(&SendPosActionServer::handle_accepted, this, _1)
    );

    rclcpp::QoS qos(rclcpp::KeepLast(10));
    cmd_vel_pub = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", qos);
    robot_state_sub = this->create_subscription<geometry_msgs::msg::Twist>("/robot_state", qos, robot_state_cb);
    angle_pub = this->create_publisher<geometry_msgs::msg::Point>("/angle_data", qos);
  }

private:
  double distance = 100.0, error, integral_error, prev_error, P_term, I_term, D_term;
  Point robot_point, set_point;
  geometry_msgs::msg::Twist robot_state;
  geometry_msgs::msg::Twist pub_msg;
  geometry_msgs::msg::Point angle_data;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr robot_state_sub;
  rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr angle_pub;

  // 2点の距離を求める関数
  double calcDistance(const Point& point1, const Point& point2) {
    int dx = point1.x - point2.x;
    int dy = point1.y - point2.y;
    double d = std::sqrt(dx * dx + dy * dy);
    return d;
  }

  rclcpp_action::Server<SendPos>::SharedPtr action_server;
  // 目標値の設定時に呼び出されるハンドラ
  rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const SendPos::Goal> goal){
    //リクエストを保存
    set_point.x = static_cast<int>(goal->x);
    set_point.y = static_cast<int>(goal->y);
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
    rclcpp::Rate loop_rate(100);
    //const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<SendPos::Feedback>();
    auto result = std::make_shared<SendPos::Result>();

    while(distance > EPSILON && rclcpp::ok()){
      //robot_pointにロボットの現在位置を保存
      robot_point.x = static_cast<int>(robot_state.linear.x);
      robot_point.y = static_cast<int>(robot_state.linear.y);
      distance = this->calcDistance(robot_point, set_point);
      //ロボットの現在位置からみた目標位置の絶対角度を求める
      angle_data.y = std::atan2(set_point.y - robot_point.y, set_point.x - robot_point.x) + M_PI/2;
      if (angle_data.y > M_PI){
        angle_data.y -= 2*M_PI;
      }
      //ロボットの現在の絶対角度を保存
      angle_data.x = robot_state.angular.z;
      // 目標にある程度近づいtら積分項をリセット
      if (distance < EPSILON*2.0){
        integral_error = 0.0;
      }
      // 目標角度と現在角度の誤差を計算
      error = angle_data.y - angle_data.x;
      angle_data.z = error;
      // 比例項
      P_term = KP_A * error;
      // 積分項
      integral_error += error;
      I_term = KI_A * integral_error;
      // 微分項
      D_term = KD_A * (error - prev_error);
      prev_error = error;
      double pid = P_term + I_term + D_term;
      //最大速度と距離の定数倍を比較して小さい方を速度にする
      pub_msg.linear.x = std::min(MAX_LIN_VEL, KP_L*distance);
      //最大角速度と相対角度の定数倍を比較して小さい方を角速度にする
      if (pid > 0.0){
        pub_msg.angular.z = -std::min(MAX_ANG_VEL, pid);
      }else{
        pub_msg.angular.z = -std::max(-MAX_ANG_VEL, pid);
      }
      //std::cout << "P:" << P_term << " I:" << I_term << " D:" << D_term << " PID(<0.5):" << P_term + I_term + D_term << std::endl;
      std::cout << "前：" << pub_msg.linear.x << " 時計回り：" << pub_msg.angular.z << std::endl;
      this->cmd_vel_pub->publish(pub_msg);
      this->angle_pub->publish(angle_data);
      
      // アクションがキャンセルされていないか確認
      if (goal_handle->is_canceling()) {
        result->success = false;
        goal_handle->canceled(result);
        RCLCPP_INFO(this->get_logger(), "Goal Canceled");
        return;
      }
      // フィードバックの送信
      feedback->x = robot_state.linear.x;
      feedback->y = robot_state.linear.y;
      goal_handle->publish_feedback(feedback);
      RCLCPP_INFO(this->get_logger(), "Publish Feedback");
      loop_rate.sleep();
    }
    std::cout << "目標到達" << std::endl;
    distance = 100.0;
    pub_msg.linear.x = 0;
    pub_msg.angular.z = 0;
    this->cmd_vel_pub->publish(pub_msg);
    this->angle_pub->publish(angle_data);

    if (rclcpp::ok()) {
      result->success = true;
      goal_handle->succeed(result);
      RCLCPP_INFO(this->get_logger(), "Goal Succeeded");
    }
  }

  void execute_test(const std::shared_ptr<GoalHandleSendPos> goal_handle){
    rclcpp::Rate loop_rate(1);
    auto feedback = std::make_shared<SendPos::Feedback>();
    auto result = std::make_shared<SendPos::Result>();
    int count = 0;
    for(int count = 0; count < 10 && rclcpp::ok(); count ++){
      // アクションがキャンセルされていないか確認
      if (goal_handle->is_canceling()) {
        result->success = false;
        goal_handle->canceled(result);
        RCLCPP_INFO(this->get_logger(), "Goal Canceled");
        return;
      }
      // フィードバックの送信
      feedback->x = count;
      feedback->y = count;
      goal_handle->publish_feedback(feedback);
      RCLCPP_INFO(this->get_logger(), "Publish Feedback");
      loop_rate.sleep();
    }
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