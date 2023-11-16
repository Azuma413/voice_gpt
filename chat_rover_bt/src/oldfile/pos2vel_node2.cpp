#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "chatrover_msgs/srv/send_pos.hpp"
#include "geometry_msgs/msg/point.hpp"
#include <thread>

using namespace std::chrono_literals;

struct Point{
    int x;
    int y;
};

constexpr double EPSILON = 10.0;
constexpr double KP_L = 0.0001;
constexpr double KP_A = 0.1;
constexpr double KI_A = 0.1;
constexpr double KD_A = 0.1;
constexpr double MAX_LIN_VEL = 0.1;  // 速度の最大値
constexpr double MAX_ANG_VEL = 0.5;  // 角速度の最大値 0.6くらいだとなめらかに回転する

class SendPosServer : public rclcpp::Node{
  public:
  SendPosServer() : Node("send_pos_server"){
    std::cout << "サーバー起動" << std::endl;
    //ARからロボットの状態を取得
    auto robot_state_cb = [this](const geometry_msgs::msg::Twist& msg) -> void{robot_state = msg;};
    //10ms毎に実行される関数
    auto timer_callback = [this]() -> void{
      //std::cout << "timer" << std::endl;
      //serviceから指令があれば実行
      if(!flag){
        //robot_pointにロボットの現在位置を保存
        robot_point.x = static_cast<int>(robot_state.linear.x);
        robot_point.y = static_cast<int>(robot_state.linear.y);
        //ロボットの現在位置と目標位置の距離を求める
        distance = this->calcDistance(robot_point, set_point);
        //目標点との距離がepsilon以下であればflagをtrueにする。
        if(distance < EPSILON){
          std::cout << "目標到達" << std::endl;
          flag = true;
          pub_msg.linear.x = 0;
          pub_msg.angular.z = 0;
        }else{
          //ロボットの現在位置からみた目標位置の絶対角度を求める
          angle_data.y = -std::atan2(set_point.x - robot_point.x, set_point.y - robot_point.y);
          //ロボットの現在の絶対角度を保存
          angle_data.x = robot_state.angular.z;
          //角度を表示
          std::cout << "\ntarget: " << angle_data.y/M_PI*180.0 << "\ncurrent: " << angle_data.x/M_PI*180 << std::endl;
          // 目標角度と現在角度の誤差を計算
          error = angle_data.y - angle_data.x;
          // 比例項
          P_term = KP_A * error;
          // 積分項
          integral_error += error;
          I_term = KI_A * integral_error;
          // 微分項
          D_term = KD_A * (error - prev_error);
          prev_error = error;
          //最大速度と距離の定数倍を比較して小さい方を速度にする
          pub_msg.linear.x = std::min(MAX_LIN_VEL, KP_L*distance);
          //最大角速度と相対角度の定数倍を比較して小さい方を角速度にする
          pub_msg.angular.z = std::min(MAX_ANG_VEL, P_term + I_term + D_term);
        }
      }
      this->cmd_vel_pub->publish(pub_msg);
      this->angle_pub->publish(angle_data);
    };
    auto service_callback = [this](const std::shared_ptr<chatrover_msgs::srv::SendPos::Request> request, std::shared_ptr<chatrover_msgs::srv::SendPos::Response> response) -> void{
      std::cout << "service 呼び出し" << std::endl;
      //リクエストを保存
      set_point.x = static_cast<int>(request->x);
      set_point.y = static_cast<int>(request->y);
      flag = false;
      //flag == trueとなるまで待機
      std::thread([this, response]() {
        while (!flag && rclcpp::ok()) {
          std::this_thread::sleep_for(10ms);
        }
        std::cout << "ループ待機終了" << std::endl;
        response->success = true;
      }).detach();
      std::cout << "呼び出し終了" << std::endl;
    };
    rclcpp::QoS qos(rclcpp::KeepLast(10));
    cmd_vel_pub = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", qos);
    robot_state_sub = this->create_subscription<geometry_msgs::msg::Twist>("/robot_state", qos, robot_state_cb);
    send_pos_srv = this->create_service<chatrover_msgs::srv::SendPos>("/send_pos", service_callback);
    timer = this->create_wall_timer(10ms, timer_callback);
    angle_pub = this->create_publisher<geometry_msgs::msg::Point>("/angle_data", qos);
    executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  }

  void run(){
    std::cout << "run() 呼び出し" << std::endl;
    executor->add_node(shared_from_this());
    while (rclcpp::ok()) {
      executor->spin_some();
      std::this_thread::sleep_for(10ms);
    }
  }

  private:
  bool flag = true;
  double distance = 100.0, error, integral_error, prev_error, P_term, I_term, D_term;
  Point robot_point, set_point;
  geometry_msgs::msg::Twist robot_state, pub_msg;
  geometry_msgs::msg::Point angle_data;
  rclcpp::TimerBase::SharedPtr timer;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr robot_state_sub;
  rclcpp::Service<chatrover_msgs::srv::SendPos>::SharedPtr send_pos_srv;
  rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr angle_pub;
  rclcpp::executors::SingleThreadedExecutor::SharedPtr executor;

  double calcDistance(const Point& point1, const Point& point2) {
    int dx = point1.x - point2.x;
    int dy = point1.y - point2.y;
    double distance = std::sqrt(dx * dx + dy * dy);
    return distance;
  }
};

int main(int argc, char * argv[])
{
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);
  rclcpp::init(argc, argv);
  auto server = std::make_shared<SendPosServer>();
  server->run();
  rclcpp::shutdown();
  return 0;
}