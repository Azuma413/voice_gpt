#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include <chrono>
#include <cstdlib>
#include <memory>

using namespace std::chrono_literals;

class SimpleClient : public rclcpp::Node{
  public:
  SimpleClient() : Node("motor_power_client"){
    bool flag;
    motor_power_cli = create_client<std_srvs::srv::SetBool>("/motor_power");
    while(!motor_power_cli->wait_for_service(1s)){
      if(!rclcpp::ok()){
        std::cout << "motor_power service not available" << std::endl;
        flag = true;
        break;
      }
      if(flag){
        std::cout << "motor_power service not available" << std::endl;
        flag = false;
      }
    }
    std::cout << "motor_power service available" << std::endl;

    //モーターパワーを一旦切る
    bool result = false;
    send_motor_power(false, result);
    if(result){
      std::cout << "Succeeded in turning off power to motor!" << std::endl;
    }else{
      std::cout << "Failed to turn off power to motor." << std::endl;
    }

    //モーターパワーを入れる
    send_motor_power(true, result);
    if(result){
      std::cout << "Motor success to start!" << std::endl;
    }else{
      std::cout << "Motor fails to start!" << std::endl;
    }
  }
  void send_motor_power(bool power, bool& result){
    auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
    request->data = power;
    auto future_result = motor_power_cli->async_send_request(request);
    if (rclcpp::spin_until_future_complete(this->shared_from_this(), future_result) == rclcpp::FutureReturnCode::SUCCESS) {
      result = future_result.get()->success;
      return;
    }
    std::cout << "can't get future_result" << std::endl;
    return;
  }
  private:
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr motor_power_cli;

};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SimpleClient>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}