#include "behaviortree_cpp/bt_factory.h"
#include "../include/my_action_node.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "rclcpp/rclcpp.hpp"
#include "../include/my_ros_node.hpp"

using namespace MyActionNodes;
using namespace BT;


int main(int argc, char* argv[]){
  rclcpp::init(argc, argv);
  global_node = std::make_shared<BtRosNode>();
  BehaviorTreeFactory factory;
  factory.registerNodeType<GPT1>("GPT1");
  factory.registerNodeType<GPT2>("GPT2");
  factory.registerNodeType<SendPos>("SendPos");
  factory.registerNodeType<VOSK>("VOSK");
  factory.registerNodeType<YOLO>("YOLO");
  std::string package_path = ament_index_cpp::get_package_share_directory("chat_rover_bt");
  factory.registerBehaviorTreeFromFile(package_path + "/config/main_bt.xml");
  BT::Tree tree = factory.createTree("MainTree");
  printTreeRecursively(tree.rootNode());
  NodeStatus status = NodeStatus::RUNNING;

  while(status == NodeStatus::RUNNING && rclcpp::ok()){
    rclcpp::spin_some(global_node);
    status = tree.tickOnce();
  }

  rclcpp::shutdown();
  return 0;
}



