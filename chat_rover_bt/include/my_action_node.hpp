#pragma once
#include <memory>
#include <string>
#include <rclcpp/allocator/allocator_common.hpp>
#include "behaviortree_cpp/behavior_tree.h"
#include "behaviortree_cpp/bt_factory.h"
#include "my_ros_node.hpp"
#include <rclcpp/executors.hpp>
#include "geometry_msgs/msg/twist.hpp"

/*
"GPT1"
input_port "in_voice"
output_port "out_text"
GPT1ノードへテキストを送信して，返ってきたテキストをportにセットする。

"GPT2"
input_port "in_text"
input_port "in_object"
output_port "out_command"
ロボットやオブジェクトの位置も含めてGPT2ノードへテキストを送信し，返ってきたテキストをチェックしてOKならportにセットする。

"SendPos"
input_port "in_command"
portから取得した目標位置のリストとQRの情報を参照しつつcmd_velを出力する。

"VOSK"
output_port "out_voice"
返ってきたテキストをportにセットする。

"YOLO"
output_port "out_object"
返ってきたobjectの位置情報をportにセットする
*/
using namespace BT;

namespace MyActionNodes{
        class UpdateBB : public StatefulActionNode
    {
    public:
        UpdateBB(const std::string& name, const NodeConfig& config) : StatefulActionNode(name, config){ }

        static PortsList providedPorts()
        {
            return { OutputPort<std::string>("action_key"),
                     OutputPort<std::string>("object_name"),
                     OutpubPort<std::bool>("success"),
                     InputPort<std::bool>("success") };
        }

        NodeStatus onStart() override
        {
            std::cout << "call UpdateBB" << std::endl;
            return NodeStatus::RUNNING;
        }

        NodeStatus onRunning() override{
            return NodeStatus::RUNNING;
        }

        void onHalted() override{
            std::cout << "interrupt UpdateBB Node" << std::endl;
        }
    };


    class UpdateBB : public StatefulActionNode
    {
    public:
        UpdateBB(const std::string& name, const NodeConfig& config) : StatefulActionNode(name, config){ }

        static PortsList providedPorts()
        {
            return { OutputPort<std::string>("action_key"),
                     OutputPort<std::string>("object_name"),
                     OutpubPort<std::bool>("success"),
                     InputPort<std::bool>("success") };
        }

        NodeStatus onStart() override
        {
            std::cout << "call UpdateBB" << std::endl;
            bool result = false;
            while(1){
                result = global_node->send_motor_power(true);
                if(result){
                    break;
                }
            }
            return NodeStatus::RUNNING;
        }

        NodeStatus onRunning() override{
            std::cout << "------" << std::endl;
            global_node->sub_gpt_command(gpt_command_data);
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
            setOutput("field_pos", (int)field_state);
            global_node->sub_linear_auto(linear_auto);
            global_node->sub_hand_grip(hand_grip);
            global_node->sub_hand_collect(hand_collect);
            if(hand_grip && !hand_collect && !linear_auto){
                linear_state = "1";
            }else if(!hand_grip && hand_collect && !linear_auto){
                linear_state = "2";
            }else if(!hand_grip && !hand_collect && linear_auto){
                linear_state = "3";
            }else{
                linear_state = "0";
            }
            setOutput("linear_state", linear_state);
            return NodeStatus::RUNNING;
        }

        void onHalted() override{
            std::cout << "interrupt UpdateBB Node" << std::endl;
        }
    private:
        std::string gpt_command_data = "";
    };

//"Search" StatefulActionNode
//input_port "field_state"
//output_port "detect_pos"
    class Search : public StatefulActionNode
    {
    public:
        Search(const std::string& name, const NodeConfig& config) : StatefulActionNode(name, config){ }

        static PortsList providedPorts()
        {
            return { OutputPort<Point>("detect_pos"),
                     InputPort<int>("field_state") };
        }

        NodeStatus onStart() override
        {
            std::cout << "call Search" << std::endl;
            return NodeStatus::RUNNING;
        }

        NodeStatus onRunning() override{
            std::vector<Point> points;
            global_node->sub_scan(points);
            int size = points.size();
            std::vector<std::vector<Point>> clusters;
            std::vector<int> clusterAssignments(size, -1);
            int currentCluster = 0;
            for(int i = 0; i < size; i++){
                if(clusterAssignments[i] == -1){
                    clusterAssignments[i] = currentCluster;
                    for(int j = i + 1; j < size; j++){
                        if(calculateDistance(points[i], points[j]) < point_distance){
                            clusterAssignments[j] = currentCluster;
                        }
                    }
                    currentCluster++;
                }
            }
            std::vector<std::vector<Point>> clusteredPoints(currentCluster);
            for (int i = 0; i < size; ++i) {
                int cluster = clusterAssignments[i];
                clusteredPoints[cluster].push_back(points[i]);
            }
            std::vector<Point> resultPoints;
            for (int cluster = 0; cluster < currentCluster; ++cluster) {
                if (clusteredPoints[cluster].size() >= min_cluster_size) {
                    double totalX = 0.0;
                    double totalY = 0.0;
                    for (const Point& p : clusteredPoints[cluster]) {
                        totalX += p.x;
                        totalY += p.y;
                    }
                    double averageX = totalX / clusteredPoints[cluster].size();
                    double averageY = totalY / clusteredPoints[cluster].size();
                    resultPoints.push_back({averageX, averageY});
                }
            }
            size = resultPoints.size();
            if (size == 0) {
                std::cout << "点群が空です。" << std::endl;
                return NodeStatus::RUNNING;
            }

            Expected<int> msg = getInput<int>("field_state");
            if (!msg)
            {
                throw BT::RuntimeError("missing required input [field_state]: ", msg.error() );
            }
            int field_state = msg.value();
            int idx = 0;
            if(field_state == 1){
                double sum = 0;
                for(int i = 0; i < size; i++){
                    double s = resultPoints[i].y - resultPoints[i].x;
                    if(s < sum){
                        sum = s;
                        idx = i;
                    }
                }
            }else if(field_state == 2){
                double sum = 0;
                for(int i= 0; i < size; i++){
                    double s = resultPoints[i].x + resultPoints[i].y;
                    if(s < sum){
                        sum = s;
                        idx = i;
                    }
                }
            }else{
                std::cout << "field_state error at Search" << std::endl;
            }
            Point detect_pos = resultPoints[idx];
            setOutput("detect_pos", detect_pos);
            return NodeStatus::SUCCESS;
        }

        void onHalted() override{
            std::cout << "interrupt Search Node" << std::endl;
        }
    private:
        double calculateDistance(const Point& p1, const Point& p2) {
            return std::sqrt(std::pow(p1.x - p2.x, 2) + std::pow(p1.y - p2.y, 2));
        }
    };
}