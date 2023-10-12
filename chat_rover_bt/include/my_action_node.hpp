#pragma once
#include <memory>
#include <string>
#include <rclcpp/allocator/allocator_common.hpp>
#include "behaviortree_cpp/behavior_tree.h"
#include "behaviortree_cpp/bt_factory.h"
#include "my_ros_node.hpp"
#include <rclcpp/executors.hpp>
#include "geometry_msgs/msg/twist.hpp"

using namespace BT;

struct Point{
    int x;
    int y;
};

constexpr double EPSILON = 10.0;
constexpr double LIN_VEL = 0.001;
constexpr double ANG_VEL = 0.1;
constexpr double MAX_LIN_VEL = 0.5;  // 速度の最大値
constexpr double MAX_ANG_VEL = 1.0;  // 角速度の最大値

namespace MyActionNodes{
/*
"GPT1"
input_port "in_voice"
output_port "out_text"
GPT1ノードへテキストを送信して，返ってきたテキストをportにセットする。
*/
    class GPT1 : public StatefulActionNode
    {
    public:
        GPT1(const std::string& name, const NodeConfig& config) : StatefulActionNode(name, config){ }

        static PortsList providedPorts()
        {
            return { InputPort<std::string>("in_voice"),
                     OutputPort<std::string>("out_text") };
        }

        NodeStatus onStart() override
        {
            std::cout << "call GPT1" << std::endl;
            Expected<std::string> msg = getInput<std::string>("in_voice");
            if (!msg){
                throw BT::RuntimeError("missing required input [in_voice]: ", msg.error() );
            }
            send_data = msg.value();
            return NodeStatus::RUNNING;
        }

        NodeStatus onRunning() override{
            global_node->send_gpt1_service(send_data, get_data);
            if(get_data == "NULL"){
                return NodeStatus::FAILURE;
            }else{
                setOutput("out_text", get_data);
                return NodeStatus::SUCCESS;
            }
        }

        void onHalted() override{
            std::cout << "interrupt GPT1 Node" << std::endl;
        }
    private:
        std::string send_data;
        std::string get_data;
    };

/*
"GPT2"
input_port "in_text"
input_port "in_object"
output_port "out_command"
ロボットやオブジェクトの位置も含めてGPT2ノードへテキストを送信し，返ってきたテキストをチェックしてOKならportにセットする。
*/
    class GPT2 : public StatefulActionNode
    {
    public:
        GPT2(const std::string& name, const NodeConfig& config) : StatefulActionNode(name, config){ }

        static PortsList providedPorts()
        {
            return { InputPort<std::string>("in_text"),
                     InputPort<std::string>("in_object"),
                     OutputPort<std::string>("out_command") };
        }

        NodeStatus onStart() override
        {
            std::cout << "call GPT2" << std::endl;
            Expected<std::string> msg1 = getInput<std::string>("in_text");
            if (!msg1){
                throw BT::RuntimeError("missing required input [in_text]: ", msg1.error() );
            }
            send_data = msg1.value();
            Expected<std::string> msg2 = getInput<std::string>("in_object");
            if (!msg2){
                throw BT::RuntimeError("missing required input [in_object]: ", msg2.error() );
            }
            send_data += "/n";
            send_data += msg2.value();
            return NodeStatus::RUNNING;
        }

        NodeStatus onRunning() override{
            global_node->send_gpt2_service(send_data, get_data);
            setOutput("out_command", get_data);
            return NodeStatus::SUCCESS;
        }

        void onHalted() override{
            std::cout << "interrupt GPT2 Node" << std::endl;
        }
    private:
        std::string send_data;
        std::string get_data;
    };

/*Motor fails to start
"SendPos"
input_port "in_command"
portから取得した目標位置のリストとQRの情報を参照しつつcmd_velを出力する。
*/
    class SendPos : public StatefulActionNode
    {
    public:
        SendPos(const std::string& name, const NodeConfig& config) : StatefulActionNode(name, config){ }

        static PortsList providedPorts()
        {
            return { InputPort<std::string>("in_command") };
        }

        NodeStatus onStart() override
        {
            std::cout << "call SendPos" << std::endl;
            Expected<std::string> msg = getInput<std::string>("in_command");
            if (!msg){
                throw BT::RuntimeError("missing required input [in_command]: ", msg.error() );
            }
            command_message = msg.value();
            //expect format "{(1, 2), (3, 4), (5, 6)}"
            try{
                points = extractCoordinates(command_message);
            }catch(const std::runtime_error& e){
                std::cout << "Error:" << e.what() << std::endl;
                return NodeStatus::FAILURE;
            }
            size = points.size();
            return NodeStatus::RUNNING;
        }

        NodeStatus onRunning() override{
            Point robot_point, set_point = points[count];
            double distance = 0.0;
            do{
                geometry_msgs::msg::Twist robot_state;
                global_node->sub_robot_state(robot_state);
                robot_point.x = robot_state.linear.x;
                robot_point.y = robot_state.linear.y;
                distance = calcDistance(robot_point, set_point);
                double target_angle = std::atan2(set_point.y - robot_point.y, set_point.x - robot_point.x);
                double current_angle = robot_state.angular.z;
                double linear_vel = std::min(MAX_LIN_VEL, LIN_VEL*distance);
                double angular_vel = std::min(MAX_ANG_VEL, ANG_VEL*(target_angle - current_angle));
                global_node->pub_cmd_vel(linear_vel, angular_vel);
            }while(distance > EPSILON);
            global_node->pub_cmd_vel(0.0, 0.0);
            count++;
            if(count >= size){
                return NodeStatus::SUCCESS;
            }else{
                return NodeStatus::RUNNING;
            }
        }

        void onHalted() override{
            std::cout << "interrupt SendPos Node" << std::endl;
        }
    private:
        int size = 0;
        int count = 0;
        std::string command_message;
        std::vector<Point> points;
        bool parseCoordinate(std::string coordinate, Point& point) {
            std::istringstream ss(coordinate);
            char discard;
            if (ss >> discard >> point.x >> discard >> point.y >> discard) {
                return true;
            }
            return false;
        }
        std::vector<Point> extractCoordinates(const std::string& input) {
            std::vector<Point> coordinates;
            std::istringstream ss(input);
            char discard;
            if (ss >> discard) {  // 最初の {
                while (true) {
                    Point point;
                    if (ss >> discard) {  // ( を読み飛ばす
                        std::string coordinate;
                        if (std::getline(ss, coordinate, ')')) {
                            if (parseCoordinate(coordinate, point)) {
                                coordinates.push_back(point);
                                if (!(ss >> discard)) {  // 最後の ) を読み飛ばす
                                    break;
                                }
                            } else {
                                throw std::runtime_error("Invalid coordinate format.");
                            }
                        } else {
                            throw std::runtime_error("Invalid coordinate format.");
                        }
                    } else {
                        throw std::runtime_error("Invalid coordinate format.");
                    }
                }
            } else {
                throw std::runtime_error("Invalid input format.");
            }

            return coordinates;
        }
        double calcDistance(const Point& point1, const Point& point2) {
            int dx = point1.x - point2.x;
            int dy = point1.y - point2.y;
            double distance = std::sqrt(dx * dx + dy * dy);
            return distance;
        }
    };

/*
"VOSK"
output_port "out_voice"
返ってきたテキストをportにセットする。
*/
    class VOSK : public StatefulActionNode
    {
    public:
        VOSK(const std::string& name, const NodeConfig& config) : StatefulActionNode(name, config){ }

        static PortsList providedPorts()
        {
            return { OutputPort<std::string>("out_voice") };
        }

        NodeStatus onStart() override
        {
            std::cout << "call VOSK" << std::endl;
            return NodeStatus::RUNNING;
        }

        NodeStatus onRunning() override{
            std::string data;
            global_node->send_get_voice(data);
            setOutput("out_voice", data);
            return NodeStatus::SUCCESS;
        }

        void onHalted() override{
            std::cout << "interrupt VOSK Node" << std::endl;
        }
    };

/*
"YOLO"
output_port "out_object"
返ってきたobjectの位置情報をportにセットする
*/
    class YOLO : public StatefulActionNode
    {
    public:
        YOLO(const std::string& name, const NodeConfig& config) : StatefulActionNode(name, config){ }

        static PortsList providedPorts()
        {
            return { OutputPort<std::string>("out_object") };
        }

        NodeStatus onStart() override
        {
            std::cout << "call YOLO" << std::endl;
            return NodeStatus::RUNNING;
        }

        NodeStatus onRunning() override{
            std::string data;
            global_node->send_get_object(data); //AIに入力可能な形式での文字列が期待される
            setOutput("out_object", data);
            return NodeStatus::SUCCESS;
        }

        void onHalted() override{
            std::cout << "interrupt YOLO Node" << std::endl;
        }
    };
}