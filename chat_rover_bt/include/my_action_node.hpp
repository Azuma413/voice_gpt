#pragma once
#include <memory>
#include <string>
#include <rclcpp/allocator/allocator_common.hpp>
#include "behaviortree_cpp/behavior_tree.h"
#include "behaviortree_cpp/bt_factory.h"
#include "my_ros_node.hpp"
#include <rclcpp/executors.hpp>
#include "geometry_msgs/msg/twist.hpp"
#include <nlohmann/json.hpp>

using namespace BT;
using json = nlohmann::json;

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
      <input_port name="in_voice"/>
      <output_port name="list_len"/>
      <output_port name="out_text"/>
GPT1ノードへテキストを送信して，返ってきたテキストをportにセットする。//帰ってくる文字列はリストの形になっているので、分割してinput
*/
    class GPT1 : public StatefulActionNode
    {
    public:
        GPT1(const std::string& name, const NodeConfig& config) : StatefulActionNode(name, config){ }

        static PortsList providedPorts()
        {
            return { InputPort<std::string>("in_voice"),
                     OutputPort<int>("list_len"),
                     OutputPort<std::string>("out_text") };
        }

        NodeStatus onStart() override
        {
            std::cout << "\ncall GPT1" << std::endl;
            Expected<std::string> msg = getInput<std::string>("in_voice");
            if (!msg){
                throw BT::RuntimeError("missing required input [in_voice]: ", msg.error() );
            }
            send_data = msg.value();
            std::cout << "[in_voice]:" << send_data << std::endl;
            return NodeStatus::RUNNING;
        }

        NodeStatus onRunning() override{
            global_node->send_gpt1_service(send_data, get_data);
            if(get_data == "NULL"){
                return NodeStatus::FAILURE;
            }else{
                //分割する必要はないので、"を数えればいい
                int count = 0;
                for (char character : get_data) {
                    if (character == target) {
                        count++;
                    }
                }
                count = count/2 - 1;
                std::cout << "[list_len]:" << count << std::endl;
                setOutput("list_len", count);
                std::cout << "[out_text]:" << get_data << std::endl;
                setOutput("out_text", get_data);
                return NodeStatus::SUCCESS;
            }
        }

        void onHalted() override{
            std::cout << "interrupt GPT1 Node" << std::endl;
        }
    private:
        char target = '"';
        std::string send_data;
        std::string get_data;
    };

/*
"GPT2"
input_port "in_text"
input_port "in_object"
<input_port name="text_len"/>
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
                     InputPort<int>("text_len"),
                     OutputPort<std::string>("out_command") };
        }

        NodeStatus onStart() override
        {
            std::cout << "\ncall GPT2" << std::endl;
            if(count == 0){
                //初回のみ実行する処理
                Expected<int> msg0 = getInput<int>("text_len");
                if(!msg0){
                    throw BT::RuntimeError("missing required input [text_len]: ", msg0.error());
                }
                list_len = msg0.value();
                std::cout << "[text_len]:" << list_len << std::endl;
                Expected<std::string> msg1 = getInput<std::string>("in_text");
                if (!msg1){
                    throw BT::RuntimeError("missing required input [in_text]: ", msg1.error() );
                }
                //期待される入力(json)
                // {"instruction":["text1", "text2", ...]}
                std::cout << "[in_text]:" << msg1.value() << std::endl;
                try{
                    json jsonData = json::parse(msg1.value().c_str());
                    text_list = jsonData["instruction"].get<std::vector<std::string>>();
                }catch(const std::runtime_error& e){
                    std::cout << "Error:" << e.what() << std::endl;
                    return NodeStatus::FAILURE;
                }
            }

            std::ostringstream oss;
            oss << "instruction: " << text_list[count];
            Expected<std::string> msg2 = getInput<std::string>("in_object");
            if (!msg2){
                throw BT::RuntimeError("missing required input [in_object]: ", msg2.error() );
            }
            oss << "/nobject position: " << msg2.value();
            geometry_msgs::msg::Twist robot_state;
            global_node->sub_robot_state(robot_state);
            oss << "/nyour position: (" << robot_state.linear.x << ", " << robot_state.linear.y << ")";
            send_data = oss.str();
            std::cout << "send_data: " << send_data << std::endl;
            return NodeStatus::RUNNING;
        }

        NodeStatus onRunning() override{
            global_node->send_gpt2_service(send_data, get_data);
            if(get_data == "NULL"){
                return NodeStatus::FAILURE;
            }else{
                std::cout << "[out_command]:" << get_data << std::endl;
                setOutput("out_command", get_data);
                count++;
                if(count == list_len){
                    count = 0;
                }
                return NodeStatus::SUCCESS;
            }
        }

        void onHalted() override{
            std::cout << "interrupt GPT2 Node" << std::endl;
        }
    private:
        std::vector<std::string> text_list;
        int count = 0;
        int list_len;
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
            std::cout << "\ncall SendPos" << std::endl;
            Expected<std::string> msg = getInput<std::string>("in_command");
            if (!msg){
                throw BT::RuntimeError("missing required input [in_command]: ", msg.error() );
            }
            std::cout << "[in_command]:" << msg.value() << std::endl;
            try{
                json jsonData = json::parse(msg.value().c_str());
                for (const auto& coordinate : jsonData["coordinates"]) {
                    Point point;
                    point.x = coordinate["x"].get<int>();
                    point.y = coordinate["y"].get<int>();
                    points.push_back(point);
                    if(point.x == -1 && point.y == -1){
                        return NodeStatus::FAILURE;
                    }
                }
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
            }while(distance > EPSILON && rclcpp::ok());
            
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
            std::cout << "\ncall VOSK" << std::endl;
            return NodeStatus::RUNNING;
        }

        NodeStatus onRunning() override{
            std::string data;
            global_node->send_get_voice(data);
            std::cout << "[out_voice]:" << data << std::endl;
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
            std::cout << "\ncall YOLO" << std::endl;
            return NodeStatus::RUNNING;
        }

        NodeStatus onRunning() override{
            std::string data;
            global_node->send_get_object(data); //AIに入力可能な形式での文字列が期待される
            std::cout << "[out_object]:" << data << std::endl;
            setOutput("out_object", data);
            return NodeStatus::SUCCESS;
        }

        void onHalted() override{
            std::cout << "interrupt YOLO Node" << std::endl;
        }
    };
}