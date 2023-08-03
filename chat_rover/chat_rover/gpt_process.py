import openai
import os
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from chatrover_msgs.msg import ObjectPosition

openai.api_key = os.getenv("OPENAI_API_KEY")

system_conf = {"role": "system", "content": "Conversations are given in the following format.\n{Name}: {Content}\nName = Your conversation partner's name\nContent = content of speech"}
class ChatGPTClass:
    def __init__(self):
        self.messages = []
    def chatProcess(self, userMessage):
        if len(self.messages)>10:
            del self.messages[0:2]
        # ユーザの回答を保存
        self.messages += [system_conf, {"role": "user", "content": userMessage}, {"role": "assistant", "content": 'response : '}]

        # APIを使ってChatGPTの回答を生成
        response = openai.ChatCompletion.create(model="gpt-3.5-turbo",messages=self.messages)
        ai_response = response['choices'][0]['message']['content']
        # AIの回答を保存
        self.messages[-1]["content"] += ai_response
        del self.messages[-3]
        #print(self.messages, flush=True)
        return ai_response
    
class GPTController(Node):

    def __init__(self):
        super().__init__('gpt_controller')
        self.publisher = self.create_publisher(String, '/gpt_command', 10)
        self.create_subscription(ObjectPosition, '/object_pos', self.yolo_callback, 10)
        self.create_subscription(String, '/voice_text', self.vosk_callback, 10)
        self.detect_object_list = ""
        self.chat = ChatGPTClass()

    def yolo_callback(self, msg):
        self.detect_object_list = ""
        if len(msg.name) == 0:
            return
        for i in range(len(msg.name)):
            self.detect_object_list += "{" + msg.name[i] + ":[x=" + str(msg.position[i].x) + ", y=" + str(msg.position[i].y) + ", z=" + str(msg.position[i].z) +  "}, "

    def vosk_callback(self, msg):
        send_commands = String()
        input_data = "commands:" + msg.data + "\nobject data:" + self.detect_object_list
        print(input_data, flush=True)
        #send_commands.data = self.chat.chatProcess(input_data)
        #self.publisher.publish(send_commands)

def main(args=None):
    rclpy.init(args=args)
    controller = GPTController()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

"""
sample
commands:おはようございます
object data:{person:[x=559.481201171875, y=411.8475036621094, z=0.0}, {bed:[x=559.481201171875, y=411.8475036621094, z=0.0}, {bed:[x=559.481201171875, y=411.8475036621094, z=0.0}, {bed:[x=559.481201171875, y=411.8475036621094, z=0.0}, {backpack:[x=559.481201171875, y=411.8475036621094, z=0.0}, {handbag:[x=559.481201171875, y=411.8475036621094, z=0.0}, 
commands:今日もいい天気ですね
object data:{person:[x=582.73583984375, y=412.4528503417969, z=0.0}, {bed:[x=582.73583984375, y=412.4528503417969, z=0.0}, {bed:[x=582.73583984375, y=412.4528503417969, z=0.0}, {bed:[x=582.73583984375, y=412.4528503417969, z=0.0}, 
commands:赤いボールを取ってきてください
object data:{person:[x=410.2515869140625, y=345.660400390625, z=0.0}, {bed:[x=410.2515869140625, y=345.660400390625, z=0.0}, {bed:[x=410.2515869140625, y=345.660400390625, z=0.0}, {chair:[x=410.2515869140625, y=345.660400390625, z=0.0}, {bed:[x=410.2515869140625, y=345.660400390625, z=0.0}, {chair:[x=410.2515869140625, y=345.660400390625, z=0.0}, {backpack:[x=410.2515869140625, y=345.660400390625, z=0.0}, 

"""