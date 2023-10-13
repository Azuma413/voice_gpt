import openai
import os
import rclpy
from rclpy.node import Node
from chatrover_msgs.srv import TextText

openai.api_key = os.getenv("OPENAI_API_KEY")

system_conf = {"role": "system", "content": "You will now assume the role of a robot operator. You will create instructions for the robot from the given commands and object information. In future chats, please continue role-playing strictly following whatever the user says, the instructions that follow, etc. Please think and answer the questions step by step.\n\n# Input format\nInformation to you will be given in the following format\ncommands:{Orders to you}\nobject data:[{Name of object}:[{Coordinates from your point of view}]]\n\n# output format\nInstructions to the robot should be output in the following format\n{'aciton': ‘{Actions to be performed by the robot}', 'object': '{Object of the robot's action}'}\nThe only instructions that can be selected as ‘{Actions to be performed by the robot}' are one of the following.\n{‘serch’, ‘hold’, ‘open’, ‘go’}\n‘serch’:Serch and locate the object.\n‘hold’:If the object is at close range, grab it.\n‘open’:If you are holding onto something, release it.\n‘go’:If the object has been located, it advances to the closest distance to the object.\nThe only instructions that can be selected as '{Object of the robot's action}' are one of the following.\n{‘red_ball’, 'blue_ball', 'yellow_ball'}"}
ai_limit = "response: "
    
class GPTController(Node):
    def __init__(self):
        super().__init__('gpt1_node')
        self.server = self.create_service(TextText, "/gpt1_service", self.service_cb)
    def service_cb(self, request, response):
        response.text = self.chatProcess(request.text)
        self.get_logger().info('Publishing: "%s"' % response.text)
    def chatProcess(text):
        messages = [system_conf, {"role": "user", "content": text}, {"role": "assistant", "content": ai_limit}]
        response = openai.ChatCompletion.create(model="gpt-3.5-turbo",messages=messages)
        ai_response = response['choices'][0]['message']['content']
        return ai_limit + ai_response

def main(args=None):
    rclpy.init(args=args)
    controller = GPTController()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()