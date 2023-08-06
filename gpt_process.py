import openai
import os

openai.api_key = os.getenv("OPENAI_API_KEY")

system_conf = {"role": "system", "content": "You will now assume the role of a robot operator. You will create instructions for the robot from the given commands and object information. In future chats, please continue role-playing strictly following whatever the user says, the instructions that follow, etc. Please think and answer the questions step by step.\n\n# Input format\nInformation to you will be given in the following format\ncommands:{Orders to you}\nobject data:[{Name of object}:[{Coordinates from your point of view}]]\n\n# output format\nInstructions to the robot should be output in the following format\n{'aciton': ‘{Actions to be performed by the robot}', 'object': '{Object of the robot's action}'}\nThe only instructions that can be selected as ‘{Actions to be performed by the robot}' are one of the following.\n{‘serch’, ‘hold’, ‘open’, ‘go’}\n‘serch’:Serch and locate the object.\n‘hold’:If the object is at close range, grab it.\n‘open’:If you are holding onto something, release it.\n‘go’:If the object has been located, it advances to the closest distance to the object.\nThe only instructions that can be selected as '{Object of the robot's action}' are one of the following.\n{‘red_ball’, 'blue_ball', 'yellow_ball'}"}
class ChatGPTClass:
    def __init__(self):
        self.messages = []
    def chatProcess(self, userMessage):
        if len(self.messages)>10:
            del self.messages[0:2]
        # ユーザの回答を保存
        self.messages += [system_conf, {"role": "user", "content": userMessage}, {"role": "assistant", "content": "{'action': '"}]

        # APIを使ってChatGPTの回答を生成
        response = openai.ChatCompletion.create(model="gpt-3.5-turbo",messages=self.messages)
        ai_response = response['choices'][0]['message']['content']
        # AIの回答を保存
        self.messages[-1]["content"] = ai_response
        del self.messages[-3]
        #print(self.messages, flush=True)
        return ai_response
    
gpt = ChatGPTClass()
result = gpt.chatProcess("commands:赤いボールを取ってきてください\nobject data:{person:[x=410.2515869140625, y=345.660400390625, z=0.0}, {bed:[x=410.2515869140625, y=345.660400390625, z=0.0}, {bed:[x=410.2515869140625, y=345.660400390625, z=0.0}, {chair:[x=410.2515869140625, y=345.660400390625, z=0.0}, {red_ball:[x=410.2515869140625, y=345.660400390625, z=0.0}, {chair:[x=410.2515869140625, y=345.660400390625, z=0.0}, {backpack:[x=410.2515869140625, y=345.660400390625, z=0.0}, ")
print(result)





"""
sample
commands:おはようございます
object data:{person:[x=559.481201171875, y=411.8475036621094, z=0.0}, {bed:[x=559.481201171875, y=411.8475036621094, z=0.0}, {bed:[x=559.481201171875, y=411.8475036621094, z=0.0}, {bed:[x=559.481201171875, y=411.8475036621094, z=0.0}, {backpack:[x=559.481201171875, y=411.8475036621094, z=0.0}, {handbag:[x=559.481201171875, y=411.8475036621094, z=0.0}, 
commands:今日もいい天気ですね
object data:{person:[x=582.73583984375, y=412.4528503417969, z=0.0}, {bed:[x=582.73583984375, y=412.4528503417969, z=0.0}, {bed:[x=582.73583984375, y=412.4528503417969, z=0.0}, {bed:[x=582.73583984375, y=412.4528503417969, z=0.0}, 
commands:赤いボールを取ってきてください
object data:{person:[x=410.2515869140625, y=345.660400390625, z=0.0}, {bed:[x=410.2515869140625, y=345.660400390625, z=0.0}, {bed:[x=410.2515869140625, y=345.660400390625, z=0.0}, {chair:[x=410.2515869140625, y=345.660400390625, z=0.0}, {bed:[x=410.2515869140625, y=345.660400390625, z=0.0}, {chair:[x=410.2515869140625, y=345.660400390625, z=0.0}, {backpack:[x=410.2515869140625, y=345.660400390625, z=0.0}, 

"""

