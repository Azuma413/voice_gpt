from llava.utils import disable_torch_init
from llava.model.builder import load_pretrained_model
from llava.mm_utils import get_model_name_from_path
import rclpy
from rclpy.node import Node
from chatrover_msgs.srv import ImageText
import argparse
import torch
import requests
from PIL import Image
from io import BytesIO
from transformers import TextStreamer
from llava.constants import (
    IMAGE_TOKEN_INDEX,
    DEFAULT_IMAGE_TOKEN,
    DEFAULT_IM_START_TOKEN,
    DEFAULT_IM_END_TOKEN,
)
from llava.conversation import conv_templates, SeparatorStyle
from llava.mm_utils import (
    tokenizer_image_token,
    KeywordsStoppingCriteria,
)
import cv2
from cv_bridge import CvBridge

print("hello world")

prompt = "Describe the object in the image appropriately in two words or less."
model_path = "liuhaotian/llava-v1.5-13b"

# Model
model_name = get_model_name_from_path(model_path)
tokenizer, model, image_processor, context_len = load_pretrained_model(
    model_path, None, model_name, load_8bit=True
)

def generate_text(inp: str, image: Image = None):
    conv_mode = "llava_v1"
    conv = conv_templates[conv_mode].copy()
    roles = conv.roles
    image_tensor = None
    if image is not None:
        image_tensor = (
            image_processor.preprocess(image, return_tensors="pt")["pixel_values"]
            .half()
            .cuda()
        )
        # first message
        if model.config.mm_use_im_start_end:
            inp = (
                DEFAULT_IM_START_TOKEN
                + DEFAULT_IMAGE_TOKEN
                + DEFAULT_IM_END_TOKEN
                + "\n"
                + inp
            )
        else:
            inp = DEFAULT_IMAGE_TOKEN + "\n" + inp
        conv.append_message(conv.roles[0], inp)
    else:
        # later messages
        conv.append_message(conv.roles[0], inp)
    conv.append_message(conv.roles[1], None)
    prompt = conv.get_prompt()
    input_ids = (
        tokenizer_image_token(prompt, tokenizer, IMAGE_TOKEN_INDEX, return_tensors="pt")
        .unsqueeze(0)
        .cuda()
    )
    stop_str = conv.sep if conv.sep_style != SeparatorStyle.TWO else conv.sep2
    keywords = [stop_str]
    stopping_criteria = KeywordsStoppingCriteria(keywords, tokenizer, input_ids)
    streamer = TextStreamer(tokenizer, skip_prompt=True, skip_special_tokens=True)
    with torch.inference_mode():
        output_ids = model.generate(
            input_ids,
            images=image_tensor,
            do_sample=True,
            temperature=0.2,
            max_new_tokens=256,
            streamer=streamer,
            use_cache=True,
            stopping_criteria=[stopping_criteria],
        )
    outputs = tokenizer.decode(output_ids[0, input_ids.shape[1] :]).strip()
    conv.messages[-1][-1] = outputs
    # outputsはDock</s>となっているので、</s>を削除する
    return outputs[:-4]

# ROS2のサービスサーバーノード
class ImageTextServer(Node):

    def __init__(self):
        super().__init__('image_text_server')
        print("image_text_server is running.")
        self.srv = self.create_service(ImageText, 'image_text', self.image_text_callback)
        self.bridge = CvBridge()

    def image_text_callback(self, request, response):
        print("get request")
        image = self.load_image(request.image)
        response.text = generate_text(prompt, image)
        return response
    
    # ROSのsensor_msgs/ImageをPILのImageに変換する
    def load_image(self, ros_image):
        cv_image = self.bridge.imgmsg_to_cv2(ros_image, "rgb8")
        pil_image = Image.fromarray(cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB))
        return pil_image
    
def main():
    rclpy.init()
    server = ImageTextServer()
    rclpy.spin(server)
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()