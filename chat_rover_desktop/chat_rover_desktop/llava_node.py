from llava.utils import disable_torch_init
from llava.model.builder import load_pretrained_model
from llava.mm_utils import get_model_name_from_path
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

model_path = "liuhaotian/llava-v1.5-13b"
# Model
model_name = get_model_name_from_path(model_path)
tokenizer, model, image_processor, context_len = load_pretrained_model(
    model_path, None, model_name, load_8bit=True
)

def load_image(image_file):
    if image_file.startswith("http") or image_file.startswith("https"):
        response = requests.get(image_file)
        image = Image.open(BytesIO(response.content)).convert("RGB")
    else:
        image = Image.open(image_file).convert("RGB")
    return image

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


prompt = "Describe the object in the image appropriately in two words or less."
image_file = "https://llava-vl.github.io/static/images/view.jpg"

image = load_image(image_file)
generate_text(prompt, image)