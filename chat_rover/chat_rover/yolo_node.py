from chatrover_msgs.srv import TextText
import rclpy
from rclpy.node import Node
import cv2
import torch #pip3 install torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cpu
from super_gradients.training import models
from super_gradients.common.object_names import Models
import pyrealsense2 as rs
import numpy as np

COMMON_PIXEL = (1280, 720)
SAMPLING_TIME = 30

print("Is gpu available?: ",torch.cuda.is_available())

class ObjectRecognition(Node):

    def __init__(self):
        super().__init__('yolo_node')
        self.server = self.create_service(TextText, "/get_object", self.get_object_cb)

        # -------------------------- Realsense用の処理 -------------------------- #
        conf = rs.config()
        conf.enable_stream(
            rs.stream.color,
            COMMON_PIXEL[0],
            COMMON_PIXEL[1],
            rs.format.rgb8,
            SAMPLING_TIME,
        )
        self.rspipe = rs.pipeline()
        self.rspipe.start(conf)

        #---yolo用の処理---#
        device = torch.device("cuda:0") if torch.cuda.is_available() else torch.device("cpu")
        self.model = models.get(Models.YOLO_NAS_L, pretrained_weights="coco").to(device)

    def get_object_cb(self, request, response):
        """
        実際に物体を検出する関数
        """
        frames = self.rspipe.wait_for_frames()
        # color_frameとdepth_frameを取得
        color_frame = frames.get_color_frame()
        # color_frameをOpenCV形式に変換
        color_image = np.asanyarray(color_frame.get_data())
        results = self.model.predict(color_image)
        cv_result = cv2.cvtColor(results[0].image, cv2.COLOR_RGB2BGR)
        name_list = results[0].class_names

        print(name_list)

        label_list = results[0].prediction.labels
        box_list = results[0].prediction.bboxes_xyxy
        response.text = "{"
        for i in range(len(box_list)):
            name = name_list[int(label_list[i])]
            x = (box_list[i][0] + box_list[i][2])/2
            y = (box_list[i][1] + box_list[i][3])/2
            response.text += name + ":(" + str(x) + ", " + str(y) + "), "
            cv2.drawMarker(cv_result, int(x), int(y), (0, 0, 255), cv2.MARKER_SQUARE, 5, 5)
            cv2.putText(cv_result, name, int(x/2) - 20, int(y/2) + 30, cv2.FONT_HERSHEY_PLAIN, 2, (0, 0, 255), 2)
        cv2.imshow("detect object", cv_result)

def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = ObjectRecognition()
    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()
    cv2.destroyAllWindows()
    rclpy.shutdown()

if __name__ == '__main__':
    main()