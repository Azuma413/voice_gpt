from chatrover_msgs.msg import ObjectPosition
from sensor_msgs.msg import Image
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Point
from cv_bridge import CvBridge
import cv2
import torch
from super_gradients.training import models
from super_gradients.common.object_names import Models

#service /get_object

frame_size = {4096/3, 2160/3}

print("Is gpu available?: ",torch.cuda.is_available())

class ObjectRecognition(Node):

    def __init__(self, img_pub=True):
        self.img_pub = img_pub # YOLOで処理した後の画像をpublishするかどうか。
        super().__init__('object_recognition')
        self.pub_pos = self.create_publisher(ObjectPosition, '/object_pos', 1)
        self.pub_img = self.create_publisher(Image, '/detect_image', 1)
        self.create_subscription(Image, '/image_raw', self.listener_callback, 1)
        self.bridge = CvBridge() # ROSからOpenCVに画像形式を変換する用のブリッジinstanceを生成
        self.marker_img = None # 検出したマーカーを含む画像を格納する変数
        #---yolo用の処理---#
        device = torch.device("cuda:0") if torch.cuda.is_available() else torch.device("cpu")
        self.model = models.get(Models.YOLO_NAS_L, pretrained_weights="coco").to(device)

    def listener_callback(self, msg):
        """
        実際に物体を検出する関数
        """
        marker_pos = ObjectPosition() # 検出したマーカー位置を格納する変数
        name = String()
        input_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        results = self.model.predict(input_image) # color_imageをYOLOで処理
        cv_result = cv2.cvtColor(results[0].image, cv2.COLOR_RGB2BGR)
        name_list = results[0].class_names
        label_list = results[0].prediction.labels
        box_list = results[0].prediction.bboxes_xyxy
        for i in range(len(box_list)):
            name = name_list[int(label_list[i])]
            x = ((box_list[i][0] + box_list[i][2])/2 - frame_size[0]/2)/frame_size[0]
            # y = (box_list[i][1] + box_list[i][3])/2
            marker_pos.name.append(name)
            marker_pos.position.append(x)
            cv2.drawMarker(cv_result, (int((box_list[i][0] + box_list[i][2])/2), int((box_list[i][1] + box_list[i][3])/2)), (0, 0, 255), cv2.MARKER_SQUARE, 5, 5)
            cv2.putText(cv_result, name, (int(int((box_list[i][0] + box_list[i][2])/2)) - 20, int((box_list[i][1] + box_list[i][3])/2) + 30), cv2.FONT_HERSHEY_PLAIN, 2, (0, 0, 255), 2)
        # ROSの画像形式に変換
        self.marker_img = self.bridge.cv2_to_imgmsg(cv_result, "bgr8")
        self.pub_pos.publish(marker_pos)
        if self.img_pub:
            self.pub_img.publish(self.marker_img)

def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = ObjectRecognition()
    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()