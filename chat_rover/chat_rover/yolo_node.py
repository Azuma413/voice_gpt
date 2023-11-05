from chatrover_msgs.srv import TextText
import rclpy
from rclpy.node import Node
import cv2
import torch #pip3 install torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cpu
from super_gradients.training import models
from super_gradients.common.object_names import Models
import threading
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

COMMON_PIXEL = (1280, 720)
SAMPLING_TIME = 30

print("Is gpu available?: ",torch.cuda.is_available())

class ObjectRecognition(Node):

    def __init__(self):
        super().__init__('yolo_node')
        self.server = self.create_service(TextText, "/get_object", self.get_object_cb)
        self.create_subscription(Image, "/camera/camera/color/image_raw", self.image_cb, 1)
       # self.create_subscription(Image, "/image_raw", self.image_cb, 1)
        self.bridge = CvBridge()
        #---yolo用の処理---#
        device = torch.device("cuda:0") if torch.cuda.is_available() else torch.device("cpu")
        self.model = models.get(Models.YOLO_NAS_L, pretrained_weights="coco").to(device)
        
        # ---画像表示用のスレッド--- #
        self.display_thread = threading.Thread(target=self.display_window)
        self.display_thread.daemon = True  # デーモンスレッドに設定
        self.image = None
        self.color_image = None
        
    def image_cb(self, msg):
        self.color_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

    def get_object_cb(self, request, response):
        """
        実際に物体を検出する関数
        """
        results = self.model.predict(self.color_image)
        #cv_result = cv2.cvtColor(results[0].image, cv2.COLOR_RGB2BGR)
        cv_result = results[0].image
        name_list = results[0].class_names
        label_list = results[0].prediction.labels
        box_list = results[0].prediction.bboxes_xyxy
        response.text = "{"
        for i in range(len(box_list)):
            name = name_list[int(label_list[i])]
            x = int((box_list[i][0] + box_list[i][2])/2)
            y = int((box_list[i][1] + box_list[i][3])/2)
            response.text += name + ":(" + str(x) + ", " + str(y) + "), "
            cv2.drawMarker(cv_result, (x, y), (0, 0, 255), cv2.MARKER_SQUARE, 5, 5)
            cv2.putText(cv_result, name, (x - 20, y + 30), cv2.FONT_HERSHEY_PLAIN, 2, (255, 0, 0), 2)
        response.text += "}"
        self.image = cv_result
        if not self.display_thread.is_alive():
            self.display_thread.start()
        return response
    
    def display_window(self):
        while True:
            cv2.imshow("detect object", self.image)
            key = cv2.waitKey(1)
            if key == ord('q'):
                break
        cv2.destroyAllWindows()

def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = ObjectRecognition()
    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()