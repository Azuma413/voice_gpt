import cv2
from cv2 import aruco
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

COMMON_PIXEL = (1280, 720)
SAMPLING_TIME = 30

print("opencv version: ", cv2.__version__)

class ARNode(Node):

    def __init__(self):
        super().__init__('ar_node')
        self.robot_state_pub = self.create_publisher(Twist, "/robot_state", 1)
        self.detect_image_pub = self.create_publisher(Image, "/detect_image", 1)
        self.create_subscription(Image, "/camera/camera/color/image_raw", self.image_cb, 1)
        timer_period = 0.01 #s
        self.timer = self.create_timer(timer_period, self.detect_robot)
        self.bridge = CvBridge()
        self.color_image = None
        # ------aruco setup--------#
        dictionary = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
        parameters = aruco.DetectorParameters()
        self.detector = aruco.ArucoDetector(dictionary, parameters)
        self.cameraMatrix = np.zeros((3, 3))
        self.distCoeffs = np.zeros(5)
        
    def image_cb(self, msg):
        self.color_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

    def detect_robot(self):
        pub_data = Twist()
        if not self.color_image is None:
            color_image = self.color_image
            corners, ids, _ = self.detector.detectMarkers(color_image)
            if ids is not None:
                corners = np.array(corners[0])
                ids = ids[0]
                for i in range( ids.size ):
                    start_point = np.array([np.mean(corners[i,:,0]), np.mean(corners[i,:,1])])
                    end_point = start_point + (corners[i,0]+corners[i,1]-corners[i,2]-corners[i,3])/2
                    cv2.arrowedLine(color_image, (int(start_point[0]), int(start_point[1])), (int(end_point[0]), int(end_point[1])), (0,0,255), 5)
                    cv2.putText(color_image, str(ids[i]), (int(start_point[0]), int(start_point[1])), cv2.FONT_HERSHEY_PLAIN, 2, (255, 0, 0), 2)
                    if ids[i] == 0:
                        pub_data.linear.x = float(start_point[0])
                        pub_data.linear.y = float(start_point[1])
                        pub_data.angular.z = np.pi+np.arctan2(end_point[0]-start_point[0], end_point[1]-start_point[1])
                        self.robot_state_pub.publish(pub_data)
            #color_image = cv2.cvtColor(color_image, cv2.COLOR_RGB2BGR)
            cv_result = self.bridge.cv2_to_imgmsg(color_image, "bgr8")
            self.detect_image_pub.publish(cv_result)

def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = ARNode()
    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()