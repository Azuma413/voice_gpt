import cv2
import sys
import cv2.aruco as aruco
import numpy as np
import pyrealsense2 as rs
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

COMMON_PIXEL = (1280, 720)
SAMPLING_TIME = 30

class ARNode(Node):

    def __init__(self):
        super().__init__('ar_node')
        self.robot_state_pub = self.create_publisher(Twist, "/robot_state", 1)
        timer_period = 0.01 #s
        self.timer = self.create_timer(timer_period, self.detect_robot)
        self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
        self.cameraMatrix = np.zeros((3, 3))
        self.distCoeffs = np.zeros(5)
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

    def detect_robot(self):
        pub_data = Twist()
        frames = self.rspipe.wait_for_frames()
        # color_frameとdepth_frameを取得
        color_frame = frames.get_color_frame()
        # color_frameをOpenCV形式に変換
        color_image = np.asanyarray(color_frame.get_data())

        cv2.imshow("test", color_image)
        parameters = aruco.DetectorParameters_create()
        corners, ids, _ = aruco.detectMarkers(color_image, self.aruco_dict, parameters=parameters)
        color_image = aruco.drawDetectedMarkers(color_image, corners, ids)
        rvecs, tvecs, _objPoints = aruco.estimatePoseSingleMarkers(corners, 0.05, self.cameraMatrix, self.distCoeffs)
        if ids is not None:
            for i in range( ids.size ):
                aruco.drawAxis(color_image, self.cameraMatrix, self.distCoeffs, rvecs[i], tvecs[i], 0.1)
        pub_data.linear.x = 0.0
        pub_data.linear.y = 0.0
        pub_data.angular.z = 0.0
        self.robot_state_pub.publish(pub_data)
        cv2.imshow('frame', color_image)

def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = ARNode()
    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()
    cv2.destroyAllWindows()
    rclpy.shutdown()

if __name__ == '__main__':
    main()