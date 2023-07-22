#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
from json.encoder import INFINITY
from sensor_msgs.msg import Image, JointState
from geometry_msgs.msg import Point
import cv2
from cv_bridge import CvBridge
import numpy as np
from numpy import sin, cos, tan
import pyrealsense2 as rs
import torch

common_pixel = (1280, 720)  # (width ,height) color_frameとdepth_frameで共通の画像サイズとする。
sampling_time = 30 # realsenseの画像を取得する周期(ms)
#weights_path = "weights/default/yolov5m.pt" # YOLOのデフォルトモデル
weights_path = "weights/default/best.pt" # ドローン用にトレーニングしたモデル。時々読み込みエラーがでる。
pub_img = True # YOLOで処理した後の画像をpublishするかどうか。

class ObjectsRecognition:
    """
    pyrealsense2から取得した画像をYOLOで処理して最大の検出目標のスクリーン座標を取得する。
    """
    def __init__(self):
        self.bridge = CvBridge() # ROSからOpenCVに画像形式を変換する用のブリッジinstanceを生成
        self.marker_pos = [0.0, 0,0, 0,0] # 検出したマーカー位置を格納する変数
        self.marker_img = None # 検出したマーカーを含む画像を格納する変数

        #---realsense用の処理---#
        conf = rs.config()
        conf.enable_stream(rs.stream.depth, common_pixel[0], common_pixel[1], rs.format.z16, sampling_time)
        conf.enable_stream(rs.stream.color, common_pixel[0], common_pixel[1], rs.format.rgb8, sampling_time)
        self.pipe = rs.pipeline()
        self.pipe.start(conf)
        # 深度frame用のfilter設定。もしかしたら要らないかもしれない。
        self.decimate = rs.decimation_filter()
        self.decimate.set_option(rs.option.filter_magnitude, 1)
        self.spatial = rs.spatial_filter()
        self.spatial.set_option(rs.option.filter_magnitude, 1)
        self.spatial.set_option(rs.option.filter_smooth_alpha, 0.25)
        self.spatial.set_option(rs.option.filter_smooth_delta, 50)
        self.hole_filling = rs.hole_filling_filter()
        self.depth_to_disparity = rs.disparity_transform(True)
        self.disparity_to_depth = rs.disparity_transform(False)

        #---yolo用の処理---#
        self.model = torch.hub.load("ultralytics/yolov5", "custom", path=weights_path, force_reload=True)

    def recognize_objects(self):
        """
        実際に物体を検出する関数
        """
        #---realsenseとYOLO用の処理---#
        frames = self.pipe.wait_for_frames()
        color_frame = frames.get_color_frame()
        depth_frame = frames.get_depth_frame()
        color_image = np.asanyarray(color_frame.get_data())
        results = self.model(color_image) # color_imageをYOLOで処理
        cv_result = cv2.cvtColor(results.render()[0], cv2.COLOR_RGB2BGR)
        # depth_frameをfilterに掛ける
        filter_frame = self.decimate.process(depth_frame)
        filter_frame = self.depth_to_disparity.process(filter_frame)
        filter_frame = self.spatial.process(filter_frame)
        filter_frame = self.disparity_to_depth.process(filter_frame)
        filter_frame = self.hole_filling.process(filter_frame)
        result_frame = filter_frame.as_depth_frame()

        rects = np.array(results.xyxy[0]) # 検出されたバウンディングボックスをすべて取得
        if len(rects) > 0: # マーカーが少なくとも1つ検出されている場合の処理
            rect = max(rects, key=(lambda x: (x[0]-x[2]) * (x[1]-x[3]))) # マーカーのうち, 輪郭の面積最大のものを取り出す.
            self.marker_pos[0] = (rect[0] + rect[2])/2 # マーカーの中心の座標を保存
            self.marker_pos[1] = (rect[1] + rect[3])/2
            self.marker_pos[2] = result_frame.get_distance(int((rect[0] + rect[2])/2),int((rect[1] + rect[3])/2)) # 座標の深度情報(m)を取得。極座標ではなくデカルト座標の深度が得られる。

        # OpenCVからROSの画像形式に変換
        self.marker_img = self.bridge.cv2_to_imgmsg(cv_result, "bgr8")

class ConvertSC2WC:
    """
    realsenseから得たpixelの座標と深度情報をデカルト座標に直す。
    こちらのサイトを参考にして実装した。https://dev.classmethod.jp/articles/convert-coords-screen-to-space/
    画像上での座標と深度情報(u, v, depth)からローバー視点での直交座標(x, y, z)を得る。
    カメラの位置t, カメラの回転行列R, カメラの内部パラメータ行列K
    """
    def __init__(self):
        self.cam_coord = [0.0, 0.0, 0.4] # カメラの座標(m)。カメラが大体40cmくらいの高さに取り付けてあるので0.4としている。
        self.cam_tilt = 0.0 # カメラの縦方向の角度(rad)。真上が0。
        fov = 86 # カメラの水平方向の視野角
        self.cam_info = (fov, common_pixel[0], common_pixel[1]) # カメラ情報（内部パラメータ）
        self.world_coord = Point() # 最終的な座標を保存する変数

    def loop(self, data):
        """
        実際に回す関数。スクリーン座標[u, v, depth]を引数とする。
        """
        t = self.cam_coord # カメラの位置
        R = self.calc_R(self.cam_tilt) # カメラの回転行列
        K = self.calc_K(*self.cam_info) # カメラの内部パラメータ行列
        self.convert_uvz_to_xyz(data, R, t, K) # 座標変換

    def update_cam_rot_cb(self, data):
        """
        dynamixelのjoint_stateからカメラの縦方向の回転情報を更新するROS用コールバック関数
        """
        self.cam_tilt = data.position[1]

    def calc_R(self, tilt_rad):
        """
        カメラの回転行列を求める。
        引数はカメラの縦回転(rad)
        """
        a = tilt_rad
        R = np.asarray([
            [1, 0, 0],
            [0, cos(a), -sin(a)],
            [0, sin(a), cos(a)],
            ])
        return R

    def calc_K(self, fov_x, pixel_w, pixel_h, cx=None, cy=None):
        """
        カメラの内部パラメータ行列を求める。
        引数は水平方向の視野角，横の画素数，縦の画素数
        """
        if cx is None:
            cx = pixel_w / 2.0
        if cy is None:
            cy = pixel_h / 2.0
        fx = 1.0 / (2.0 * tan(np.radians(fov_x) / 2.0)) * pixel_w
        fy = fx
        K = np.asarray([
            [fx, 0, cx],
            [0, fy, cy],
            [0, 0, 1],
        ])
        return K

    def convert_uvz_to_xyz(self, screen_coord, R, t, K):
        """
        座標変換する関数。
        引数はスクリーン座標(u, v, depth), カメラの回転行列R, カメラの位置t, カメラの内部パラメータ行列K
        """
        u = screen_coord[0]
        v = screen_coord[1]
        z = screen_coord[2]
        K_inv = np.linalg.inv(K) # 内部パラメータ行列の逆行列を求める
        cs = np.asarray([u, v, 1]) # スクリーン座標
        cs_ = cs * z
        cc = np.dot(K_inv, cs_) # カメラ座標
        cw = np.dot(R, cc) + t # 直交座標
        self.world_coord.x = cw[0]
        self.world_coord.y = cw[1]
        self.world_coord.z = cw[2]
        print("右:{:.2f} 前:{:.2f} 上:{:.2f}".format(*cw))

if __name__=='__main__':
    # ObjectsRecognition node の生成
    rospy.init_node('ColorRecognition', anonymous=True)
    rospy.loginfo('ColorRecognition Initialized.')
    # ObjectsRecognitionのインスタンスを生成
    ojrc = ObjectsRecognition()
    sc2wc = ConvertSC2WC()
    pub_marker_pos = rospy.Publisher("detected_marker_pos", Point, queue_size=1) # 検出したマーカー位置のpublisher
    pub_marker_img = rospy.Publisher("detected_marker_img", Image, queue_size=1) # 検出したマーカーを含んだ画像のpublisher
    rospy.Subscriber('/dynamixel_workbench/joint_states', JointState, sc2wc.update_cam_rot_cb, queue_size=1) # 現在のpantilt_posを他のノードから受信するためのSubscriberを設定
    r = rospy.Rate(10) # whileループの大体の周期を決定

    while not rospy.is_shutdown():
        ojrc.recognize_objects() # 物体を検出
        sc2wc.loop(ojrc.marker_pos) # 座標を変換
        pub_marker_pos.publish(sc2wc.world_coord) # 変換した座標をpublish
        if pub_img:
            pub_marker_img.publish(ojrc.marker_img) # 検出マーカーと含む画像をpublish
        r.sleep()

    ojrc.pipe.stop() # realslenseの処理を止める
