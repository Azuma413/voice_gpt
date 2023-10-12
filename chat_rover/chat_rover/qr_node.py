#topic /robot_state

import cv2
import pyzbar.pyzbar
from pyzbar.pyzbar import decode
import pyrealsense2 as rs
import numpy as np
#pip install pyzbar
#sudo apt install libzbar0


mode = 1 # 1:Zbar 2:OpenCV
limit_rate = 1/6


COMMON_PIXEL = (1280, 720)
# Realsenseの画像を取得する周期(ms)
SAMPLING_TIME = 30
conf = rs.config()
conf.enable_stream(
    rs.stream.depth,
    COMMON_PIXEL[0],
    COMMON_PIXEL[1],
    rs.format.z16,
    SAMPLING_TIME,
)
conf.enable_stream(
    rs.stream.color,
    COMMON_PIXEL[0],
    COMMON_PIXEL[1],
    rs.format.rgb8,
    SAMPLING_TIME,
)
# Realsenseのパイプラインinstanceを生成
rspipe = rs.pipeline()
# Realsenseのパイプラインを開始
rspipe.start(conf)
delay = 1


if mode == 1:
    window_name = 'Pyzbar'
elif mode == 2:
    window_name = 'OpenCV'
    qcd = cv2.QRCodeDetector()


while True:
    raw_frames = rspipe.wait_for_frames()
    # color_frameとdepth_frameを取得
    color_frame = raw_frames.get_color_frame()
    depth_frame = raw_frames.get_depth_frame()
    # color_frameをOpenCV形式に変換
    frame = np.asanyarray(color_frame.get_data())
    frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
    x_len = len(frame[0])
    y_len = len(frame)
    limit_frame = frame[int(y_len*(1-limit_rate)/2):int(y_len*(1+limit_rate)/2),int(x_len*(1-limit_rate)/2):int(x_len*(1+limit_rate)/2)]
    if mode == 1:
        for d in decode(limit_frame, symbols=[pyzbar.pyzbar.ZBarSymbol.QRCODE]):
            s = d.data.decode()
            print(s)
            limit_frame = cv2.rectangle(limit_frame, (d.rect.left, d.rect.top), (d.rect.left + d.rect.width, d.rect.top + d.rect.height), (0, 255, 0), 3)
            limit_frame = cv2.putText(limit_frame, s, (d.rect.left, d.rect.top + d.rect.height), cv2.FONT_HERSHEY_SIMPLEX, 2, (0, 0, 255), 2, cv2.LINE_AA)
            limit_frame = cv2.putText(limit_frame, str(depth_frame.get_distance(int(x_len*(1-limit_rate)/2 + d.rect.left + d.rect.width/2), int(y_len*(1-limit_rate)/2 + d.rect.top + d.rect.height/2))*100)+"cm", (d.rect.left, d.rect.top), cv2.FONT_HERSHEY_SIMPLEX, 2, (0, 0, 255), 2, cv2.LINE_AA)
    elif mode == 2:
        ret_qr, decoded_info, points, _ = qcd.detectAndDecodeMulti(limit_frame)
        if ret_qr:
            for s, p in zip(decoded_info, points):
                if s:
                    print(s)
                    color = (0, 255, 0)
    else:
        color = (0, 0, 255)
        limit_frame = cv2.polylines(limit_frame, [p.astype(int)], True, color, 8)
        cv2.imshow(window_name, limit_frame)

    if cv2.waitKey(delay) & 0xFF == ord('q'):
        break


cv2.destroyWindow(window_name)