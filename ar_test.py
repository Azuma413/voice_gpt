import cv2
from cv2 import aruco #pip install opencv-contrib-python==4.7.0.72
import numpy as np

print("opencv version: ", cv2.__version__)

dictionary = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
parameters = aruco.DetectorParameters()
detector = aruco.ArucoDetector(dictionary, parameters)
cameraMatrix = np.zeros((3, 3))
distCoeffs = np.zeros(5)
color_image = cv2.imread("ar2.png")
cv2.imshow("read", color_image)
corners, ids, _ = detector.detectMarkers(color_image)
corners = np.array(corners[0])
ids = ids[0]
print("corners:", corners)
print("ids:", ids)
if ids is not None:
    for i in range( ids.size ):
        start_point = np.array([np.mean(corners[i,:,0]), np.mean(corners[i,:,1])])
        end_point = start_point + (corners[i,0]+corners[i,1]-corners[i,2]-corners[i,3])/2
        cv2.arrowedLine(color_image, (int(start_point[0]), int(start_point[1])), (int(end_point[0]), int(end_point[1])), (0,0,255), 5)
        cv2.putText(color_image, str(ids[i]), (int(start_point[0]), int(start_point[1])), cv2.FONT_HERSHEY_PLAIN, 2, (255, 0, 0), 2)
cv2.imshow("test", color_image)
print((np.pi+np.arctan2(end_point[0]-start_point[0], end_point[1]-start_point[1]))/np.pi*180.0)
cv2.waitKey(0)
cv2.destroyAllWindows()  # ウィンドウを閉じる