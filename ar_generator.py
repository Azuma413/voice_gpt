#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import cv2
aruco = cv2.aruco
dictionary = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)

def arGenerator():
    fileName = "ar.png"
    # 0: ID番号，720x720ピクセル
    generator = aruco.generateImageMarker(dictionary, 0, 720)
    cv2.imwrite(fileName, generator)
    img = cv2.imread(fileName)

arGenerator()
