#!/usr/bin/env python

import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from std_msgs.msg import String
from Recorder import Recorder

class Line_detector:
    def __init__(self):
        # self.recorder = Recorder('line_detector')
        rospy.init_node('line_detector')
        self.bridge = CvBridge()
        rospy.Subscriber('/camera', Image, self.sub_camera, queue_size = 1, buff_size = 52428800)
        rospy.Subscriber('/endgame', String, self.sub_endgame)
        rospy.spin()

    def sub_camera(self, frame):
        frame = self.bridge.imgmsg_to_cv2(frame, 'bgr8')
        frame = self.img_process(frame)
        frame_line_detected = self.plot_position(frame)
        # self.recorder(frame)
        cv2.imshow('frame_line_detected', frame_line_detected)
        cv2.waitKey(1)

    def sub_endgame(self, data):
        if data.data == 'stop':
            rospy.signal_shutdown('line_detector terminate')

    def img_process(self, frame):
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # 模糊處理 
        frame = cv2.GaussianBlur(frame, (7, 7), 0)

        # 大津二值化
        _, threshold_image = cv2.threshold(frame, 0, 255, cv2.THRESH_BINARY+cv2.THRESH_OTSU)

        # 腐蝕, 膨脹
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (7, 7))
        frame = cv2.erode(threshold_image, kernel, iterations=2)
        frame = cv2.dilate(frame, kernel)
    
        return frame
    
    def plot_position(self, frame):
        # 反轉 
        frame_inv = cv2.bitwise_not(frame)
        M = cv2.moments(frame_inv)

        frame = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
        H, W, _ = frame.shape
        frame = cv2.line(frame,(int(W/2),0),(int(W/2),int(H)),(55,255,255),2)

        if M["m00"] != 0:
            cX = int(M["m10"] / M["m00"])
            cY = int(M["m01"] / M["m00"])
    
        frame = cv2.circle(frame, (int(cX), int(H/2)), 5, (0, 0, 255), -1)
        frame = cv2.line(frame,(int(cX),0),(int(cX),int(H*2)),(0, 0, 255),2)

        # 要傳給PID function的 data
        # offset = (cX - (W/2))/(W/2)

        return frame


if __name__ == "__main__":
    line_detector = Line_detector()
