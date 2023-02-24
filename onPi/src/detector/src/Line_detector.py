#!/usr/bin/env python

import rospy
import cv2
import signal
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from std_msgs.msg import String
from Recorder import Recorder


    
class Line_detector:
    """
    Class to detect the line and calculate the percentage of the offset.
    """
    def __init__(self):
        self.bridge = CvBridge()
        self.recorder = Recorder('line_detector')
        self.pub = rospy.Publisher('/offset', String, queue_size = 1)
        self.offset_1 = 0
        self.offset_2 = 0
        self.offset_3 = 0

        rospy.init_node('line_detector')
        rate = rospy.Rate(10)
    
    def __call__(self):
        rospy.Subscriber('/camera', Image, self.sub_camera, queue_size = 1, buff_size = 52428800)
        rospy.Subscriber('/endgame', String, self.sub_endgame, queue_size = 1) 

        # terminate the node safetly
        signal.signal(signal.SIGINT, self.signal_handler) 
        rospy.spin()

    def sub_camera(self, ori_frame):
        """
        Execute when receiving the msg from camera topic
        
        :ori_frame: 
            1. ROS communication format.
            2. The original frame capture from the camera.
        """
        frame = self.trans_format(ori_frame)
        frame_processed = self.img_process(frame)

        # 之後可能會須調用 contours 資料, 先把 find 和 plot 分開寫
        contours = self.find_contours(frame_processed)
        frame = self.plot_contours(frame, contours)

        # 之後可能會需要調用 moments 資料, 先把 find 和 plot 分開寫
        moments = self.find_moments(frame_processed)
        frame = self.plot_moments(frame, moments)

        # Record thr frame
        self.recorder(frame)

        # pub的函式不知道為什麼要放在這裡才會被執行到
        # 放在__call__ 裡不會執行
        self.pub.publish(str(self.offset_1))

        cv2.imshow('frame_line_detected', frame)
        cv2.waitKey(1)

    def sub_endgame(self, data):
        """
        Execute when receiving the msg from endgame topic.
        Using the self.going to break the loop in __call__.
        """
        if data.data == 'stop':
            rospy.signal_shutdown('line_detector terminate')
 
    def signal_handler(self):
        self.recorder.writer.release()
        cv2.destroyAllWindows()
        print('\nline_detector stop\n')
    
    def trans_format(self, frame):
        # 格式轉換
        frame = self.bridge.imgmsg_to_cv2(frame, 'bgr8')
        return frame

    def img_process(self, frame):
        # BGR　to GRAY
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
        # 模糊處理
        frame = cv2.GaussianBlur(frame, (7, 7), 0) 
        
        # 大津二值化
        _, threshold_image = cv2.threshold(frame, 0, 255, cv2.THRESH_BINARY+cv2.THRESH_OTSU) 
        
        # 腐蝕, 膨脹
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (7, 7))
        frame = cv2.erode(threshold_image, kernel, iterations=2)
        frame = cv2.dilate(frame, kernel)

        # 反轉 
        # frame = cv2.bitwise_not(frame)

        # Canny
        lower = 100 # 150
        upper = 300 # 200
        frame = cv2.Canny(frame, lower, upper, apertureSize = 3)

        return frame
        

    def find_contours(self, frame):
        contours = []
        contours, _ = cv2.findContours(frame, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        return contours

    def plot_contours(self, frame, contours):
        cv2.drawContours(frame, contours, -1, (0, 0, 255), 3)
        return frame
    
    def find_moments(self, frame):
        """
        Find the line using the moment of the area
        # in three seperate section
        # 0~1/6
        # 1/6~1/3
        # 1/3~2/3
        """
        H, W = frame.shape

        # crop
        frame_top1 = frame[0:int(H/6), 0:W]
        frame_top2 = frame[int(H/6):int(H/3), 0:W]
        frame_mid = frame[int(H/3):int(2*H/3), 0:W]
        
        # Moments
        moments = []
        M_top1 = cv2.moments(frame_top1)
        M_top2 = cv2.moments(frame_top2)
        M_mid = cv2.moments(frame_mid)

        moments.append(M_top1)
        moments.append(M_top2)
        moments.append(M_mid)

        return moments

    def plot_moments(self, frame, moments):
        """
        Plot the line using the moment of the area
        # in three seperate section
        # 0~1/6
        # 1/6~1/3
        # 1/3~2/3
        """

        H, W, _ = frame.shape
        
        M_top1 = moments[0]
        M_top2 = moments[1]
        M_mid = moments[2]

        # frame line
        # frame = cv2.line(frame,(int(W/2),0),(int(W/2),int(H)),(55,255,255),1)
        # frame = cv2.line(frame,(0,int(H/6)),(W,int(H/6)),(55,255,255),1)
        # frame = cv2.line(frame,(0,int(H/3)),(W,int(H/3)),(55,255,255),1)
        # frame = cv2.line(frame,(0,int(2*H/3)),(W,int(2*H/3)),(55,255,255),1)

        yellow = (55,255,255)

        # moment calculate using top section of the frame
        cX_top1 = 0
        cY_top1 = 0
        if M_top1["m00"] != 0:
            cX_top1 = int(M_top1["m10"] / M_top1["m00"])
            cY_top1 = int(M_top1["m01"] / M_top1["m00"])
        frame = cv2.circle(frame, (int(cX_top1), int(cY_top1)), 6, yellow, -1)

        # moment calculate using middle of the frame
        cX_top2 = 0
        cY_top2 = 0
        if M_top2["m00"] != 0:
            cX_top2 = int(M_top2["m10"] / M_top2["m00"])
            cY_top2 = int(M_top2["m01"] / M_top2["m00"])+ int(H/6)
        frame = cv2.circle(frame, (int(cX_top2), int(cY_top2)), 6, yellow, -1)

        # conect to point top and mid
        frame = cv2.line(frame,(int(cX_top1), int(cY_top1)),(int(cX_top2), int(cY_top2)),yellow,2)

        # moment calculate using down frame
        cX_mid = 0
        cY_mid = 0
        if M_mid["m00"] != 0:
            cX_mid = int(M_mid["m10"] / M_mid["m00"])
            cY_mid = int(M_mid["m01"] / M_mid["m00"])+ int(H/3)
        frame = cv2.circle(frame, (int(cX_mid), int(cY_mid)), 5, yellow, -1)

        # connect point mid and center 
        frame = cv2.line(frame, (int(cX_mid), int(cY_mid)), (int(cX_top2), int(cY_top2)),yellow,2)

        return frame

    def plot_position(self, frame):
        # # 兩位小數
        # self.offset = round( (cX_mid - (W/2))/(W/2)*100 , 2)
        
        """
        To detect the lines
        HoughLines settings
        """
        # Test1 straight lines
        # rho = 1 # 距離精度
        # theta = np.pi/180 # 角度精度
        # # threshod = 30 # 超過閥值才會被檢測為線段
        # # minLineLength = 100 # 線段以像素為單位的最小長度
        # # maxLineGap = 200 # 一條線段中允許的最大斷裂
        # lines = cv2.HoughLinesP(frame, rho, theta, 10, minLineLength = 3, maxLineGap = 50)
        # frame = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
        # if lines is not None:
        #     for x1, y1, x2, y2 in lines[0]:
        #         cv2.line(frame, (x1, y1), (x2, y2), (0, 0, 255), 2)

        # Test2 curve lines
        # minLineLength = 30
        # maxLineGap = 5
        # lines = cv2.HoughLinesP(frame,cv2.HOUGH_PROBABILISTIC, np.pi/180, 30, minLineLength,maxLineGap)
        # for x in range(0, len(lines)):
        #     for x1,y1,x2,y2 in lines[x]:
        #         #cv2.line(inputImage,(x1,y1),(x2,y2),(0,128,0),2, cv2.LINE_AA)
        #         pts = np.array([[x1, y1 ], [x2 , y2]], np.int32)
        #         cv2.polylines(frame, [pts], True, (0,255,0))

        return frame
        
if __name__ == "__main__":
    line_detector = Line_detector()
    line_detector()