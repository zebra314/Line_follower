#!/usr/bin/env python

import cv2
import numpy as np


class Line_detector:
    """
    Class to detect the line and calculate the percentage of the offset.
    """
    def __init__(self):

        # PID setting
        self.P = 0
        self.I = 0
        self.D = 0
        self.Kp = 0.07
        self.Ki = 0.0008
        self.Kd = 0.6
        self.ideal = 0
        self.lastError = 0

        # motor speed
        self.motorSpeed = 30

    def __call__(self, frame):
        frame_process = self.img_process(frame)

        contours = self.find_contours(frame_process)
        moments = self.find_moments(frame_process)

        frame = self.plot_contours(frame, contours)
        frame = self.plot_moments(frame, moments)

        return frame

    def img_process(self, frame):
        # BGR　to GRAY
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
        # Blur
        frame = cv2.GaussianBlur(frame, (7, 7), 0) 
        
        # Otsu's Threshold
        _, threshold_image = cv2.threshold(frame, 0, 255, cv2.THRESH_BINARY+cv2.THRESH_OTSU) 
        
        # morphological image processing
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (7, 7))
        frame = cv2.erode(threshold_image, kernel, iterations=2)
        frame = cv2.dilate(frame, kernel)

        # inverse
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

        # calculate the current position
        self.ideal = W/2
        self.position = cX_mid

        # connect point mid and center 
        frame = cv2.line(frame, (int(cX_mid), int(cY_mid)), (int(cX_top2), int(cY_top2)),yellow,2)

        return frame

    def PID(self):

        # right offset error > 0
        # left offeset error > 0 
        error = (self.ideal - self.position) 

        self.P = error
        # self.I = self.I + error
        self.D = error - self.lastError
        self.lastError = error

        # calculate the correction
        motorspeed = self.P * self.Kp + self.D * self.Kd # + self.I * self.Ki
        
        leftspeed = int(self.motorSpeed - motorspeed)
        rightspeed = int(self.motorSpeed + motorspeed)

        if leftspeed > 60:
            leftspeed = 60
        elif leftspeed < 0:
            leftspeed = 0
        
        if rightspeed > 60:
            rightspeed = 60
        elif rightspeed < 0:
            rightspeed = 0

        stringspeed = str(leftspeed) + ' ' + str(rightspeed)
        return stringspeed
    
    def Hough(self, frame):
        """
        To detect straight and curve lines using HoughLine algorithm 
        """
        # Detect straight lines
        rho = 1 # distance accuracy
        theta = np.pi/180 # angle accuracy
        # threshod = 30 # 超過閥值才會被檢測為線段
        # minLineLength = 100 # 線段以像素為單位的最小長度
        # maxLineGap = 200 # 一條線段中允許的最大斷裂
        lines = cv2.HoughLinesP(frame, rho, theta, 10, minLineLength = 3, maxLineGap = 50)
        frame = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
        if lines is not None:
            for x1, y1, x2, y2 in lines[0]:
                cv2.line(frame, (x1, y1), (x2, y2), (0, 0, 255), 2)

        # Detect curve lines
        minLineLength = 30
        maxLineGap = 5
        lines = cv2.HoughLinesP(frame,cv2.HOUGH_PROBABILISTIC, np.pi/180, 30, minLineLength,maxLineGap)
        for x in range(0, len(lines)):
            for x1,y1,x2,y2 in lines[x]:
                #cv2.line(inputImage,(x1,y1),(x2,y2),(0,128,0),2, cv2.LINE_AA)
                pts = np.array([[x1, y1 ], [x2 , y2]], np.int32)
                cv2.polylines(frame, [pts], True, (0,255,0))