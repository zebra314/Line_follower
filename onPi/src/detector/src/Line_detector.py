#!/usr/bin/env python

import cv2
import numpy as np


class Line_detector:
    """
    Detect the line in the frame.
    Input a single frame and output points and contours in list format.
    """
    def __init__(self, resolution = 16):
        self.slice_num = resolution
        pass
    
    def __call__(self, frame):
        frame_processed = self.img_process(frame, self.slice_num)
        return frame_processed

    def img_process(self, frame, slice_num):  
        """
        :frame: BGR format
        """
        IMG_HEIGHT, IMG_WIDTH = frame.shape[:2]
        X_DIV = int(IMG_HEIGHT/float(slice_num))
        poly_points = [None] * slice_num
        detected_contours = [None] * slice_num
    
        # Blur
        frame_blur = cv2.GaussianBlur(frame,(7,7),0)

        # Threshold
        frame_threshold = self.threshold_otsu(frame_blur)

        # Morphological transformation
        frame_morpho = self.MorphoTrans(frame_threshold)
        
        # Find contours in each slice
        for i in range(0, slice_num) :
            frame_sliced = frame_morpho[X_DIV*i + 1:X_DIV*(i+1), int(0):int(IMG_WIDTH)]
            conts,_ = cv2.findContours(frame_sliced.copy(),cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_NONE)
            if(conts):
                c = max(conts, key = cv2.contourArea)
                M = cv2.moments(c)
                if M['m00'] != 0:
                    poly_points[i] = (int(M['m10']/M['m00']), int(M['m01']/M['m00']) + X_DIV * i)
                else :
                   poly_points[i] = (0, 0)
                detected_contours[i] = c + (0, X_DIV * i)

        contours = [i for i in detected_contours if i is not None]
        points = [i for i in poly_points if i is not None]
        return points, contours

    def threshold_custom(self, frame):
        """
        :frame: BGR foramt
        """
        LB=np.array([0,0,0], np.uint8)
        UB=np.array([180,255,75], np.uint8)

        imgHSV = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(imgHSV, LB, UB)
        return mask

    def threshold_otsu(self, frame):
        """
        :frame: BGR foramt
        """
        frame_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        _, threshold_image = cv2.threshold(frame_gray, 0, 255, cv2.THRESH_BINARY+cv2.THRESH_OTSU)
        threshold_image = cv2.bitwise_not(threshold_image)
        return threshold_image
    
    def MorphoTrans(self, frame):
        """
        :frame: Threshold format
        """
        kernelOpen = np.ones((5,5))
        kernelClose = np.ones((20,20))
        maskOpen = cv2.morphologyEx(frame,cv2.MORPH_OPEN,kernelOpen)
        maskClose = cv2.morphologyEx(maskOpen,cv2.MORPH_CLOSE,kernelClose)
        return maskClose
    
    def debug_img(self, frame, points, contours):
        for i in contours:
            cv2.drawContours(frame, i, -1, (0,0,255), 3)
        for i in points:
            frame = cv2.circle(frame, i, 6, (0,0,255), -1)

        # (vx, vy) : vector
        # (x, y) : point on the line 
        vx, vy, x, y = cv2.fitLine(np.int32(points), cv2.DIST_L2, 0, 0.01, 0.01)
        cv2.line(frame, (int(x+100*vx),int(y+100*vy)), (int(x),int(y)), (0, 255, 255), 3)

        return frame