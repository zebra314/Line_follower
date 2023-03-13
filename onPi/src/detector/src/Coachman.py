#!/usr/bin/env python

import math
import numpy as np
import cv2 
import queue

class Coachman:
    def __init__(self):

        self.ctrl = 0
        self.ctrl_last = 0
        self.pid = PID([1000, 16, 200])
        self.normal_speed = 60
    
    def __call__(self, frame, path, poly):
        """
        :return: frame and motorspeed
        """
        self.IMG_HEIGHT, self.IMG_WIDTH = frame.shape[:2]

        if len(path) == 0:
            # print('No line detected.')
            self.ctrl = 0
            return frame, self.ctrl

        # Check if the line detected correctly
        frame_convexHull, Clear = self.convexhull_check(frame.copy(), path) 

        if not Clear:
            self.ctrl = self.ctrl_last
            # print('Noise detected.')
        else :
            self.ctrl = self.speed_get(path)
            self.ctrl_last = self.ctrl
            # print('Line Detected.')
        
        reviser = min(int(self.ctrl), 30) if self.ctrl else max(int(self.ctrl),-30)
        motorspeed =  str(self.normal_speed - reviser) + ' ' + str(self.normal_speed + reviser)
        return frame_convexHull, motorspeed
        

    def evaluate_function(self, angle_part, translate_part, x, y):
        x = abs(x)
        return math.exp(-0*y/self.IMG_HEIGHT/20.)*(
        (1 - 1.*x/self.IMG_WIDTH/2.0)*angle_part +
        1.*x/self.IMG_WIDTH/2.0 * translate_part)
    
    def convexhull_check(self, frame, path):
        """
        :return: frame
        """
        hull = cv2.convexHull(np.array(path)) 
        area = cv2.contourArea(hull)
        AREA = self.IMG_HEIGHT * self.IMG_WIDTH
        cv2.drawContours(frame, [hull], 0, (0, 255, 0), 1)

        # print(area/AREA)
        if (area/AREA) > 0.2:
            return frame, False
        else :
            return frame, True

    def speed_get(self, path):
        n = len(path)
        curve_cm = [0, 0]
        for i in range(0, len(path)):
            curve_cm[0] += 1.*path[i][0]/len(path)
            curve_cm[1] += 1.*path[i][1]/len(path)

        vec_sec = [path[0][0]-path[n-1][0], path[0][1]-path[n-1][1]]
        ang_sec = 90 - np.angle(vec_sec[0] - vec_sec[1] * 1J, deg=True)

        translate_part = 90 - np.interp(curve_cm[0], [0, self.IMG_WIDTH], [180, 0])
        angle_part = ang_sec

        ctrl_estimate = self.evaluate_function(angle_part, translate_part, \
                        curve_cm[0] - self.IMG_WIDTH/2.0, self.IMG_HEIGHT - curve_cm[1])

        self.pid.step(ctrl_estimate)
        ctrl = self.pid.get_ctrl()
        return ctrl

class PID:
    def __init__(self, KPID, QUEUE_SIZE = 2000):
        self.PID = list([0., 0., 0.])
        self.KPID = list([1.*KPID[0], 1.*KPID[1]/QUEUE_SIZE, 1.*KPID[2]])
        self.integral_ghb = queue.Queue(QUEUE_SIZE)
        self.last_tmp = 0
        self.ctrl = 0
        self.queue_size = QUEUE_SIZE

    def step(self, cur_data):
        self.PID[0] = cur_data
        self.PID[2] = cur_data - self.last_tmp
        self.last_tmp = cur_data
        if not self.integral_ghb.full():
            self.integral_ghb.put(cur_data)
            self.PID[1] += cur_data
        else:
            self.PID[1] = self.PID[1] - self.integral_ghb.get() + cur_data
            self.integral_ghb.put(cur_data)
        self.ctrl =  sum(self.PID[i] * self.KPID[i] for i in range(3))/sum(self.KPID)
    
    def get_ctrl(self):
        return self.ctrl