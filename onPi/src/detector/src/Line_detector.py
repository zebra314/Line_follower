#!/usr/bin/env python

import rospy
import cv2
import signal
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
        self.offset = 0

        rospy.init_node('line_detector')
        rate = rospy.Rate(10)
    
    def __call__(self):
        rospy.Subscriber('/camera', Image, self.sub_camera, queue_size = 1, buff_size = 52428800)
        rospy.Subscriber('/endgame', String, self.sub_endgame, queue_size = 1) 

        # terminate the node safetly
        signal.signal(signal.SIGINT, self.signal_handler) 
        rospy.spin()

    def sub_camera(self, frame):
        """
        Execute when receiving the msg from camera topic
        """
        frame = self.bridge.imgmsg_to_cv2(frame, 'bgr8') 
        frame = self.img_process(frame)
        frame = self.plot_position(frame)
        self.recorder(frame)

        # pub的函式不知道為什麼要放在這裡才會被執行到
        # 放在__call__ 裡不會執行
        self.pub.publish(str(self.offset))

        cv2.imshow('frame_line_detected', frame)
        cv2.waitKey(1)

    def sub_endgame(self, data):
        """
        Execute when receiving the msg from endgame topic.
        Using the self.going to break the loop in __call__.
        """
        self.going = data.data 
            
    def signal_handler(self):
        self.recorder.writer.release()
        cv2.destroyAllWindows()
        print('\nline_detector stop\n')

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

        # 兩位小數
        self.offset = round( (cX - (W/2))/(W/2)*100 , 2)

        return frame
        
if __name__ == "__main__":
    line_detector = Line_detector()
    line_detector()