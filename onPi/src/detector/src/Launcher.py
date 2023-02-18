#!/usr/bin/env python

import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from std_msgs.msg import String
from Object_detection import ObjectDetection
from Recorder import Recorder

class Launcher:
    def __init__(self):
        self.objectDetection = ObjectDetection()
        self.bridge = CvBridge()       
        self.pub = rospy.Publisher('/start', String, queue_size = 1)
        self.recorder = Recorder('launcher')

        rospy.init_node('launcher')
        rospy.Subscriber('/camera', Image, self.callback, queue_size = 1, buff_size = 52428800)        

        rospy.spin()
 
    def callback(self, frame):
        frame = self.bridge.imgmsg_to_cv2(frame, 'bgr8')
        status, frame_launcher = self.objectDetection(frame)
        self.pub.publish(status)
        self.recorder(frame)

        cv2.imshow('frame_launcher', frame_launcher)
        cv2.waitKey(1)  

        if status == 'start':
            rospy.signal_shutdown('launcher terminate')
            cv2.destroyAllWindows()
            self.recorder.writer.release()

if __name__ == "__main__":
    termintor = Launcher()