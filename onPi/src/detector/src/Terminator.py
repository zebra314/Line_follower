#!/usr/bin/env python

import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from std_msgs.msg import String
from Object_detection import ObjectDetection
from Recorder import Recorder

class Terminator:
    """
    This class is just created for testing shutdown the node by an event.
    """
    def __init__(self):
        self.objectDetection = ObjectDetection()
        self.bridge = CvBridge()       
        self.pub = rospy.Publisher('/endgame', String, queue_size = 1)

        rospy.init_node('terminator')
        rospy.Subscriber('/camera', Image, self.callback, queue_size = 1, buff_size = 52428800)        

        rospy.spin()
 
    def callback(self, frame):
        frame = self.bridge.imgmsg_to_cv2(frame, 'bgr8')
        status, frame_terminator = self.objectDetection(frame)
        self.pub.publish(status)

        cv2.imshow('frame_terminator', frame_terminator)
        cv2.waitKey(1)  

        if status == 'stop':
            rospy.signal_shutdown('terminator terminate')
            cv2.destroyAllWindows()

if __name__ == "__main__":
    termintor = Terminator()