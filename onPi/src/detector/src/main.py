#!/usr/bin/env python

import rospy
import cv2
import signal
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from std_msgs.msg import String
from Line_detector import Line_detector
from Recorder import Recorder
from Object_detector import Object_detector


class detector:
    def __init__(self):
        rospy.init_node('detector')
        self.pub = rospy.Publisher('/offset', String, queue_size = 1)
        self.bridge = CvBridge()
        # self.recorder1 = Recorder('camera')
        # self.recorder2 = Recorder('convert')
        self.line_detector = Line_detector()
        # self.object_detector = Object_detector()

        self.offset = 0
        
        rate = rospy.Rate(10)
    
    def __call__(self):
        rospy.Subscriber('/image', Image, self.sub_camera, queue_size = 1, buff_size = 52428800)

        # Terminate the node safetly
        signal.signal(signal.SIGINT, self.signal_handler) 
        rospy.spin()

    def sub_camera(self, frame):
        """
        Execute when receiving the msg from camera topic
        :frame: In ROS communication format. It's the original frame captured from the camera.
        """

        # Transform the format of the frame
        frame_cv2 = self.bridge.imgmsg_to_cv2(frame, 'bgr8')
        frame_convert = self.line_detector(frame_cv2.copy())

        # Record the frame
        # self.recorder1(frame_cv2)
        # self.recorder2(frame_convert)

        # Do not use imshow if your operating system is Ubuntu Server
        # cv2.imshow('camera', frame_cv2)
        # cv2.imshow('convert', frame_convert)

        # motorspeed = self.line_detector.PID()
        # self.pub.publish(str(motorspeed))
        cv2.waitKey(1) & 0xFF

    def signal_handler(self):
        # self.recorder1.writer.release()
        # self.recorder2.writer.release()

        cv2.destroyAllWindows()
        print('\nline_detector stop\n')
    
    def sub_endgame(self, data):
        """
        Execute when receiving the msg from endgame topic.
        Using the self.going to break the loop in __call__.
        """
        if data.data == 'stop':
            rospy.signal_shutdown('line_detector terminate')

if __name__ == "__main__":
    detector = detector()
    detector()