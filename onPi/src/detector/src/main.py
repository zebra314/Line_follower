#!/usr/bin/env python

import rospy
import cv2
import signal
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from std_msgs.msg import String
from Line_detector import Line_detector
from Coachman import Coachman
from Recorder import Recorder
from Object_detector import Object_detector


class detector:
    def __init__(self):
        rospy.init_node('detector')
        self.pub = rospy.Publisher('/offset', String, queue_size = 1)
        self.bridge = CvBridge()
        self.line_detector = Line_detector(resolution = 16)
        self.coachman = Coachman()
        # self.object_detector = Object_detector()
        self.recorder= Recorder('convert')
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

        # Detect the line
        frame_lineDetected, points, contours = self.line_detector(frame_cv2.copy())

        # Controll the motor
        frame_hullDetected,  motorspeed = self.coachman(frame_lineDetected, points, contours)
        # cv2.imshow('frame_hullDetected', frame_hullDetected)

        # Record the frame
        self.recorder(frame_hullDetected)

        self.pub.publish(str(motorspeed))
        cv2.waitKey(1) & 0xFF

    def signal_handler(self):
        self.recorder.writer.release()
        self.pub.publish('0 0')
        cv2.destroyAllWindows()
        print('\nline_detector stop\n')
    
    def sub_endgame(self, data):
        """
        Execute when receiving the msg from endgame topic.
        Using the self.going to break the loop in __call__.
        """
        if data.data == 'stop':
            self.pub.publish('0 0')
            rospy.signal_shutdown('line_detector terminate')

if __name__ == "__main__":
    detector = detector()
    detector()