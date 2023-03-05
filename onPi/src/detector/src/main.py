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
        # self.recorder= Recorder('convert')
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
        points, contours = self.line_detector(frame_cv2.copy())

        # Show debug image 
        for i in contours:
            cv2.drawContours(frame_cv2, i, -1, (0,0,255), 3)
        for i in points:
            frame_cv2 = cv2.circle(frame_cv2, i, 6, (0,0,255), -1)
        # (vx, vy) : vector
        # (x, y) : point on the line 
        vx, vy, x, y = cv2.fitLine(np.int32(points), cv2.DIST_L2, 0, 0.01, 0.01)
        cv2.line(frame, (int(x+100*vx),int(y+100*vy)), (int(x),int(y)), (0, 255, 255), 3)
        frame_plot = self.line_detector.debug_img(frame_cv2, points, contours)
        cv2.imshow('frame_plot', frame_plot)

        # Record the frame
        # self.recorder(frame_convert)

        # self.pub.publish(str(motorspeed))
        cv2.waitKey(1) & 0xFF

    def signal_handler(self):
        # self.recorder.writer.release()
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