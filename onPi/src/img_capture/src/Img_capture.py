#!/usr/bin/env python

import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from std_msgs.msg import String


class Camera:
    def __init__(self, camera_id):
        self.camera_id = camera_id
        self.cap = cv2.VideoCapture(self.camera_id)
        print('Camera is connected')

        self.bridge = CvBridge()
        rospy.init_node('img_capture')
        self.pub = rospy.Publisher('/camera', Image, queue_size = 1)
        
        rate = rospy.Rate(10)

    def talker(self):
        while not rospy.is_shutdown():
            ret, frame = self.cap.read()
            if not ret or cv2.waitKey(1) & 0xFF == ord('q'): 
                break
            cv2.imshow('frame', frame)
            msg = self.bridge.cv2_to_imgmsg(frame, 'bgr8')
            self.pub.publish(msg)

            rospy.Subscriber('/endgame', String, self.sub_endgame)

        self.cap.release()
        cv2.destroyAllWindows()
    
    def sub_endgame(self, data):
        if data.data == 'stop':
            rospy.signal_shutdown('img_capture terminate')


if __name__ == '__main__':
    camera = Camera(2)
    try:
        camera.talker()
    except rospy.ROSInterruptException:
        pass