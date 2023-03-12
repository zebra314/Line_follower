#!/usr/bin/env python

import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image


class Camera:
    def __init__(self, camera_id):
        rospy.init_node('camera')
        self.camera_id = camera_id
        self.cap = cv2.VideoCapture(self.camera_id)

        if self.cap.isOpened():
            print('\nCamera connected.\n')
        else :
            print('\nCamera not connected.\n')

        self.bridge = CvBridge()
        
        self.pub = rospy.Publisher('/image', Image, queue_size = 1)
        
        rate = rospy.Rate(10)

    def talker(self):
        while not rospy.is_shutdown():
            ret, frame = self.cap.read()
            if not ret : 
                break
            msg = self.bridge.cv2_to_imgmsg(frame, 'bgr8')
            self.pub.publish(msg)

        self.cap.release()
        cv2.destroyAllWindows()
    
if __name__ == '__main__':
    camera = Camera(2)
    try:
        camera.talker()
    except rospy.ROSInterruptException:
        pass