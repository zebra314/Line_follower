#!/usr/bin/env python

import datetime
import os
import cv2
import rospy
import cv2
import signal
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image


class Recorder:
    """
    Class to record the video and store to the video folder
    """
    
    def __init__(self, name):
        """
        Setup the video format and where to store
        """
        fourcc = cv2.VideoWriter_fourcc(*'mp4v')
        self.bridge = CvBridge()
        self.get_file_name(name)
        self.check_folder()
        self.writer = cv2.VideoWriter(self.path + self.file_name + '.mp4', fourcc, 30.0, (640,480))
    
    def __call__(self, frame):
        """
        :frame: ROS communication format
        """
        frame_cv2 = self.bridge.imgmsg_to_cv2(frame, 'bgr8')
        self.writer.write(frame_cv2)
    
    def get_file_name(self, name):
        """
        Generate the file name using the date and current time
        """
        now = datetime.datetime.now()
        self.file_name = str(now.year) + "-" + str(now.month) + "-" + str(now.day) + " " + str(now.hour) + ":" + str(now.minute) + ":" +str(now.second) + '-' + str(name)
        
    def check_folder(self):
        """        
        Detect if the video folder exist
        if not, create one
        """
        name = os.getlogin()
        self.path = '/home/' + name + '/Line_follower/asset/videos/'
        
        isExist = os.path.exists(self.path) 
        if not isExist:
            os.mkdir(self.path)
        print('Video save in' + self.path)
    
    def terminate(self):
        self.writer.release()

def signal_handler():
    for name, obj in globals.copy.items():
        if isinstance(obj, Recorder):
            eval(name + '.terminate()')
    print('\nrecorder stop\n') 

def main():
    rospy.init_node('recorder')
    recorder_pure = Recorder('pure')
    recorder_hullDetected = Recorder('hullDetected')

    rospy.Subscriber('/image', Image, recorder_pure, queue_size = 1, buff_size = 52428800)
    rospy.Subscriber('/hullDetected', Image, recorder_hullDetected, queue_size = 1, buff_size = 52428800)
    
    # Terminate the node safetly
    signal.signal(signal.SIGINT, signal_handler)
    rospy.spin()

if __name__ == '__main__':
    main()

