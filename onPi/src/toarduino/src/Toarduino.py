#!/usr/bin/env python

import rospy
import serial
import signal
from time import sleep
from std_msgs.msg import String


class Toarduino:
    def __init__(self):
        rospy.init_node('Toarduino')

        # connect to arduino board
        self.ser = serial.Serial('/dev/ttyUSB0',9600)
        self.ser.timeout = 2.5
        print('\narduino connected\n')  
    
    def __call__(self):
        rospy.Subscriber('/offset', String, self.sub_offset, queue_size = 1)
        
        # terminate the node safetly
        signal.signal(signal.SIGINT, self.signal_handler)
        rospy.spin()

    def sub_offset(self, msg):
        # send the msg to arduino
        self.ser.write(bytes(str(msg.data), 'utf-8'))
        arduino_echo = ''
        while arduino_echo == '' :
            arduino_echo = self.ser.readline().decode('utf').strip()
        print(arduino_echo)
    
    def signal_handler(self):
        self.ser.close()
        print('\nToarduino stop\n')

if __name__ == '__main__':
    toarduino = Toarduino()
    toarduino()