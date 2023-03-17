#!/usr/bin/env python

import rospy
import serial
import signal
from std_msgs.msg import String
from time import sleep


class Toarduino:
    def __init__(self):
        rospy.init_node('Toarduino')

        # connect to arduino board
        self.ser = serial.Serial('/dev/ttyACM0',57600, timeout=0.05)
        sleep(2)
        print('\n' + self.ser.name + ' connected.\n')  
    
    def __call__(self):
        rospy.Subscriber('/offset', String, self.sub_offset, queue_size = 1)

        try:
            # terminate the node safetly
            signal.signal(signal.SIGINT, self.signal_handler)
            rospy.spin()
        except KeyboardInterrupt:
            self.signal_handler

    def sub_offset(self, msg):
        # send the msg to arduino

        speed = msg.data.split(' ')
        motor_left = speed[0]
        motor_right = speed[1]
        self.ser.flushOutput()
        self.ser.flushInput()

        self.ser.write(bytes(str(motor_left)+':', 'utf-8'))
        self.ser.write(bytes(str(motor_right)+'!', 'utf-8'))
        echo1 = self.ser.readline().decode('utf', errors='ignore').strip()
        echo2 = self.ser.readline().decode('utf', errors='ignore').strip()
        print('echo1: ' + echo1)
        print('echo2: ' + echo2)
    
    def signal_handler(self):
        self.ser.close()
        print('\nToarduino stop\n')

if __name__ == '__main__':
    toarduino = Toarduino()
    toarduino()

