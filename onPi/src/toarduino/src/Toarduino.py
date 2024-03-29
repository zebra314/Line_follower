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
        self.ser = serial.Serial('/dev/ttyUSB0',57600, timeout=0.05)
        sleep(2)
        print('\n' + self.ser.name + ' connected.\n')  
    
    def __call__(self):
        rospy.Subscriber('/offset', String, self.sub_offset, queue_size = 1)

        try:
            # terminate the node safetly
            signal.signal(signal.SIGINT, self.signal_handler)
            rospy.spin()
        except KeyboardInterrupt:
            self.ser.write(bytes(str('0')+':', 'utf-8'))
            self.ser.write(bytes(str('0')+'!', 'utf-8'))
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
        echo_left = self.ser.readline().decode('utf', errors='ignore').strip()
        echo_right = self.ser.readline().decode('utf', errors='ignore').strip()

        print('\necho_left: ' + echo_left)
        print('echo2_right: ' + echo_right + '\n')
    
    def signal_handler(self):
        self.ser.close()
        print('\nToarduino stop\n')

if __name__ == '__main__':
    toarduino = Toarduino()
    toarduino()

