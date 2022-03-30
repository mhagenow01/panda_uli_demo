#!/usr/bin/env python

""" Sets up topics for controlled valve
    - turning on and off computer control mode
    - sending 0 and 1 commands for the valve
    - receiving valve state during LFD

 Last Updated: 09/27/2021
"""

__author__ = "Mike Hagenow"

import numpy as np
import rospy
import sys
from std_msgs.msg import Int32, Bool
import serial
import signal
import time
from functools import partial


def signal_handler(valve, sig, frame):
    print('You pressed Ctrl+C!')
    valve.ser.write(bytearray('e', 'ascii'))
    sys.exit(0)

class ControlledValve():
    def __init__(self, computerControl=False):
        rospy.init_node('controlledvalve', anonymous=True)

        self.ser = serial.Serial('/dev/ttyACM0', 9600, timeout=0.5)
        time.sleep(2) # wait 2 seconds to allow port to open
        self.r = rospy.Rate(1000)  # 1000hz (Arduino runs at 100)
        self.curr_val = 0

        if not computerControl:
            self.state_pub = rospy.Publisher('valvestate', Int32, queue_size=1)
            self.int = Int32()
            self.runLoop()
        else: # computer control
            self.ser.write(bytearray('s', 'ascii'))
            rospy.Subscriber("valvestate", Int32, self.valve_callback)
            signal.signal(signal.SIGINT, partial(signal_handler, self))
            rospy.spin()


    def valve_callback(self,data):
        if data.data == 1 and self.curr_val !=1 :
            self.curr_val = 1
            self.ser.write(bytearray('1', 'ascii'))
            print("1 sent")
        elif data.data ==0 and self.curr_val !=0 :
            self.curr_val = 0
            self.ser.write(bytearray('0', 'ascii'))
            print("0 sent")

    def runLoop(self):
        while not rospy.is_shutdown():
            try:
                ser_bytes = self.ser.readline()  
                
                decoded_bytes = int(ser_bytes.decode("ascii"))
                if decoded_bytes==1 or decoded_bytes==0:
                    self.int.data = int(decoded_bytes)
                    self.state_pub.publish(self.int)
                
            except Exception as e:
                print(e)
                pass
            self.r.sleep()



if __name__ == "__main__":
    computerControl = sys.argv[1]=='True'
    valve = ControlledValve(computerControl)
