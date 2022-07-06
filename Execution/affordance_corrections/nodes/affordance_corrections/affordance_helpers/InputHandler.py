#!/usr/bin/env python3
# File name: InputHandler.py
# Description: Takes types of input and unifies the topics
# Author: Mike Hagenow
# Date: 6/17/2021

import rospy
import numpy as np
import time

from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Float64, Int8

class InputHandler:
    ''' Class runs as a separate thread to properly manipulate input
        from a variety of devices and communicate to the ros_affordance_wrapper'''
    def __init__(self, input_method="phone", pubtime=0.02):
        self.updatepub = rospy.Publisher('correction',Twist,queue_size=1)
        self.flippub = rospy.Publisher('pc_flip',String,queue_size=1)
        self.articpub = rospy.Publisher('articulation',Float64,queue_size=1)

        self.twist = Twist()

        # used to downsample input
        self.pubtime = pubtime
        self.lastPubTime = time.time()

        # timer to prevent buttons double clicking
        self.pubtimebutton = 0.25
        self.lastPubTimeButton = time.time()

        if (input_method=="spacemouse"):
            ''' Mode 0 is using the Space Mouse '''
            rospy.Subscriber("/spacenav/joy", Joy, self.spaceMouseParse, False)
        elif (input_method=="spacemouseNL"):
            ''' Mode 1 is using the Space Mouse with Nonlinear Mapping '''
            rospy.Subscriber("/spacenav/joy", Joy, self.spaceMouseParse,True)
        elif (input_method=="teleop"):
            ''' Mode 2 is the teleop interface within RVIZ '''
            rospy.Subscriber("/rviz/teleop", Twist, self.teleopRvizParse)
            rospy.Subscriber("/rviz/artic", Int8, self.teleopRvizArtic)
        elif (input_method=="phone"):
            ''' Mode 3 is the gamepad on the phone '''
            rospy.Subscriber("/joy", Joy, self.GamePadParse)
        else:
            ''' default back to teleop'''
            rospy.Subscriber("/rviz/teleop", Twist, self.teleopRvizParse)
    
    def mapping(self,x,low=0.005,high=.25):
        a = (np.log(high)-np.log(low))/0.9
        b = np.exp(np.log(low)-.1*a)
        return np.sign(x)*b*((np.exp(a*np.abs(x))-1))
        

    def GamePadParse(self, msg):
        ax = np.array(msg.axes)

        self.twist.linear.x = self.mapping(ax[1])
        self.twist.linear.y = self.mapping(ax[0])
        self.twist.angular.x = self.mapping(-ax[2],low=.01,high=1)
        self.twist.angular.y = self.mapping(ax[3],low=.01,high=1)
        if msg.buttons[4]:
            self.twist.angular.z = -np.pi/4
        elif msg.buttons[5]:
            self.twist.angular.z = np.pi/4
        else:
            self.twist.angular.z = 0
        if msg.buttons[6]:
            self.twist.linear.z = .01
        elif msg.buttons[7]:
            self.twist.linear.z = -.01
        else:
            self.twist.linear.z = .0
        currTime = time.time()
        if currTime > (self.lastPubTime+self.pubtime):
            self.updatepub.publish(self.twist)
            self.lastPubTime = currTime

    def spaceMouseParse(self,data,nonlinear):
        ''' Takes Joy topic from spacemouse and republishes as a Twist '''
        # Scaling factors for translation and rotation of spacemouse input (empirically determined)
        k_rot = 3.0
        k_trans = 0.005
        articulation_rate = 0.05

        # Two differenct scalings on input: nonlinear and linear velocities
        if nonlinear:
            low_rate = 0.02
            high_rate = 1.6
            a = (np.log(high_rate)-np.log(low_rate))/0.9
            b = np.exp(np.log(low_rate)-0.1*a)
            vals = np.sign(np.array(data.axes))*b*(np.exp(a*np.abs(np.array(data.axes))))
        else:
            vals = np.array(data.axes)
        if(np.max(np.abs(data.axes))>0.1):
            vals[np.abs(data.axes)<0.1]=0.0 # saturate small inputs
            self.twist.linear.x = k_trans * vals[0]
            self.twist.linear.y = k_trans * vals[1]
            self.twist.linear.z = k_trans * vals[2]
            self.twist.angular.x = k_rot * vals[3]
            self.twist.angular.y = k_rot * vals[4]
            self.twist.angular.z = k_rot * vals[5]
            currTime = time.time()
            if currTime > (self.lastPubTime+self.pubtime):
                self.updatepub.publish(self.twist)
                self.lastPubTime = currTime
        elif(data.buttons[1]==1):
            temp_ang = Float64()
            temp_ang.data = articulation_rate
            currTime = time.time()
            if currTime > (self.lastPubTime+self.pubtime):
                self.articpub.publish(temp_ang)
                self.lastPubTime = currTime
        elif(data.buttons[0]==1):
            temp_ang = Float64()
            temp_ang.data = -articulation_rate
            currTime = time.time()
            if currTime > (self.lastPubTime+self.pubtime):
                self.articpub.publish(temp_ang)
                self.lastPubTime = currTime

    def teleopRvizParse(self,data):
        ''' Takes Joy topic from the teleop widget panel and republishes as a Twist '''
        # Scaling factors for translation and rotation of spacemouse input
        k_rot = 3.0
        k_trans = 0.005
        vals = np.array([data.linear.x, data.linear.y, data.linear.z, data.angular.x, data.angular.y, data.angular.z])
        if(np.max(np.abs(vals))>0.2):
            vals[np.abs(vals)<0.2]=0.0 # saturate small inputs
            self.twist.linear.x = k_trans * vals[0]
            self.twist.linear.y = k_trans * vals[1]
            self.twist.linear.z = k_trans * vals[2]
            self.twist.angular.x = k_rot * vals[3]
            self.twist.angular.y = k_rot * vals[4]
            self.twist.angular.z = k_rot * vals[5]
            currTime = time.time()
            if currTime > (self.lastPubTime+self.pubtime):
                self.updatepub.publish(self.twist)
                self.lastPubTime = currTime
    def teleopRvizArtic(self,data):
        ''' separate message for whether a button was pressed to cycle the articulation'''
        articulation_rate = 0.05
        if(data.data==1): # 1 for positive, -1 for negative rotation
            temp_ang = Float64()
            temp_ang.data = articulation_rate
            currTime = time.time()
            if currTime > (self.lastPubTime+self.pubtime):
                self.articpub.publish(temp_ang)
                self.lastPubTime = currTime
        else:
            temp_ang = Float64()
            temp_ang.data = -articulation_rate
            currTime = time.time()
            if currTime > (self.lastPubTime+self.pubtime):
                self.articpub.publish(temp_ang)
                self.lastPubTime = currTime

def inputLaunch(input_method):
    ''' kick off method creates class and spins for rostopic'''
    input = InputHandler(input_method)
    rospy.spin()
