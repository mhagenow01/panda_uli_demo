#!/usr/bin/env python

""" Record bag file using speech

 Last Updated: 09/07/2021
"""

__author__ = "Mike Hagenow"

import time
import speech_recognition as sr
import os
import rosbag
import rospy
from threading import Lock
import sys

from geometry_msgs.msg import PoseStamped, WrenchStamped
from std_msgs.msg import Int32

class BagRecorder():
    def __init__(self,prepend=""):

        self.prepend = prepend

        self.lock = Lock()

        # obtain audio from the microphone
        self.r = sr.Recognizer()
        self.r.pause_threshold = 0.05
        self.r.non_speaking_duration = 0.05
        self.m = sr.Microphone()
        # set threshold level
        with self.m as source:
            self.r.adjust_for_ambient_noise(source)
        # print("Set minimum energy threshold to {}".format(self.r.energy_threshold))

        # start listening in the background (note that we don't have to do this inside a `with` statement)
        stop_listening = self.r.listen_in_background(self.m, self.callback, phrase_time_limit=1.0)
        self.loop()

    def callback(self, recognizer, audio):
        # received audio data, now we'll recognize it using Google Speech Recognition
        try:
            # for testing purposes, we're just using the default API key
            # to use another API key, use `r.recognize_google(audio, key="GOOGLE_SPEECH_RECOGNITION_API_KEY")`
            # instead of `r.recognize_google(audio)`
            word = recognizer.recognize_google(audio)
            # print("Google Speech Recognition thinks you said " + recognizer.recognize_google(audio))
            print("recog: ",word)
            if "start" in word:
                os.system('play -nq -t alsa synth {} sine {}'.format(0.2, 660))
                self.recording = True
            elif "stop" in word:
                os.system('play -nq -t alsa synth {} sine {}'.format(0.2, 330))
                self.recording = False

        except sr.UnknownValueError:
            # print("Google Speech Recognition could not understand audio")
            pass
        except sr.RequestError as e:
            # print("Could not request results from Google Speech Recognition service; {0}".format(e))
            pass


    def rollerPoseCallback(self,data):
        if self.curr_recording:
            self.lock.acquire()
            self.activebag.write("/vrpn_client_node/roller/pose",data)
            self.lock.release()
    def sealantPoseCallback(self,data):
        if self.curr_recording:
            self.lock.acquire()
            self.activebag.write("/vrpn_client_node/sealant_gun/pose",data)
            self.lock.release()
    def ftminiCallback(self,data):
        if self.curr_recording:
            self.lock.acquire()
            self.activebag.write("/ftmini40",data)
            self.lock.release()
    def valveCallback(self,data):
        if self.curr_recording:
            self.lock.acquire()
            self.activebag.write("/valvestate",data)
            self.lock.release()


    def loop(self):
        self.recording = False
        self.curr_recording = False

        rospy.init_node('bagrecorder', anonymous=True)

        # Subscribe to all possible LFD Topics
        rospy.Subscriber("/vrpn_client_node/roller/pose", PoseStamped, self.rollerPoseCallback, queue_size=1)
        rospy.Subscriber("/vrpn_client_node/sealant_gun/pose", PoseStamped, self.sealantPoseCallback, queue_size=1)
        rospy.Subscriber("/ftmini40", WrenchStamped, self.ftminiCallback, queue_size=1)
        rospy.Subscriber("/valvestate", Int32, self.valveCallback, queue_size=1)

        time.sleep(1)

        os.system('play -nq -t alsa synth {} sine {}'.format(0.1, 880))
        os.system('play -nq -t alsa synth {} sine {}'.format(0.1, 880))
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self.recording and not self.curr_recording:
                demo_name = "demo_"+str(long(time.time()))+".bag"
                if self.prepend!="":
                    demo_name = self.prepend + "_" + demo_name
                self.lock.acquire()
                self.activebag = rosbag.Bag(demo_name,'w')
                self.lock.release()
                self.curr_recording = True

            elif not self.recording and self.curr_recording:
                self.curr_recording = False
                time.sleep(2)
                print("Demo: "+demo_name+" is recorded")
                self.lock.acquire()
                self.activebag.close()
                self.lock.release()
                

            rate.sleep()


if __name__ == "__main__":
    prepend = ""
    if len(sys.argv)>1:
        prepend = sys.argv[1]
    br = BagRecorder(prepend=prepend)
