#!/usr/bin/env python

"""
Using chin movement to determine that user is still talking
https://github.com/hansonrobotics/hr_reflexes/issues/6

Conditions:
  - [ ] Only trigger if there is exactly one face
  - [ ] Moving Window over /realsense face.mouth_open over threshold
  - [ ] If Sophia is speaking (/sophia6/speech_events listen to start/stop)
  - [ ] STT event was under threshold of time away (last event on /sophia6/perceived_text)

Trigger:
  - [ ] Shut up (str data "shutup" on /sophia6/tts_control)
  - [ ] Pop from stack of >10 apologies and say them (str data ".." on /sophia6/tts)
  - [ ] Backtrack in Dialog (DOC: not possible in current release, Wenwei must update Chatscript server for this to work. Will work via :retry or :redo command.)
  - [ ] WORKAROUND: Interrupt Sophia's speech and repeat last utterance. 
"""

import rospy
import numpy as np
from time import sleep
import time, threading

from realsense_ros.msg import *
from pau2motors.msg import pau
from std_msgs.msg import UInt8, String

class realsense_stream:
    def __init__(self):
        self.userSpokeThreshold = 2.0
        self.avgMouthOpenThreshold = 0.2
        self.avgMouthOpenWindowSize = 5
        self.avgMouthOpenStack = [0 for _ in range(self.avgMouthOpenWindowSize)]

        self.activeFaces = 0
        self.avgMouthOpen = 0.0
        self.sophiaSpeaking = False
        self.userSpokeRecently = False

        rospy.init_node('interrupt_reflex')

        # Subscribers
        rospy.Subscriber('/realsense', Vision, self.parseRealsense)
        rospy.Subscriber('/sophia6/speech_events', String, self.parseIsSpeaking)
        rospy.Subscriber('/sophia6/perceived_text', String, self.parseLastSpeechInput)

        # Publishers
        self.publishShutup = rospy.Publisher('/sophia6/tts_control', String, queue_size=1)
        self.publishSpeechEvent = rospy.Publisher('/sophia6/tts', String, queue_size=1)

        # Debug output
        self.debugOutput()



    def debugOutput(self):
      print("activeFaces: %d, mouthOpen: %f, sophia speaking: %d, user spoke recently: %d" % (self.activeFaces, self.avgMouthOpen, self.sophiaSpeaking, self.userSpokeRecently))

      if self.activeFaces == 0 and self.avgMouthOpen >= self.avgMouthOpenThreshold and self.sophiaSpeaking and self.userSpokeRecently:
        print("TRIGGERED!")
        self.publishShutup.publish(String("shutup"))
        self.publishSpeechEvent.publish(String("Oh I'm sorry, did not mean to interrupt"))

      if not rospy.is_shutdown():
        threading.Timer(0.5, self.debugOutput).start()


    def parseIsSpeaking(self, msg):
      if msg.data == "start":
        self.sophiaSpeaking = True
      elif msg.data == "stop":
        self.sophiaSpeaking = False

    def resetLastSpeechInput(self):
      self.userSpokeRecently = False

    def parseLastSpeechInput(self, msg):
      self.userSpokeRecently = True
      threading.Timer(self.userSpokeThreshold, self.resetLastSpeechInput).start()


    def parseRealsense(self, msg):
        self.activeFaces = len(msg.faces)

        if self.activeFaces == 1:
          face = msg.faces[0]

          mouthopen = float(face.mouth_open) / 100.0

          self.avgMouthOpenStack.pop(0)
          self.avgMouthOpenStack.append(mouthopen)

          self.avgMouthOpen = np.array(self.avgMouthOpenStack).mean()
        else:
          self.avgMouthOpenStack = [0 for _ in range(self.avgMouthOpenWindowSize)]

          

if __name__ == "__main__":
    stream = realsense_stream()
    rospy.spin()

