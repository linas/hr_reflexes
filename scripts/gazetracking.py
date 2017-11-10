#!/usr/bin/env python
from time import sleep
import rospy
from realsense_ros.msg import *
from pau2motors.msg import pau
from std_msgs.msg import UInt8
from audio_stream.msg import audiodata

from hr_msgs.msg import TTS

class realsense_stream:
    def __init__(self):

        self.gazeDelay = 0.05
        self.gazeThreshold = 0.8

        self.gazeLeft = 0.0
        self.gazeRight = 0.0
        self.gazeUp = 0.0
        self.gazeDown = 0.0


        rospy.init_node('realsense')

        # Subscribers
        rospy.Subscriber('/realsense', Vision, self.parseRealsense)
        self.publishAnimation = rospy.Publisher('/sophia6/tts', TTS, queue_size=1)

    def doAnimation(self, animation, speed=1.0, magnitude=1.0):
      msg = TTS()
      msg.text = "|%s,%.1f,%.1f|" % (animation, speed, magnitude)
      msg.lang = "en-US"
      print msg.text
      self.publishAnimation.publish(msg)

    def resetValues(self):
      self.gazeLeft = 0.0
      self.gazeRight = 0.0
      self.gazeUp = 0.0
      self.gazeDown = 0.0

    def parseRealsense(self, msg):
        if len(msg.faces) != 1:
          self.resetValues()
        else:
          face = msg.faces[0]

          self.gazeLeft =  ((1.0 - self.gazeDelay) * self.gazeLeft) + (self.gazeDelay * (float(face.eyes_turn_left) / 100.0))
          self.gazeRight =  ((1.0 - self.gazeDelay) * self.gazeRight) + (self.gazeDelay * (float(face.eyes_turn_right) / 100.0))
          self.gazeUp =  ((1.0 - self.gazeDelay) * self.gazeUp) + (self.gazeDelay * (float(face.eyes_turn_up) / 100.0))
          self.gazeDown =  ((1.0 - self.gazeDelay) * self.gazeDown) + (self.gazeDelay * (float(face.eyes_turn_down) / 100.0))

          self.looksLeft = self.gazeLeft > self.gazeThreshold
          self.looksRight = self.gazeRight > self.gazeThreshold
          self.looksUp = self.gazeUp > self.gazeThreshold
          self.looksDown = self.gazeDown > self.gazeThreshold

          print("  %.1f" % self.gazeUp)
          print("%.1f   %.1f " % (self.gazeLeft, self.gazeRight))
          print("  %.1f" % self.gazeDown)

          if (self.looksLeft):
            self.doAnimation("lookr", 0.6, 1.0)
            print("animation!")
            self.resetValues()
            

          if (self.looksRight):
            self.doAnimation("lookl", 0.6, 1.0)
            print("animation!")
            self.resetValues()

          print("")

if __name__ == "__main__":
    stream = realsense_stream()
    rospy.spin()

