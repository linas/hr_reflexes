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



        rospy.init_node('realsense')

        # Subscribers
        rospy.Subscriber('/realsense', Vision, self.parseRealsense)
        self.publishAnimation = rospy.Publisher('/sophia6/tts', TTS, queue_size=1)

        self.pub = rospy.Publisher('/blender_api/set_pau', pau, queue_size=10)


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


          msg = pau()

          msg.m_headRotation.x = face.quat.x
          msg.m_headRotation.y = face.quat.y
          msg.m_headRotation.z = face.quat.z
          msg.m_headRotation.w = face.quat.w

          self.pub.publish(msg)


          print("")

if __name__ == "__main__":
    stream = realsense_stream()
    rospy.spin()

