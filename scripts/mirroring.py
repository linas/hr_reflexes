#!/usr/bin/env python
from time import sleep
import rospy
from realsense_ros.msg import *
from pau2motors.msg import pau
from std_msgs.msg import UInt8
from audio_stream.msg import audiodata

class realsense_stream:
    def __init__(self):
        # Configuratoin parameters
        self.decibelLimit = 40.0
        self.decibelCurrent = 0.0

        # percentage of sensor value that gets added each step.
        self.browDelay = 0.25
        self.mouthDelay = 0.0 # deactivate mouth mirroring...
        self.blinkDelay = 0.3

        #
        self.leftBrowRaise = 0.0
        self.leftBrowLower = 0.0
        self.rightBrowRaise = 0.0
        self.rightBrowLower = 0.0
        self.mouthOpen = 0.0
        self.eyesClosed = 0.0


        rospy.init_node('realsense')

        # Subscribers
        rospy.Subscriber('/realsense', Vision, self.parseRealsense)
        rospy.Subscriber('/sophia11/audio_sensors', audiodata, self.parseAudioSensor) # TODO: Make independent of robot name

        # Publishers
        self.pub = rospy.Publisher('/blender_api/set_pau', pau, queue_size=10)

        # Set the correct animation mode
        self.animationmodePub = rospy.Publisher('/blender_api/set_animation_mode', UInt8, queue_size=1)
        self.animationmode = UInt8()
        self.animationmode.data = 148
        sleep(1.5)
        self.animationmodePub.publish(self.animationmode)


    def parseAudioSensor(self, msg):
      self.decibelCurrent = msg.Decibel

    def parseRealsense(self, msg):
        if len(msg.faces) == 1:
          face = msg.faces[0]

          #if self.decibelCurrent >= self.decibelLimit:
          #  mouthopen = 0.0
          #else:
          #  mouthopen = float(face.mouth_open) / 100.0

          self.leftBrowRaise =  ((1.0 - self.browDelay) * self.leftBrowRaise) + (self.browDelay * (float(face.left_brow_raise) / 100.0))
          self.leftBrowLower =  ((1.0 - self.browDelay) * self.leftBrowLower) + (self.browDelay * (float(face.left_brow_lower) / 100.0))
          self.rightBrowRaise = ((1.0 - self.browDelay) * self.rightBrowRaise) + (self.browDelay * (float(face.right_brow_raise) / 100.0))
          self.rightBrowLower = ((1.0 - self.browDelay) * self.rightBrowLower) + (self.browDelay * (float(face.right_brow_lower) / 100.0))

          self.mouthOpen = ((1.0 - self.mouthDelay) * self.mouthOpen) + (self.mouthDelay * (float(face.mouth_open) / 100.0))
          self.eyesClosed = ((1.0 - self.blinkDelay) * self.eyesClosed) + (self.blinkDelay * (float(face.left_eye_closed + face.right_eye_closed) / 200.0))

          

          msg = pau()
          msg.m_coeffs = [self.mouthOpen, self.leftBrowRaise, self.leftBrowRaise/1.5, self.leftBrowLower, 
                          self.rightBrowRaise, self.rightBrowRaise/1.5, self.rightBrowLower,
                          self.eyesClosed, self.eyesClosed, self.eyesClosed, self.eyesClosed]

          msg.m_shapekeys = ['lip-JAW.DN', 'brow_outer_UP.L', 'brow_inner_UP.L', 'brow_outer_DN.L', 
                            'brow_outer_up.R', 'brow_inner_UP.R', 'brow_outer_DN.R',
                             'eye-blink.UP.R', 'eye-blink.UP.L', 'eye-blink.LO.R', 'eye-blink.LO.L']

          print(msg.m_coeffs)

          self.animationmodePub.publish(self.animationmode)
          self.pub.publish(msg)
          print("publishing")

if __name__ == "__main__":
    stream = realsense_stream()
    rospy.spin()

