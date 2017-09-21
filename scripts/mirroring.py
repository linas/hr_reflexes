#!/usr/bin/env python
from time import sleep
import rospy
from realsense_ros.msg import *
from pau2motors.msg import pau
from std_msgs.msg import UInt8

class realsense_stream:
    def __init__(self):
        rospy.init_node('realsense')
        rospy.Subscriber('/realsense', Vision, self.parseRSStream)

        self.pub = rospy.Publisher('/blender_api/set_pau', pau, queue_size=10)

        # Set the correct animation mode
        self.animationmodePub = rospy.Publisher('/blender_api/set_animation_mode', UInt8, queue_size=1)
        self.animationmode = UInt8()
        self.animationmode.data = 148
        sleep(1.5)
        self.animationmodePub.publish(self.animationmode)

    def parseRSStream(self, msg):
        if len(msg.faces)>0:
          face = msg.faces[0]

          mouthopen = float(face.mouth_open) / 100.0
          left_brow_raise = float(face.left_brow_raise) / 100.0
          left_brow_lower = float(face.left_brow_lower) / 100.0
          right_brow_raise = float(face.right_brow_raise) / 100.0
          right_brow_lower = float(face.right_brow_lower) / 100.0

          eye_closed_signal = float(face.left_eye_closed + face.right_eye_closed) / 200.0

          msg = pau()
          msg.m_coeffs = [mouthopen, left_brow_raise, left_brow_lower, right_brow_raise, right_brow_lower,
                          eye_closed_signal, eye_closed_signal, eye_closed_signal, eye_closed_signal]
          msg.m_shapekeys = ['lip-JAW.DN', 'brow_outer_UP.L', 'brow_outer_DN.L', 'brow_outer_up.R', 'brow_outer_DN.R',
                             'eye-blink.UP.R', 'eye-blink.UP.L', 'eye-blink.LO.R', 'eye-blink.LO.L']

          self.animationmodePub.publish(self.animationmode)
          self.pub.publish(msg)
          print("publishing")

if __name__ == "__main__":
    stream = realsense_stream()
    rospy.spin()
