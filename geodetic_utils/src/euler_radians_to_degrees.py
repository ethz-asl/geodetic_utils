#!/usr/bin/env python

# Simple Python publisher/subscriber node to convert a vector of orientations
# from degrees to radians

import rospy
from geometry_msgs.msg import Vector3
from math import *
import sys

class EulerRadiansToDegrees:

  def __init__(self):
    self.radians_sub = rospy.Subscriber('input_topic', Vector3, self.callback)
    self.degrees_pub = rospy.Publisher('output_topic', Vector3, queue_size=10)

  def callback(self, message):
    degrees_out = Vector3()
    degrees_out.x = degrees(message.x)
    degrees_out.y = degrees(message.y)
    degrees_out.z = degrees(message.z)
    self.degrees_pub.publish(degrees_out)

if __name__ == '__main__':
  rospy.init_node('euler_radians_to_degrees', anonymous=True)
  node = EulerRadiansToDegrees()
  rospy.spin()
