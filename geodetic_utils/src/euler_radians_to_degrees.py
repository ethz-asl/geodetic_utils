#!/usr/bin/env python

# Simple Python publisher/subscriber node to convert a vector of orientations
# from degrees to radians

import rospy
from geometry_msgs.msg import Vector3
from math import *
import sys

class EulerDegreestoRadians:

  def __init__(self):
    self.radians_sub = rospy.Subscriber("input_topic", Vector3, self.callback)
    self.degrees_pub = rospy.Publisher("output_topic"), Vector3, queue_size=10)

  def callback(self, message):
    degrees = Vector3()
    degrees.x = math.radians(message.x)
    degrees.y = math.radians(message.y)
    degrees.z = math.radians(message.z)
    self.degrees_pub.publish(degrees)
    
if __name__ == '__main__':
  rospy.init_node('euler_degrees_to_radians', anonymous=True)
  node = EulerDegreestoRadians()
  rospy.spin()