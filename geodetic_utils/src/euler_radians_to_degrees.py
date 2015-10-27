#!/usr/bin/env python

# Simple Python publisher/subscriber node to convert a vector of orientations
# from degrees to radians

import rospy
from geometry_msgs.msg import Vector3
import numpy
import sys

class eulerDegreestoRadians:
  
  def __init__(self):
    self.radians_sub = rospy.Subscriber(rospy.get_param('~input_topic'), Vector3, self.callback)
    self.degrees_pub = rospy.Publisher(rospy.get_param('~output_topic'), Vector3, queue_size=10)

  def callback(self, message):
    degrees = Vector3()
    degrees.x = numpy.rad2deg(message.x)
    degrees.y = numpy.rad2deg(message.y)
    degrees.z = numpy.rad2deg(message.z)
    self.degrees_pub.publish(degrees)
    
if __name__ == '__main__':
  rospy.init_node('euler_degrees_to_radians', anonymous=True)
  node = eulerDegreestoRadians()
  rospy.spin()
