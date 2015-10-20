#!/usr/bin/env python

# Simple Python publisher/subscriber node to convert a vector of orientations from degrees to radians

import rospy
from geometry_msgs.msg import Vector3
import numpy

def callback(message):
  degrees = Vector3()
  degrees.x = numpy.rad2deg(message.x)
  degrees.y = numpy.rad2deg(message.y)
  degrees.z = numpy.rad2deg(message.z)
  degrees_pub.publish(degrees)

radians_sub = rospy.Subscriber('/orientation_euler', Vector3, callback)
degrees_pub = rospy.Publisher('/orientation_euler_degrees', Vector3, queue_size=10)

if __name__ == '__main__':
  rospy.init_node('euler_degrees_to_radians', anonymous=True)
  rospy.spin()
