#!/usr/bin/env python

# Simple Python publisher/subscriber node to convert a vector of orientations from degrees to radians

import rospy
from geometry_msgs.msg import Vector3
import numpy

def callback_compass(message):
  degrees = Vector3()
  degrees.x = numpy.rad2deg(message.x)
  degrees.y = numpy.rad2deg(message.y)
  degrees.z = numpy.rad2deg(message.z)
  degrees_compass_pub.publish(degrees)

def callback_fcu(message):
  degrees = Vector3()
  degrees.x = numpy.rad2deg(message.x)
  degrees.y = numpy.rad2deg(message.y)
  degrees.z = numpy.rad2deg(message.z)
  degrees_fcu_pub.publish(degrees)

radians_compass_sub = rospy.Subscriber('/orientation_euler_compass', Vector3, callback_compass)
radians_fcu_sub = rospy.Subscriber('/orientation_euler_fcu', Vector3, callback_fcu)

degrees_compass_pub = rospy.Publisher('/orientation_euler_compass_degrees', Vector3, queue_size=10)
degrees_fcu_pub = rospy.Publisher('/orientation_euler_fcu_degrees', Vector3, queue_size=10)

if __name__ == '__main__':
  rospy.init_node('euler_degrees_to_radians', anonymous=True)
  rospy.spin()
