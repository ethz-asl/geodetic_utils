#!/usr/bin/env python

import rospy
import random
from math import *

from sensor_msgs.msg import NavSatFixtransform
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped, PointStamped

class GpsSpoofer:
  def __init__(self):
    self.fix = NavSatFix()
    #self.fix.header.stamp = ...
    self.fix.header.frame_id = 'fcu'
    self.fix.status.status = 1
    self.fix.status.service = 1
    self.fix.latitude = 44.0
    self.fix.longitude = 44.0
    self.fix.altitude = 0.0
    # TODO: fill covariance

    self.pose = PoseWithCovarianceStamped()
    self.pose.header.frame_id = 'fcu'
    self.pose.covariance[6 * 0 + 0] = 1;
    self.pose.covariance[6 * 1 + 1] = 1;
    self.pose.covariance[6 * 2 + 2] = 0.1;
    self.pose.covariance[6 * 3 + 3] = 0.01;
    self.pose.covariance[6 * 4 + 4] = 0.01;
    self.pose.covariance[6 * 5 + 5] = 0.01;

    # Height from pressure sensor
    self.height = 0.0

    self.pub_spoofed_gps = rospy.Publisher('spoofed_gps', NavSatFix, queue_size=1)
    self.pub_disturbed_pose = rospy.Publisher('disturbed_pose',   PoseWithCovarianceStamped, queue_size=1)
    self.sub_gps = rospy.Subscriber('fcu/gps', NavSatFix, self.callback_gps)
    self.sub_estimated_odometry = rospy.Subscriber('estimated_odometry', Odometry, self.callback_odometry)
    self.sub_pressure_height = rospy.Subscriber('pressure_height_point', PointStamped, self.callback_pressure_height)

  def callback_gps(self, data):
    self.fix.header.stamp = data.header.stamp
    self.pub_spoofed_gps.publish(self.fix)

  def callback_odometry(self, data):
    self.pose.header.stamp = data.header.stamp
    self.pose.pose.pose.position.x = data.pose.pose.position.x
    self.pose.pose.pose.position.y = data.pose.pose.position.y

  def callback_pressure_height(self, data):
    self.pose.pose.pose.position.z = data.point.z

  def publish_odometry(self):
    # Generate noise in x and y
    R_noise = random.uniform(0, 0.01)
    theta_noise = random.uniform(0, 2*pi)
    # Add noise
    self.pose.pose.pose.position.x = self.pose.pose.pose.position.x + R_noise*cos(theta_noise)
    self.pose.pose.pose.position.y = self.pose.pose.pose.position.y + R_noise*sin(theta_noise)
    self.pub_disturbed_pose.publish(self.pose)

if __name__ == '__main__':

  try:
    rospy.init_node('gps_spoofer', anonymous=True)
    gs = GpsSpoofer()
    counter = 0
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
      if (counter == 50)
        gs.publish_odometry(gs)
        counter = 0
      else
        counter = counter + 1
  except rospy.ROSInterruptException:
    pass
