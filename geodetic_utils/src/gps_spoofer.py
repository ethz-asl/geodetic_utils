#!/usr/bin/env python

import rospy
import random
from math import *

from sensor_msgs.msg import NavSatFix
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
    self.pose.pose.covariance[6 * 0 + 0] = 1;
    self.pose.pose.covariance[6 * 1 + 1] = 1;
    self.pose.pose.covariance[6 * 2 + 2] = 0.1;
    self.pose.pose.covariance[6 * 3 + 3] = 0.01;
    self.pose.pose.covariance[6 * 4 + 4] = 0.01;
    self.pose.pose.covariance[6 * 5 + 5] = 0.01;

    self.R_noise = 0.0
    self.theta_noise = 0.0

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
    # Take height measurement from vicon
    self.pose.pose.pose.position.z = data.pose.pose.position.z
    self.pose.pose.pose.orientation = data.pose.pose.orientation

  def callback_pressure_height(self, data):
    # Take height measurement from pressure sensor
    #self.pose.pose.pose.position.z = data.point.z
    pass

  def sample_noise(self, event):
    #print "Resampling noise"
    # Generate noise in x and y
    self.R_noise = 0.0000001 #random.uniform(0, 0.01)
    self.theta_noise = random.uniform(0, 2*pi)

  def publish_odometry(self, event):
    #print "Publishing odometry"
    # Add noise
    self.pose.pose.pose.position.x = self.pose.pose.pose.position.x + self.R_noise*cos(self.theta_noise)
    self.pose.pose.pose.position.y = self.pose.pose.pose.position.y + self.R_noise*sin(self.theta_noise)
    self.pub_disturbed_pose.publish(self.pose)

if __name__ == '__main__':

  try:
    rospy.init_node('gps_spoofer', anonymous=True)
    gs = GpsSpoofer()
    rospy.Timer(rospy.Duration(5), gs.sample_noise)
    rospy.Timer(rospy.Duration(0.2), gs.publish_odometry)
    rospy.spin()
  except rospy.ROSInterruptException:
    pass
