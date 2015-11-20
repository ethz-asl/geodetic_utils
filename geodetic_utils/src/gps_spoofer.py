#!/usr/bin/env python

import rospy
import random
from math import *

from sensor_msgs.msg import NavSatFix, Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped, PointStamped
import tf

class GPSSpoofer:
  def __init__(self):
    self.fix = NavSatFix()
    #self.fix.header.stamp = ...
    self.fix.header.frame_id = 'fcu'
    self.fix.status.status = 1
    self.fix.status.service = 1
    self.fix.latitude = 44.0
    self.fix.longitude = 44.0
    self.fix.altitude = 0.0
    # TODO: fill GPS covariance

    # TODO: load params from param server
    self.max_R_noise = 0.0 # [m] max tested: 0.15
    self.pub_period = 0.05 # [s], publish disturbed pose every X s

#    self.pos_var = pow(max(0.0707, self.max_R_noise/2.0), 2)
    self.pos_var = 0.005;

    self.latest_imu_message = Imu()

    self.pwc = PoseWithCovarianceStamped()
    self.pwc.header.frame_id = 'vicon' # doesn't really matter to MSF
    self.pwc.pose.covariance[6 * 0 + 0] = self.pos_var;
    self.pwc.pose.covariance[6 * 1 + 1] = self.pos_var;
    self.pwc.pose.covariance[6 * 2 + 2] = 0.005;
    self.pwc.pose.covariance[6 * 3 + 3] = 0.01;
    self.pwc.pose.covariance[6 * 4 + 4] = 0.01;
    self.pwc.pose.covariance[6 * 5 + 5] = 0.01;

    self.R_noise = 0.0
    self.theta_noise = 0.0
    self.timer_pub = None
    self.timer_resample = None
    self.got_odometry = False

    self.pub_spoofed_gps = rospy.Publisher('spoofed_gps', NavSatFix, queue_size=1)
    self.pub_disturbed_pose = rospy.Publisher('disturbed_pose', PoseWithCovarianceStamped, queue_size=1,  tcp_nodelay=True)

    self.sub_gps = rospy.Subscriber('fcu/gps', NavSatFix, self.callback_gps, tcp_nodelay=True)
    self.sub_estimated_odometry = rospy.Subscriber('estimated_odometry', Odometry, self.callback_odometry, queue_size=1, tcp_nodelay=True)
    self.sub_imu = rospy.Subscriber('imu/data_compass', Imu, self.callback_imu, queue_size=1, tcp_nodelay=True)
    self.sub_pressure_height = rospy.Subscriber('pressure_height_point', PointStamped, self.callback_pressure_height, queue_size=1, tcp_nodelay=True)

  def callback_gps(self, data):
    self.fix.header.stamp = data.header.stamp
    self.pub_spoofed_gps.publish(self.fix)

  def callback_odometry(self, data):
    #print "Got odometry!"
    #print "max_R_noise = ", self.max_R_noise
    #print "pos_var = ", self.pos_var

    self.pwc.header.stamp = rospy.Time.now()
#    self.pwc.pose.pose = data.pose.pose

    # Rotate x and y to align x to East
    x = data.pose.pose.position.x
    y = data.pose.pose.position.y

    angle = radians(24)

    self.pwc.pose.pose.position.x = cos(angle)*x - sin(angle)*y
    self.pwc.pose.pose.position.y = sin(angle)*x + cos(angle)*y

    # TODO: Take height measurement from pressure sensor
    self.pwc.pose.pose.position.z = data.pose.pose.position.z

    self.pwc.pose.pose.orientation = self.latest_imu_message.orientation


    if not self.got_odometry:
        print "GPSSpoofer: initializing timers"
        self.got_odometry = True
        self.timer_resample = rospy.Timer(rospy.Duration(5), self.sample_noise)
        self.timer_pub = rospy.Timer(rospy.Duration(self.pub_period), self.publish_odometry)

  def callback_imu(self, data):
    self.latest_imu_message = data

  def callback_pressure_height(self, data):
    # TODO: Take height measurement from pressure sensor
    #self.pose.pose.pose.position.z = data.point.z
    pass

  def sample_noise(self, event):
    #print "Resampling noise"
    # Generate noise in x and y
    self.R_noise = random.uniform(0, self.max_R_noise)
    self.theta_noise = random.uniform(0, 2*pi)

  def publish_odometry(self, event):
    #print "Publishing odometry"
    self.pwc.pose.pose.position.x += self.R_noise*cos(self.theta_noise)
    self.pwc.pose.pose.position.y += self.R_noise*sin(self.theta_noise)
    self.pub_disturbed_pose.publish(self.pwc)

if __name__ == '__main__':

  try:
    rospy.init_node('gps_spoofer', anonymous=True)
    gs = GPSSpoofer()
    rospy.spin()
  except rospy.ROSInterruptException:
    pass
