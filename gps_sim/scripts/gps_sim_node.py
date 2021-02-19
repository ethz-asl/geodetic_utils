#!/usr/bin/env python
import rospy
import numpy as np
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from sensor_msgs.msg import MagneticField
from GpsSimulator import GpsSimulator
from tf import transformations

from dynamic_reconfigure.server import Server
from gps_sim.cfg import GpsSimConfig


class GpsSimNode:
    def __init__(self):
        rospy.init_node('gps_sim_node', anonymous=True)

        self._gps_sim = GpsSimulator()

        # odom caches
        self._odom_received = False
        self._input_pose = []
        self._input_stamp = []

        # Ros stuff
        self._dyn_rec_srv = Server(GpsSimConfig, self.config_callback)
        self._odom_pub = rospy.Publisher('gps_out', Odometry, queue_size=10)
        self._mag_pub = rospy.Publisher('mag_out', MagneticField, queue_size=10)
        rospy.Subscriber("odom_in", Odometry, self.odom_callback)

    def config_callback(self, cfg, lvl):
        rospy.logwarn("Changing to " + cfg.sim_mode)
        self._gps_sim.set_mode(cfg.sim_mode)
        return cfg

    def publish_at_rate(self):
        if not self._odom_received:
            rospy.logwarn("No odometry received")
            return

        self._odom_received = False  # mark data as consumed
        # to prevent republishing empty data...

        # get noised data
        enu_pos, enu_cov, mag_data = self._gps_sim.simulate(self._input_pose)

        # assemble messages.
        mag_msg = MagneticField()
        mag_msg.header.stamp = self._input_stamp
        mag_msg.magnetic_field.x = mag_data[0]
        mag_msg.magnetic_field.y = mag_data[1]
        mag_msg.magnetic_field.z = mag_data[2]
        # print(np.arctan2(mag_data[1], mag_data[0]) * (180 / 3.1415))
        self._mag_pub.publish(mag_msg)

        if enu_pos is None or enu_cov is None:
            # no gps data :(
            return

        odom_msg = Odometry()
        odom_msg.header.stamp = self._input_stamp
        odom_msg.pose.pose.position.x = enu_pos[0]
        odom_msg.pose.pose.position.y = enu_pos[1]
        odom_msg.pose.pose.position.z = enu_pos[2]

        # to be verified: if covariance matrix is correctly assembled
        odom_msg.pose.covariance[0] = enu_cov[0, 0]
        odom_msg.pose.covariance[1] = enu_cov[0, 1]
        odom_msg.pose.covariance[2] = enu_cov[0, 2]
        odom_msg.pose.covariance[6] = enu_cov[1, 0]
        odom_msg.pose.covariance[7] = enu_cov[1, 1]
        odom_msg.pose.covariance[8] = enu_cov[1, 2]
        odom_msg.pose.covariance[12] = enu_cov[2, 0]
        odom_msg.pose.covariance[13] = enu_cov[2, 1]
        odom_msg.pose.covariance[14] = enu_cov[2, 2]
        self._odom_pub.publish(odom_msg)

    def odom_callback(self, data):
        rospy.logdebug("Odometry received")

        self._odom_received = True
        # extract pose
        input_pos = np.array([data.pose.pose.position.x, data.pose.pose.position.y, data.pose.pose.position.z])
        self._input_pose = transformations.quaternion_matrix(np.array([
            data.pose.pose.orientation.x,
            data.pose.pose.orientation.y,
            data.pose.pose.orientation.z,
            data.pose.pose.orientation.w
        ]))

        # assemble 4x4 matrix
        self._input_pose[0:3, 3] = input_pos
        self._input_stamp = data.header.stamp

    def run(self):
        r = rospy.Rate(10)  # 10hz
        while not rospy.is_shutdown():
            self.publish_at_rate()
            r.sleep()


if __name__ == '__main__':
    node = GpsSimNode()
    node.run()
