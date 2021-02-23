#!/usr/bin/env python
import rospy
import numpy as np
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from sensor_msgs.msg import MagneticField
from geometry_msgs.msg import TransformStamped
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
        self._input_q = []
        self._switched = False

        # Ros stuff
        self._dyn_rec_srv = Server(GpsSimConfig, self.config_callback)
        self._odom_pub = rospy.Publisher('gps_odom_out', Odometry, queue_size=1)
        self._tf_pub = rospy.Publisher("gps_transform_out", TransformStamped, queue_size=1)
        self._mag_pub = rospy.Publisher('mag_out', MagneticField, queue_size=1)
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
        odom_msg.header.frame_id = "enu"
        odom_msg.pose.pose.position.x = enu_pos[0]
        odom_msg.pose.pose.position.y = enu_pos[1]
        odom_msg.pose.pose.position.z = enu_pos[2]

        set_orientation = True
        if set_orientation:
            odom_msg.pose.pose.orientation.x = self._input_q[0]
            odom_msg.pose.pose.orientation.y = self._input_q[1]
            odom_msg.pose.pose.orientation.z = self._input_q[2]
            odom_msg.pose.pose.orientation.w = self._input_q[3]
        else:
            odom_msg.pose.pose.orientation.w = 1.0

        enu_cov_66 = np.eye(6) * 0.1
        enu_cov_66[0:3, 0:3] = enu_cov
        odom_msg.pose.covariance = enu_cov_66.flatten("C)")
        self._odom_pub.publish(odom_msg)

        # also output as transform stamped
        tf_msg = TransformStamped()
        tf_msg.header.stamp = self._input_stamp
        tf_msg.transform.translation.x = enu_pos[0]
        tf_msg.transform.translation.y = enu_pos[1]
        tf_msg.transform.translation.z = enu_pos[2]
        tf_msg.transform.rotation.x = 0
        tf_msg.transform.rotation.y = 0
        tf_msg.transform.rotation.z = 0
        tf_msg.transform.rotation.w = 1
        self._tf_pub.publish(tf_msg)

    def odom_callback(self, data):
        rospy.logdebug("Odometry received")

        #if data.header.stamp.secs >= 1613926010 and  data.header.stamp.secs < 1613926011 and self._gps_sim._current_mode == "rtk":
        #    rospy.logwarn("SWITCHED TO SPP")
        #    self._gps_sim.set_mode("float")

        #if data.header.stamp.secs >= 1613926010 + 30 and self._gps_sim._current_mode == "float":
        #    rospy.logwarn("SWITCHED TO RTK")
        #    self._gps_sim.set_mode("rtk")

        self._odom_received = True
        # extract pose
        input_pos = np.array([data.pose.pose.position.x, data.pose.pose.position.y, data.pose.pose.position.z])
        self._input_pose = transformations.quaternion_matrix(np.array([
            data.pose.pose.orientation.x,
            data.pose.pose.orientation.y,
            data.pose.pose.orientation.z,
            data.pose.pose.orientation.w
        ]))

        self._input_q = np.array([
            data.pose.pose.orientation.x,
            data.pose.pose.orientation.y,
            data.pose.pose.orientation.z,
            data.pose.pose.orientation.w
        ])

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
