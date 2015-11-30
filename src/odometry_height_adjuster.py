#!/usr/bin/env python

"""
This node listens to an odometry topic, takes the Z position value in the
first measurement received, and republishes new odometry messages with their
Z position adjusted to use the initial Z position as reference.
"""

import rospy
from nav_msgs.msg import Odometry

class OdometryHeightAdjuster:
    def __init__(self):
        self.got_first = False
        self.offset = 0.0
        self.pub = rospy.Publisher('height_adjusted_odometry', Odometry, queue_size=1)
        self.sub = rospy.Subscriber("odometry", Odometry, self.callback)
    
    def callback(self, data):
        if not self.got_first:
            self.offset = data.pose.pose.position.z
            self.got_first = True
        ao = data
        data.pose.pose.position.z -= self.offset 
        self.pub.publish(ao)

if __name__ == '__main__':
    # In ROS, nodes are uniquely named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('odometry_height_adjuster', anonymous=True)

    oha = OdometryHeightAdjuster()

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()