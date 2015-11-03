#!/usr/bin/env python

import rospy
from sensor_msgs.msg import NavSatFix

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
        
        self.pub = rospy.Publisher('spoofed_gps', NavSatFix, queue_size=1)
        self.sub = rospy.Subscriber("fcu/gps", NavSatFix, self.callback)
    
    def callback(self, data):
        self.fix.header.stamp = data.header.stamp
        self.pub.publish(self.fix)        

if __name__ == '__main__':

    try:    
        rospy.init_node('gps_spoofer', anonymous=True)
        gs = GpsSpoofer()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
