#!/usr/bin/env python

import rospy
from sensor_msgs.msg import NavSatFix

if __name__ == '__main__':

    try:    
        rospy.init_node('gps_spoofer', anonymous=True)
        pub = rospy.Publisher('spoofed_gps', NavSatFix, queue_size=1)
        rate = rospy.Rate(5) # 5hz

        fix = NavSatFix()
        fix.header.frame_id = 'fcu'
        fix.status.status = 1
        fix.status.service = 1
        fix.latitude = 44.0
        fix.longitude = 44.0
        fix.altitude = 0.0
        # TODO: fill covariance        
        
        while not rospy.is_shutdown():
            fix.header.stamp = rospy.Time.now()
            pub.publish(fix)
            rate.sleep()
    except rospy.ROSInterruptException:
        pass
