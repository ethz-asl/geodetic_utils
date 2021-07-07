#!/usr/bin/env python

import geotf
import roscpp_initializer
import time
import numpy as np
import rospy

"""  
    Example of how to use geotf (copied from geotf Cpp library)
    - Lunch using demo_python.launch
   

    Read frame configuration from rosparams.
     Configred Geo frames in that launch file:
       - ENU_LEE: Enu frame with origin on the LEE terasse at ETH
       - GPS: WGS84 GPS frame (so x=lon, y = lat, z = alt)
       - UTM: UTM 32 North frame (x = easting, y = northing, z = altitude)
       - CH1903+: Swissgrid based on new CH1903+ coordinates and Landesvermessung 95.
"""

if __name__ == "__main__":

    rospy.init_node("geotf_python_demo")

    converter = geotf.GeodeticConverter()
    # Call roscpp init to use ROS related functions through bindings
    roscpp_initializer.roscpp_init("roscpp_geotf_mesh_frame_helper", [])
    # Add frames manually because ros related functions don't work through bindings atm
    converter.initFromRosParam()

    # Wait for TF to setup
    time.sleep(1.0)

    # Initialize Mesh origin based on UTM coordinates
    mesh_origin_utm = np.array([ rospy.get_param('~mesh_origin_utm_x'),rospy.get_param('~mesh_origin_utm_y'), rospy.get_param('~mesh_origin_utm_z')])

    # output static TF
    converter.publishVecAsTf("UTM", mesh_origin_utm, "mesh")
    rospy.spin()

