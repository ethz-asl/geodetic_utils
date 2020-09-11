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
    roscpp_initializer.roscpp_init("roscpp_geotf_python_demo", [])
    # Add frames manually because ros related functions don't work through bindings atm
    converter.initFromRosParam()

    # Wait for TF to setup
    time.sleep(1.0)

    # Initialize ETH mainbuilding based on UTM coordiantes for example.
    eth_mainbuilding_utm = np.array([465882.064, 5247094.385, 498.217])

    # Output ETH mainbuilding in GPS frame
    if converter.canConvert("UTM", "GPS"):
        # Python does not support pointers, therefore python wrapper for
        # convert only takes 3 arguments and returns the output
        eth_mainbuilding_gps = converter.convert("UTM", eth_mainbuilding_utm, "GPS")

        rospy.loginfo(
            "ETH Mainbuilding WGS84 = %s",
            np.array2string(eth_mainbuilding_gps, precision=15),
        )
    else:
        rospy.logwarn("Frames not loaded.")

    # Output ETH mainbuilding in Swissgrid frame
    if converter.canConvert("UTM", "CH1903+"):
        eth_mainbuilding_ch = converter.convert("UTM", eth_mainbuilding_utm, "CH1903+")

        rospy.loginfo(
            "ETH Mainbuilding CH1903+/LV95 = %s",
            np.array2string(eth_mainbuilding_ch, precision=9),
        )
    else:
        rospy.logwarn("Frames not loaded.")

    # Output ETH mainbuilding in ENU frame based on LEE terasse
    if converter.canConvert("UTM", "ENU_LEE"):
        eth_mainbuilding_enu = converter.convert("UTM", eth_mainbuilding_utm, "ENU_LEE")

        rospy.loginfo(
            "ETH Mainbuilding in ENU Frame based on LEE Terasse = %s",
            np.array2string(eth_mainbuilding_enu, precision=15),
        )
    else:
        rospy.logwarn("Frames not loaded.")

    rospy.loginfo("Open RVIZ and press Enter to continue...")
    raw_input()

    # Example of directly converting TF locations into geo locations

    # Here we convert location 0/0/0 in tf frame "body" to UTM conversions
    # Note that we do not have to specify explictely how this is converted,
    # as we already configured the equivalence of Geoframe ENU_LEE and
    # tf frame enu in the launch file.

    # Python does not support Eigen::Affine therefore we pass a 4x4 Matrix.
    # The output is also returned as 4x4 Matrix
    body_coords = np.identity(4, dtype="double")
    utm_body_coords = converter.convertFromTf("body", body_coords, "UTM")
    rospy.loginfo(
        "UTM coordinates of body origin:\n%s", np.array2string(utm_body_coords[0:3, 3])
    )

    # Example of Publishing Geolocations as TF frames for visualization.

    # Publish TF Frame CornerUTM based on UTM coordinates
    # Note: Overloading Numpy Arrays (Vector & Affine) does not work currently. Therefore the wrapper
    # functions "publishAffAsTf" and "publishVecAsTf" should be used when using a 4x4 matrix or 
    # 3x1 vector, respectively
    utm_building_point = np.identity(4, dtype="double")
    # Translation x,y,z:
    utm_building_point[0,3] = 465727
    utm_building_point[1,3] = 5247291
    utm_building_point[2,3] = 489.619
    print(converter.publishAffAsTf("UTM", utm_building_point, "CornerUTM"))

    # Publish TF Frame CornerGPS based on UTM coordinates
    gps_building_point = np.identity(4, dtype="double")
    # Translation x,y,z:
    gps_building_point[0,3] = 47.37823
    gps_building_point[1,3] = 8.54616
    gps_building_point[2,3] = 489.619
    converter.publishAffAsTf("GPS", gps_building_point, "CornerGPS")

    # Publish TF Frame CornerENU based on ENU coordinates using a Vector
    ENU_building_point = np.zeros([3,1])
    # Translation x,y,z:
    ENU_building_point[0] = 14.58
    ENU_building_point[1] = 6.64
    ENU_building_point[2] = 0.0
    converter.publishVecAsTf("ENU_LEE", ENU_building_point, "CornerENU")

    # Publish TF Frame CornerCH based on CH1903+ coordinates
    CH_building_point = np.identity(4, dtype="double")
    # Translation x,y,z:
    CH_building_point[0,3] = 2683625.9
    CH_building_point[1,3] = 1248088.9
    CH_building_point[2,3] = 442.4
    converter.publishAffAsTf("CH1903+", CH_building_point, "CornerCH")

    



    rospy.signal_shutdown(True)
