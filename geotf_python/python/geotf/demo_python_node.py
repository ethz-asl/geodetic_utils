#!/usr/bin/env python

import geotf
import time
import numpy as np
import rospy


if __name__ == "__main__":

    rospy.init_node("geotf_python_demo")

    converter = geotf.GeodeticConverter()
    # Add frames manually because ros related functions don't work through bindings atm
    converter.addFrameByENUOrigin("ENU_LEE", 47.37842, 8.54582, 489.619)
    converter.addFrameByGCSCode("GPS", "WGS84")
    converter.addFrameByUTM("UTM", 32, True)
    converter.addFrameByEPSG("CH1903+", 2056)

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

    rospy.signal_shutdown(True)
