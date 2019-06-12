#include <geotf/geodetic_converter.h>
#include <iomanip> // for std::setprecision()
#include <ros/ros.h>
int main(int argc, char* argv[]) {
  ros::init(argc, argv, "bla");
  ros::NodeHandle nh;

  geotf::GeodeticConverter converter;
  converter.initFromRosParam();

  Eigen::Vector3d eth_mainbuilding_utm, eth_mainbuilding_gps,
      eth_mainbuilding_ch, eth_mainbuilding_enu;
  eth_mainbuilding_utm << 465882.064, 5247094.385, 498.217;

  if (converter.canConvert("UTM", "GPS")) {
    converter.convert("UTM", eth_mainbuilding_utm,
                      "GPS", &eth_mainbuilding_gps);

    ROS_WARN_STREAM("ETH Mainbuilding WGS84 = " << std::setprecision(16)
                                                << eth_mainbuilding_gps);

  } else {
    ROS_WARN_STREAM("Frames not loaded.");
  }

  if (converter.canConvert("UTM", "CH1903+")) {
    converter.convert("UTM", eth_mainbuilding_utm,
                      "CH1903+", &eth_mainbuilding_ch);

    ROS_WARN_STREAM("ETH Mainbuilding CH1903+/LV95 = " << std::setprecision(16)
                                                       << eth_mainbuilding_ch);

  } else {
    ROS_WARN_STREAM("Frames not loaded.");
  }

  if (converter.canConvert("UTM", "ENU_LEE")) {
    converter.convert("UTM", eth_mainbuilding_utm,
                      "ENU_LEE", &eth_mainbuilding_enu);

    ROS_WARN_STREAM("ETH Mainbuilding in ENU Frame based on LEE Terasse = "
                        << std::setprecision(16)
                        << eth_mainbuilding_enu);

  } else {
    ROS_WARN_STREAM("Frames not loaded.");
  }

  return 0;
}