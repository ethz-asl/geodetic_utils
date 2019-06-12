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
  sleep(1.0);

  Eigen::Affine3d body_coords(Eigen::Affine3d::Identity());
  Eigen::Affine3d utm_body_coords(Eigen::Affine3d::Identity());
  converter.convertFromTf("body",
                          body_coords,
                          "UTM",
                          &utm_body_coords);
  std::cout << utm_body_coords.translation() << std::endl;


  Eigen::Affine3d utm_building_point(Eigen::Affine3d::Identity());
  utm_building_point.translation().x() =  465727;
  utm_building_point.translation().y() =   5247291;
  utm_building_point.translation().z() =   489.619;
  converter.publishAsTf("UTM", utm_building_point, "CornerUTM");


  Eigen::Affine3d gps_building_point(Eigen::Affine3d::Identity());
  gps_building_point.translation().x() =  8.54616;
  gps_building_point.translation().y() =  47.37823;
  gps_building_point.translation().z() =   489.619;
  converter.publishAsTf("GPS", gps_building_point, "CornerGPS");

  Eigen::Affine3d ENU_building_point(Eigen::Affine3d::Identity());
  ENU_building_point.translation().x() =  14.58;
  ENU_building_point.translation().y() =  6.64;
  ENU_building_point.translation().z() =   0.0;
  converter.publishAsTf("ENU_LEE", ENU_building_point, "CornerENU");


  Eigen::Affine3d CH_building_point(Eigen::Affine3d::Identity());
  CH_building_point.translation().x() =  2683625.9;
  CH_building_point.translation().y() =  1248088.9;
  CH_building_point.translation().z() =   442.4;
  converter.publishAsTf("CH1903+", CH_building_point, "CornerCH");


  ros::spin();


  return 0;
}