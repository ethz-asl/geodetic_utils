#include <geotf/geodetic_converter.h>
#include <iomanip> // for std::setprecision()
#include <ros/ros.h>
int main(int argc, char* argv[]) {
  ros::init(argc, argv, "bla");
  ros::NodeHandle nh;

  /*
   * Example of how to use geotf
   * - Lunch using demo.launch
   */

   /* Read frame configuration from rosparams.
    * Configred Geo frames in that launch file:
    *   - ENU_LEE: Enu frame with origin on the LEE terasse at ETH
    *   - GPS: WGS84 GPS frame (so x=lon, y = lat, z = alt)
    *   - UTM: UTM 32 North frame (x = easting, y = northing, z = altitude)
    *   - CH1903+: Swissgrid based on new CH1903+ coordinates and Landesvermessung 95.
    */
  geotf::GeodeticConverter converter;
  converter.initFromRosParam();

  // Wait for TF to setup
  sleep(1.0);

  // A few vectors for conversions
  Eigen::Vector3d eth_mainbuilding_utm, eth_mainbuilding_gps,
      eth_mainbuilding_ch, eth_mainbuilding_enu;

  // Initialize ETH mainbuilding based on UTM coordiantes for example.
  eth_mainbuilding_utm << 465882.064, 5247094.385, 498.217;

  // Output ETH mainbuilding in GPS frame
  if (converter.canConvert("UTM", "GPS")) {
    converter.convert("UTM", eth_mainbuilding_utm,
                      "GPS", &eth_mainbuilding_gps);

    ROS_INFO_STREAM("ETH Mainbuilding WGS84 = " << std::setprecision(16)
                                                << eth_mainbuilding_gps);

  } else {
    ROS_WARN_STREAM("Frames not loaded.");
  }

  // Output ETH mainbuilding in Swissgrid frame
  if (converter.canConvert("UTM", "CH1903+")) {
    converter.convert("UTM", eth_mainbuilding_utm,
                      "CH1903+", &eth_mainbuilding_ch);

    ROS_INFO_STREAM("ETH Mainbuilding CH1903+/LV95 = " << std::setprecision(16)
                                                       << eth_mainbuilding_ch);

  } else {
    ROS_WARN_STREAM("Frames not loaded.");
  }

  // Output ETH mainbuilding in ENU frame based on LEE terasse
  if (converter.canConvert("UTM", "ENU_LEE")) {
    converter.convert("UTM", eth_mainbuilding_utm,
                      "ENU_LEE", &eth_mainbuilding_enu);

    ROS_INFO_STREAM("ETH Mainbuilding in ENU Frame based on LEE Terasse = "
                        << std::setprecision(16)
                        << eth_mainbuilding_enu);

  } else {
    ROS_WARN_STREAM("Frames not loaded.");
  }


  ROS_INFO_STREAM("Please start rviz for visualization and press enter.");
  std::cin.get();

  // Example of directly converting TF locations into geo locations

  // Here we convert location 0/0/0 in tf frame "body" to UTM conversions
  // Note that we do not have to specify explictely how this is converted,
  // as we already configured the equivalence of Geoframe ENU_LEE and
  // tf frame enu in the launch file.


  Eigen::Affine3d body_coords(Eigen::Affine3d::Identity());
  Eigen::Affine3d utm_body_coords(Eigen::Affine3d::Identity());
  converter.convertFromTf("body",
                          body_coords,
                          "UTM",
                          &utm_body_coords);
  ROS_INFO_STREAM("UTM coordinates of body origin:");
  std::cout << utm_body_coords.translation() << std::endl;


  // Example of Publishing Geolocations as TF frames for visualization.

  // Publish TF Frame CornerUTM based on UTM coordinates
  Eigen::Affine3d utm_building_point(Eigen::Affine3d::Identity());
  utm_building_point.translation().x() = 465727;
  utm_building_point.translation().y() = 5247291;
  utm_building_point.translation().z() = 489.619;
  std::cout << converter.publishAsTf("UTM", utm_building_point, "CornerUTM") << std::endl;

  // Publish TF Frame CornerGPS based on UTM coordinates
  Eigen::Affine3d gps_building_point(Eigen::Affine3d::Identity());
  gps_building_point.translation().x() = 47.37823;
  gps_building_point.translation().y() = 8.54616;
  gps_building_point.translation().z() = 489.619;
  converter.publishAsTf("GPS", gps_building_point, "CornerGPS");

  // Publish TF Frame CornerENU based on ENU coordinates
  Eigen::Affine3d ENU_building_point(Eigen::Affine3d::Identity());
  ENU_building_point.translation().x() = 14.58;
  ENU_building_point.translation().y() = 6.64;
  ENU_building_point.translation().z() = 0.0;
  converter.publishAsTf("ENU_LEE", ENU_building_point, "CornerENU");

  // Publish TF Frame CornerCH based on CH1903+ coordinates
  Eigen::Affine3d CH_building_point(Eigen::Affine3d::Identity());
  CH_building_point.translation().x() = 2683625.9;
  CH_building_point.translation().y() = 1248088.9;
  CH_building_point.translation().z() = 442.4;
  converter.publishAsTf("CH1903+", CH_building_point, "CornerCH");

  ros::spin();

  return 0;
}