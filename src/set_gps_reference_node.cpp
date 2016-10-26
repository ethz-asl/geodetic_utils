#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <string>
#include <vector>
#include <std_srvs/Empty.h>

double g_lat_ref;
double g_lon_ref;
double g_alt_ref;
int g_count = 1;
bool gps_ref_is_init;
int g_its;

enum EMode
{
  MODE_AVERAGE = 0,
  MODE_WAIT
};
// average over, or wait for, n GPS fixes
EMode g_mode;

bool reset_callback(std_srvs::Empty::Request&, std_srvs::Empty::Response&)
{
  g_count = 1;
  g_lat_ref = 0.0;
  g_lon_ref = 0.0;
  g_alt_ref = 0.0;
  ros::NodeHandle nh;
  nh.setParam("/gps_ref_is_init", false);
  return true;
}

void gps_callback(const sensor_msgs::NavSatFixConstPtr & msg)
{
  ros::NodeHandle nh;
  nh.getParam("/gps_ref_is_init", gps_ref_is_init);

  if (!gps_ref_is_init){


    if (msg->status.status < sensor_msgs::NavSatStatus::STATUS_FIX) {
      ROS_WARN_STREAM_THROTTLE(1, "No GPS fix");
      return;
    }

    g_lat_ref += msg->latitude;
    g_lon_ref += msg->longitude;
    g_alt_ref += msg->altitude;

    ROS_INFO("Current measurement: %3.8f, %3.8f, %4.2f", msg->latitude, msg->longitude, msg->altitude);

    if (g_count == g_its) {
      if (g_mode == MODE_AVERAGE) {
        g_lat_ref /= g_its;
        g_lon_ref /= g_its;
        g_alt_ref /= g_its;
      } else {
        g_lat_ref = msg->latitude;
        g_lon_ref = msg->longitude;
        g_alt_ref = msg->altitude;
      }


      nh.setParam("/gps_ref_latitude", g_lat_ref);
      nh.setParam("/gps_ref_longitude", g_lon_ref);
      nh.setParam("/gps_ref_altitude", g_alt_ref);
      nh.setParam("/gps_ref_is_init", true);

      ROS_INFO("Final reference position: %3.8f, %3.8f, %4.2f", g_lat_ref, g_lon_ref, g_alt_ref);

      return;
    } else {
      ROS_INFO("    Still waiting for %d measurements", g_its - g_count);
    }

  g_count++;
  }
}

int main(int argc, char** argv)
{

  ros::init(argc, argv, "set_gps_reference");
  ros::NodeHandle nh;

  nh.setParam("/gps_ref_is_init", false);

  g_lat_ref = 0.0;
  g_lon_ref = 0.0;
  g_alt_ref = 0.0;

  ros::V_string args;
  ros::removeROSArgs(argc, argv, args);

  g_its = 2;  // default number of fixes
  g_mode = MODE_AVERAGE;  // average by default

  // Look for argument: number of iterations/fixes
  if (args.size() >= 2) {  // executable name + number of iterations (2 args)
    const int parsed = atoi(args[1].c_str());
    if (parsed > 0)
      g_its = parsed;
  }

  // Look for argument: mode (average or wait)
  if (args.size() >= 3) {  // executable name + number of iterations + mode (3 args)
    if (args[2] == "wait")
      g_mode = MODE_WAIT;
  }

  ROS_INFO("Usage: set_gps_reference [n_fixes] [average|wait], defaults: n_fixes=50, average");

  ROS_INFO(
      "Taking %d measurements and %s\n", g_its,
      (g_mode == MODE_AVERAGE) ? "averaging to get the reference" : "taking the last as reference");

  ros::Subscriber gps_sub = nh.subscribe("gps", 1, &gps_callback);
  ros::ServiceServer reset_srv = nh.advertiseService("reset_gps_reference", &reset_callback);

  ros::spin();
}
