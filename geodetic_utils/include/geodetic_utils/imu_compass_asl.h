/*imu_compass.cpp
Authors: Prasenjit (pmukherj@clearpathrobotics.com)
Copyright (c) 2013, Clearpath Robotics, Inc., All rights reserved.
Redistribution and use in source and binary forms, with or without modification, are permitted provided that the
following conditions are met:
 * Redistributions of source code must retain the above copyright notice, this list of conditions and the following
   disclaimer.
 * Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following
   disclaimer in the documentation and/or other materials provided with the distribution.
 * Neither the name of Clearpath Robotics nor the names of its contributors may be used to endorse or promote products
   derived from this software without specific prior written permission.
THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

Description:
Header file for IMU Compass Class that combines gyroscope and magnetometer data to get a clean estimate of yaw.
*/

#include "ros/ros.h"
#include "tf/tf.h"
#include "tf/transform_listener.h"

#include "sensor_msgs/Imu.h"
#include "geometry_msgs/Vector3Stamped.h"
#include "std_msgs/Float32.h"

//typedef boost::shared_ptr<sensor_msgs::Imu const> ImuConstPtr;

class IMUCompass {

private:
  ros::NodeHandle node_;
  ros::Subscriber imu_sub_;
  ros::Subscriber mag_sub_;
  ros::Subscriber decl_sub_;
  ros::Publisher imu_pub_;
  ros::Publisher mag_pub_;
  ros::Publisher compass_pub_;
  ros::Publisher raw_compass_pub_;

  tf::TransformListener listener_;
  ros::Timer debug_timer_;

  void imuCallback(sensor_msgs::ImuPtr data);
  void declCallback(const std_msgs::Float32& data);
  void magCallback(const geometry_msgs::Vector3StampedConstPtr& data);
  void debugCallback(const ros::TimerEvent&);
  void repackageImuPublish(tf::StampedTransform);

  //Heading Filter functions
  void initFilter(double heading_meas); //initialize heading fiter
  bool first_mag_reading_; //signifies receiving the first magnetometer message
  bool first_gyro_reading_; //signifies receiving the first gyroscope message
  bool filter_initialized_; //after receiving the first measurement, make sure the filter is initialized
  bool gyro_update_complete_; //sigfnifies that a gyro update (motion model update) has gone through

  double mag_zero_x_, mag_zero_y_, mag_zero_z_;

  sensor_msgs::ImuPtr curr_imu_reading_;

  //Heading Filter Variables
  //State and Variance
  double curr_heading_;
  double curr_heading_variance_;
  double sensor_timeout_;

  //Motion Update Variables
  double heading_prediction_;
  double heading_variance_prediction_;
  double heading_prediction_variance_;
  double mag_declination_;
  double last_motion_update_time_;
  double last_measurement_update_time_;

  //Measurement Update Variables
  double yaw_meas_variance_;

public:
  IMUCompass(ros::NodeHandle &n);
  ~IMUCompass() {
  }
};

