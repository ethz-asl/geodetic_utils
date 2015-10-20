/*
imu_compass.cpp
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
CPP file for IMU Compass Class that combines gyroscope and magnetometer data to get a clean estimate of yaw.
*/

#include "geodetic_utils/imu_compass_asl.h"

double magn(tf::Vector3 a) {
      return sqrt(a.x()*a.x() + a.y()*a.y() + a.z()*a.z());
}

IMUCompass::IMUCompass(ros::NodeHandle &n) :
    node_(n), curr_imu_reading_(new sensor_msgs::Imu()) {
  // Acquire Parameters
  ros::param::param("~mag_bias/x", mag_zero_x_, 0.0);
  ros::param::param("~mag_bias/y", mag_zero_y_, 0.0);
  ros::param::param("~mag_bias/z", mag_zero_z_, 0.0);


  ROS_INFO("Using magnetometer bias (x,y):%f,%f", mag_zero_x_, mag_zero_y_);

  ros::param::param("~compass/sensor_timeout", sensor_timeout_, 0.5);
  ros::param::param("~compass/yaw_meas_variance", yaw_meas_variance_, 10.0);
  ros::param::param("~compass/gyro_meas_variance", heading_prediction_variance_, 0.01);
  ROS_INFO("Using variance %f", yaw_meas_variance_);

  ros::param::param("~compass/mag_declination", mag_declination_, 0.0);
  ROS_INFO("Using magnetic declination %f (%f degrees)", mag_declination_, mag_declination_ * 180 / M_PI);

  // Setup Subscribers
  imu_sub_ = node_.subscribe("imu/data", 1000, &IMUCompass::imuCallback, this);
  mag_sub_ = node_.subscribe("imu/mag", 1000, &IMUCompass::magCallback, this);
  decl_sub_ = node_.subscribe("imu/declination", 1000, &IMUCompass::declCallback, this);
  imu_pub_ = node_.advertise<sensor_msgs::Imu>("imu/data_compass", 1);
  compass_pub_ = node_.advertise<std_msgs::Float32>("imu/compass_heading", 1);
  mag_pub_ = node_.advertise<geometry_msgs::Vector3Stamped>("imu/mag_calib", 1);

  raw_compass_pub_ = node_.advertise<std_msgs::Float32>("imu/raw_compass_heading", 1);

  first_mag_reading_ = false;
  first_gyro_reading_ = false;
  gyro_update_complete_ = false;
  last_motion_update_time_ = ros::Time::now().toSec();
  debug_timer_ = node_.createTimer(ros::Duration(1), &IMUCompass::debugCallback, this);

  ROS_INFO("Compass Estimator Started");
}

void IMUCompass::debugCallback(const ros::TimerEvent&) {
  if (!first_gyro_reading_)
    ROS_WARN("Waiting for IMU data, no gyroscope data available)");
  if (!first_mag_reading_)
    ROS_WARN("Waiting for mag data, no magnetometer data available, Filter not initialized");

  if ((ros::Time::now().toSec() - last_motion_update_time_ > sensor_timeout_) && first_gyro_reading_) {
    // gyro data is coming in too slowly
    ROS_WARN("Gyroscope data being receieved too slow or not at all");
    first_gyro_reading_ = false;
  }

  if ((ros::Time::now().toSec() - last_measurement_update_time_ > sensor_timeout_) && first_mag_reading_) {
    // gyro data is coming in too slowly
    ROS_WARN("Magnetometer data being receieved too slow or not at all");
    filter_initialized_ = false;
    first_mag_reading_ = false;
  }
}

void IMUCompass::imuCallback(const sensor_msgs::ImuPtr data) {
  // Transform Data and get the yaw direction
  geometry_msgs::Vector3 gyro_vector;
  geometry_msgs::Vector3 gyro_vector_transformed;
  gyro_vector = data->angular_velocity;

  if(!first_gyro_reading_)
    first_gyro_reading_ = true;

  double dt = ros::Time::now().toSec() - last_motion_update_time_;
  last_motion_update_time_ = ros::Time::now().toSec();
  tf::StampedTransform transform;

  try {
    listener_.lookupTransform("base_link", data->header.frame_id, ros::Time(0), transform);
  } catch (tf::TransformException &ex) {
    ROS_WARN("Missed transform between base_link and %s", data->header.frame_id.c_str());
    return;
  }

  tf::Vector3 orig_bt;
  tf::Matrix3x3 transform_mat(transform.getRotation());
  tf::vector3MsgToTF(gyro_vector, orig_bt);
  tf::vector3TFToMsg(orig_bt * transform_mat, gyro_vector_transformed);
  double yaw_gyro_reading =  gyro_vector_transformed.z;

  // Run Motion Update
  if (filter_initialized_) {
    heading_prediction_ = curr_heading_ + yaw_gyro_reading * dt;  // xp = A*x + B*u
    heading_variance_prediction_ = curr_heading_variance_ + heading_prediction_variance_; // Sp = A*S*A' + R

    if (heading_prediction_ > 3.14159)
      heading_prediction_ -= 2 * 3.14159;
    else if(heading_prediction_ < -3.14159)
      heading_prediction_ += 2 * 3.14159;
    gyro_update_complete_ = true;
  }
  curr_imu_reading_ = data;
}

void IMUCompass::declCallback(const std_msgs::Float32& data) {
  mag_declination_ = data.data;
  ROS_INFO("Using magnetic declination %f (%f degrees)", mag_declination_, mag_declination_ * 180 / M_PI);
}

void IMUCompass::magCallback(const geometry_msgs::Vector3StampedConstPtr& data) {
  geometry_msgs::Vector3 imu_mag = data->vector;
  geometry_msgs::Vector3 imu_mag_transformed;

  imu_mag.x = data->vector.x*0.25;
  imu_mag.y = data->vector.y*0.25;
  imu_mag.z = data->vector.z*0.25;

  last_measurement_update_time_ = ros::Time::now().toSec();
  tf::StampedTransform transform;
  try {
    listener_.lookupTransform("base_link", data->header.frame_id, ros::Time(0), transform);
  } catch (tf::TransformException &ex) {
    ROS_WARN("Missed transform between base_link and %s", data->header.frame_id.c_str());
    return;
  }

  tf::Vector3 orig_bt;
  tf::Matrix3x3 transform_mat(transform.getRotation());
  tf::vector3MsgToTF(imu_mag, orig_bt);
  tf::vector3TFToMsg(orig_bt * transform_mat, imu_mag_transformed);

  // Compensate for hard iron
  double mag_x = imu_mag_transformed.x - mag_zero_x_;
  double mag_y = imu_mag_transformed.y - mag_zero_y_;
  double mag_z = imu_mag_transformed.z;  // calibration is purely 2D

  // Normalize vector
  tf::Vector3 calib_mag(mag_x, mag_y, mag_z);
  calib_mag = calib_mag / magn(calib_mag);
  mag_x = calib_mag.x();
  mag_y = calib_mag.y();
  mag_z = calib_mag.z();

  geometry_msgs::Vector3Stamped calibrated_mag;
  calibrated_mag.header.stamp = ros::Time::now();
  calibrated_mag.header.frame_id = "imu_link";

  calibrated_mag.vector.x = calib_mag.x();
  calibrated_mag.vector.y = calib_mag.y();
  calibrated_mag.vector.z = calib_mag.z();

  mag_pub_.publish(calibrated_mag);

  tf::Quaternion q;
  tf::quaternionMsgToTF(curr_imu_reading_->orientation, q);
  tf::Transform curr_imu_meas;
  curr_imu_meas = tf::Transform(q, tf::Vector3(0, 0, 0));
  curr_imu_meas = curr_imu_meas * transform;
  tf::Quaternion orig (transform.getRotation());

  // Till Compensation
  tf::Matrix3x3 temp(curr_imu_meas.getRotation());

  double c_r, c_p, c_y;
  temp.getRPY(c_r, c_p, c_y);
  double cos_pitch = cos(c_p);
  double sin_pitch = sin(c_p);
  double cos_roll = cos(c_r);
  double sin_roll = sin(c_r);
  double t_mag_x = mag_x * cos_pitch + mag_z * sin_pitch;
  double t_mag_y = mag_x * sin_roll * sin_pitch + mag_y * cos_roll - mag_z * sin_roll * cos_pitch;
  double head_x = t_mag_x;
  double head_y = t_mag_y;

  // Retrieve magnetometer heading
  double heading_meas = atan2(head_x, head_y);

  // If this is the first magnetometer reading, initialize filter
  if (!first_mag_reading_) {
    // Initialize filter
    initFilter(heading_meas);
    first_mag_reading_ = true;
    return;
  }
  // If gyro update (motion update) is complete, run measurement update and publish imu data
  if (gyro_update_complete_) {
    // K = Sp*C'*inv(C*Sp*C' + Q)
    double kalman_gain = heading_variance_prediction_ * (1 / (heading_variance_prediction_ + yaw_meas_variance_));
    double innovation = heading_meas - heading_prediction_;
    if (abs(innovation) > M_PI)  // large change, signifies a wraparound. kalman filters don't like discontinuities like wraparounds, handle seperately.
      curr_heading_ = heading_meas;
    else
      curr_heading_ = heading_prediction_ + kalman_gain * (innovation); // mu = mup + K*(y-C*mup)

    curr_heading_variance_ = (1 - kalman_gain) * heading_variance_prediction_; // S = (1-K*C)*Sp

    std_msgs::Float32 raw_heading_float;
    raw_heading_float.data = heading_meas;
    raw_compass_pub_.publish(raw_heading_float);

    repackageImuPublish(transform);
    gyro_update_complete_ = false;
  }
}

void IMUCompass::repackageImuPublish(tf::StampedTransform transform) {
  // Get Current IMU reading and Compass heading
  tf::Quaternion imu_reading;
  tf::quaternionMsgToTF(curr_imu_reading_->orientation, imu_reading);
  double compass_heading = curr_heading_ - mag_declination_;

  // Transform curr_imu_reading to base_link
  tf::Transform o_imu_reading;
  o_imu_reading = tf::Transform(imu_reading, tf::Vector3(0, 0, 0));
  o_imu_reading = o_imu_reading * transform;
  imu_reading = o_imu_reading.getRotation();

  // Acquire Quaternion that is the difference between the two readings
  tf::Quaternion compass_yaw = tf::createQuaternionFromRPY(0.0, 0.0, compass_heading);
  tf::Quaternion diff_yaw = tf::createQuaternionFromRPY(0.0, 0.0, compass_heading - tf::getYaw(imu_reading));

  // Transform the imu reading by the difference
  tf::Quaternion new_quaternion = diff_yaw * imu_reading;

  // Transform the imu reading back into imu_link
  sensor_msgs::Imu publish_imu;
  o_imu_reading = tf::Transform(new_quaternion, tf::Vector3(0, 0, 0));
  o_imu_reading = o_imu_reading * (transform.inverse());
  tf::quaternionTFToMsg(o_imu_reading.getRotation(), curr_imu_reading_->orientation);

  // Publish all data
  std_msgs::Float32 curr_heading_float;
  curr_heading_float.data = compass_heading;
  compass_pub_.publish(curr_heading_float);
  imu_pub_.publish(curr_imu_reading_);
}

void IMUCompass::initFilter(double heading_meas) {
  curr_heading_ = heading_meas;
  curr_heading_variance_ = 1; // not very sure
  filter_initialized_ = true;
  ROS_INFO("Magnetometer data received. Compass estimator initialized");
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "imu_compass_asl");
  ros::NodeHandle node;
  IMUCompass imu_heading_estimator(node);
  ros::spin();
  return 0;
}
