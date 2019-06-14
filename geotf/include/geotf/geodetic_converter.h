//
// Created by mpantic on 11.06.19.
//

#ifndef GDAL_ROS_GEODETIC_CONVERTER_H
#define GDAL_ROS_GEODETIC_CONVERTER_H

#include <gdal/ogr_spatialref.h>
#include <gdal/cpl_conv.h>
#include <iostream>
#include <Eigen/Dense>
#include <boost/optional.hpp>
#include <vector>
#include <memory>
#include <map>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <ros/ros.h>
#include <tf_conversions/tf_eigen.h>

namespace geotf {
class GeodeticConverter {
  typedef std::shared_ptr<OGRSpatialReference> OGRSpatialReferencePtr;
  typedef std::shared_ptr<OGRCoordinateTransformation>
      OGRCoordinateTransformationPtr;
  typedef std::pair<std::string, std::string> TransformId;

 public:

  // Initialize frame definitions from rosparams
  void initFromRosParam(const std::string& prefix = "/geotf");

  // Adds a coordinate frame by its EPSG identifier
  // see https://spatialreference.org/ref/epsg/
  // Example: CH1903+ has epsg id 2056
  //          https://spatialreference.org/ref/epsg/2056/
  bool addFrameByEPSG(const std::string& name, const int& id);

  // Add a frame by UTM Zone
  // zone is the UTM zone (e.g. switzerland is in Zone 32)
  // north is true for northern hemisphere zones.
  bool addFramebyUTM(const std::string& name,
                     const uint zone,
                     const bool north);

  // Add a coordinate frame by a Geo Coordinate System
// Prominent example: WGS84
  bool addFrameByGCSCode(const std::string& name,
                         const std::string& gcscode);

  // Creates a new ENU Frame with its origin at the given
  // Location (lon,lat, alt)
  // Where (lon,lat,alt) are defined w.r.t. WGS84
  bool addFrameByENUOrigin(const std::string& name,
                           double lat,
                           double lon,
                           double alt);

  // Creates a new frame based on the Well-Known-Text definition
  // (can be obtained from spatialreference.org for example.)
  bool addFrameByWKT(const std::string& name, const std::string& wktformat);

  // Checks if two geo frames can be converted
  bool canConvert(const std::string& input_frame,
                  const std::string& output_frame);

  // Converts Pose from one input_frame to output_frame
  // Both frames are assumed to be geoframes
  // Currently, Attitude is not adjusted.
  bool convert(const std::string& input_frame,
               const Eigen::Affine3d& input,
               const std::string& output_frame,
               Eigen::Affine3d* output);

  // Converts Position from one input_frame to output_frame
  // Both frames are assumed to be geoframes
  bool convert(const std::string& input_frame,
               const Eigen::Vector3d& input,
               const std::string& output_frame,
               Eigen::Vector3d* output) ;

  // Convets a Pose in a geoframe to a pose in a tf frame
  bool convertToTf(const std::string& geo_input_frame,
                   const Eigen::Affine3d& input,
                   const std::string& tf_output_frame,
                   Eigen::Affine3d* output,
                   const ros::Time& time = ros::Time(0.0)) ;

  // Convets a Pose in a TF to a pose in a Geo frame
  bool convertFromTf(const std::string& tf_input_frame,
                     const Eigen::Affine3d& input,
                     const std::string& geo_output_frame,
                     Eigen::Affine3d* output,
                     const ros::Time& time = ros::Time(0.0)) ;

  // Publishes a geolocation as a tf frame
  bool publishAsTf(const std::string& geo_input_frame,
                   const Eigen::Vector3d& input,
                   const std::string& frame_name) ;

  // Publishes a geolocation as a tf frame
  bool publishAsTf(const std::string& geo_input_frame,
                   const Eigen::Affine3d& input,
                   const std::string& frame_name) ;

  // Writes a list of all frame definition to console
  void writeDebugInfo() const;

 private:
  bool getTransform(const std::string& input_frame,
                    const std::string& output_frame,
                    OGRCoordinateTransformationPtr* transform) ;

  bool checkTransform(const std::string& input_frame,
                      const std::string& output_frame);

  // Coordinate frame definitions
  std::map<const std::string, const OGRSpatialReferencePtr> mappings_;

  // Cordinate frame transformations
  std::map<const TransformId, const OGRCoordinateTransformationPtr> transforms_;

  // Altitude offsets for frames to convert to WGS84
  std::map<const std::string, const double> altitude_offsets_;

  //first = geotf frame, second = tf frame
  // Defines that these two frames (on a geo frame, on a tf frame)
  // are equal an can be used for geo<->TF conversions
  // Note: Geoframe must be a cartesian frame.
  boost::optional< std::pair<std::string, std::string>>  tf_mapping_;

  std::shared_ptr<tf::TransformListener> listener_;
  std::shared_ptr<tf::TransformBroadcaster> broadcaster_;

};
}
#endif //GDAL_ROS_GEODETIC_CONVERTER_H
