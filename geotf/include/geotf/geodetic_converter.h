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
  void initFromRosParam(const std::string& prefix = "/geotf") {
    GeodeticConverter converter;
    ros::NodeHandle nh;
    if (!nh.ok() || !nh.hasParam(prefix)) {
      ROS_WARN("[GeoTF] No GeodeticTF Transformations found.");
      return;
    }
    XmlRpc::XmlRpcValue yaml_raw_data;
    nh.getParam("/geotf", yaml_raw_data);

    XmlRpc::XmlRpcValue& frame_definitions = yaml_raw_data["Frames"];
    for (auto it = frame_definitions.begin();
         it != frame_definitions.end(); ++it) {

      const std::string frame_name = it->first;

      XmlRpc::XmlRpcValue& xmlnode = it->second;
      std::string frame_type = xmlnode["Type"];
      if (frame_type == "EPSGCode") {
        if (!xmlnode.hasMember("Code")) {
          ROS_WARN_STREAM("[GeoTF] Ignoring frame " << frame_type
                                                    << ": EPSGCode needs Code setting.");
          continue;
        }
        int code = xmlnode["Code"];
        addFrameByEPSG(frame_name, code);

      } else if (frame_type == "GCSCode") {
        if (!xmlnode.hasMember("Code")) {
          ROS_WARN_STREAM("[GeoTF] Ignoring frame " << frame_type
                                                    << ": GCSCode needs Code setting.");
          continue;
        }
        std::string code = xmlnode["Code"];
        addFrameByGCSCode(frame_name, code);

      } else if (frame_type == "UTM") {
        if (!xmlnode.hasMember("Zone") || !xmlnode.hasMember("Hemisphere")) {
          ROS_WARN_STREAM("[GeoTF] Ignoring frame " << frame_type
                                                    << ": UTM needs Zone and Hemisphere setting.");
          continue;
        }
        int zone = xmlnode["Zone"];
        bool hemisphere = xmlnode["Hemisphere"] == "N";
        addFramebyUTM(frame_name, zone, hemisphere);

      } else if (frame_type == "ENUOrigin") {
        if (!xmlnode.hasMember("LatOrigin") || !xmlnode.hasMember("LonOrigin")
            || !xmlnode.hasMember("AltOrigin")) {
          ROS_WARN_STREAM("[GeoTF] Ignoring frame " << frame_type
                                                    << ": ENUOrigin needs LatOrigin, LonOrigin and AltOrigin setting.");
          continue;
        }
        double latOrigin = xmlnode["LatOrigin"];
        double lonOrigin = xmlnode["LonOrigin"];
        double altOrigin = xmlnode["AltOrigin"];

        addFrameByENUOrigin(frame_name, lonOrigin, latOrigin, altOrigin);

      }

      // Get TF Mapping
      if (yaml_raw_data.hasMember("TF_Mapping")) {
        XmlRpc::XmlRpcValue& tf_mapping = yaml_raw_data["TF_Mapping"];
        std::string geo_tf = tf_mapping["GEO_TF"];
        std::string tf = tf_mapping["TF"];
        if (mappings_.count(geo_tf)) {
          tf_mapping_ = std::make_pair(geo_tf, tf);
        } else {
          ROS_WARN_STREAM("[GeoTF] Invalid Tf connection, frame " << geo_tf
                                                                  << " not defined.");
        }
        ROS_INFO_STREAM("[GeoTF] TF connection is " << geo_tf << " = " << tf);
      } else {
        ROS_WARN_STREAM("[GeoTF] No TF connection specified.");
      }

    }

    listener_ = std::make_shared<tf::TransformListener>();
    broadcaster_ = std::make_shared<tf::TransformBroadcaster>();

  }

  // Adds a coordinate frame by its EPSG identifier
  // see https://spatialreference.org/ref/epsg/
  // Example: CH1903+ has epsg id 2056
  //          https://spatialreference.org/ref/epsg/2056/
  bool addFrameByEPSG(const std::string& name, const int& id) {
    if (mappings_.count(name)) {
      return false;
    }

    // Create Spatial Reference from well known code
    OGRSpatialReferencePtr
        spatial_ref = std::make_shared<OGRSpatialReference>();
    OGRErr err = spatial_ref->importFromEPSG(id);
    if (err != OGRERR_NONE) {
      std::cout << "ERROR" << err << std::endl;
      return false;
    }

    mappings_[name] = spatial_ref;

    ROS_INFO_STREAM("[GeoTF] Added EPSG Code " << id <<
                                               " as frame " << name);
    return true;
  }

  // Add a coordinate frame by a Geo Coordinate System
  // Prominent example: WGS84
  bool addFrameByGCSCode(const std::string& name, const std::string& gcscode) {
    if (mappings_.count(name)) {
      return false;
    }

    // Create Spatial Reference from well known code
    OGRSpatialReferencePtr
        spatial_ref = std::make_shared<OGRSpatialReference>();
    OGRErr err = spatial_ref->SetWellKnownGeogCS(gcscode.c_str());
    if (err != OGRERR_NONE) {
      std::cout << "ERROR" << err << std::endl;
      return false;
    }

    mappings_[name] = spatial_ref;
    ROS_INFO_STREAM("[GeoTF] Added GCS Code " << gcscode <<
                                              " as frame " << name);
    return true;
  }

  // Add a frame by UTM Zone
  // zone is the UTM zone (e.g. switzerland is in Zone 32)
  // north is true for northern hemisphere zones.
  bool addFramebyUTM(const std::string& name,
                     const uint zone,
                     const bool north) {

    // Create Spatial Reference from well known code
    OGRSpatialReferencePtr
        spatial_ref = std::make_shared<OGRSpatialReference>();

    spatial_ref->SetWellKnownGeogCS("WGS84");
    spatial_ref->SetUTM(zone, north);

    mappings_[name] = spatial_ref;
    ROS_INFO_STREAM("[GeoTF] Added UTM " << zone << "/"
                                         << (north ? "N" : "S") <<
                                         " as frame " << name);
    return true;
  }

  // Writes a list of all frame definition to console
  void writeDebugInfo() {

    for (auto key : mappings_) {
      std::cout << key.first << std::endl;
      char* pszWKT = NULL;
      key.second->exportToWkt(&pszWKT);
      printf("%s\n", pszWKT);
      CPLFree(pszWKT);

      std::cout << std::endl << std::endl;
    }
  }

  // Creates a new ENU Frame with its origin at the given
  // Location (lon,lat, alt)
  // Where (lon,lat,alt) are defined w.r.t. WGS84
  bool addFrameByENUOrigin(const std::string& name,
                           double lon,
                           double lat,
                           double alt) {
    // Create Spatial Reference from well known code
    OGRSpatialReferencePtr
        spatial_ref = std::make_shared<OGRSpatialReference>();

    // ENU Frame based on GPS coordinates
    spatial_ref->SetWellKnownGeogCS("WGS84");
    spatial_ref->SetOrthographic(lat, lon, 0.0, 0.0);

    altitude_offsets_[name] = -alt;

    ROS_INFO_STREAM(
        "[GeoTF] Added ENUOrigin " << lon << "/" << lat << "/" << alt <<
                                   " as frame " << name);
    mappings_[name] = spatial_ref;
    return true;
  }

  bool addFrameByWKT(const std::string& name, const std::string& wktformat) {
    // Create Spatial Reference from well known code
    OGRSpatialReferencePtr
        spatial_ref = std::make_shared<OGRSpatialReference>();

    std::vector<char> mutable_cstr
        (wktformat.c_str(), wktformat.c_str() + wktformat.size() + 1);
    char* data = mutable_cstr.data();

    spatial_ref->importFromWkt(&data);
    mappings_[name] = spatial_ref;

  }

  // Checks if two geo frames can be converted
  bool canConvert(const std::string& input_frame,
                  const std::string& output_frame) {
    return checkTransform(input_frame, output_frame);
  }

  // Converts Pose from one input_frame to output_frame
  // Both frames are assumed to be geoframes
  // Currently, Attitude is not adjusted.
  bool convert(const std::string& input_frame,
               const Eigen::Affine3d& input,
               const std::string& output_frame,
               Eigen::Affine3d* output) {
    *output = input;
    Eigen::Vector3d translation = input.translation();
    bool result = convert(input_frame,
                          input.translation(),
                          output_frame,
                          &translation);

    if (!result) {
      return false;
    }
    output->translation() = translation;
    return true;
  }

  // Converts Position from one input_frame to output_frame
  // Both frames are assumed to be geoframes
  bool convert(const std::string& input_frame,
               const Eigen::Vector3d& input,
               const std::string& output_frame,
               Eigen::Vector3d* output) {

    OGRCoordinateTransformationPtr transform;

    if (!getTransform(input_frame, output_frame, &transform)) {
      return false;
    }

    //copy data
    *output = input;

    // subtract static offset for input frame if it has one
    if (altitude_offsets_.count(input_frame)) {
      output->z() -= altitude_offsets_[input_frame];
    }

    bool transformed = transform->Transform(1,
                                            output->data(),
                                            output->data() + 1,
                                            output->data() + 2);

    if (!transformed) {
      return false;
    }

    // add static offset for output frame if it has one
    if (altitude_offsets_.count(output_frame)) {
      output->z() += altitude_offsets_[output_frame];
    }

  }

  // Convets a Pose in a geoframe to a pose in a tf frame
  bool convertToTf(const std::string& geo_input_frame,
                   const Eigen::Affine3d& input,
                   const std::string& tf_output_frame,
                   Eigen::Affine3d* output,
                   const ros::Time& time = ros::Time(0.0)) {
    std::string tf_connection_frame = tf_mapping_->second;
    std::string geotf_connection_frame = tf_mapping_->first;
    Eigen::Affine3d tf_connection_value;

    // Convert from whatever geo frame to geotf_connection_frame
    bool result = convert(geo_input_frame,
                          input,
                          geotf_connection_frame,
                          &tf_connection_value);

    if (!result) { return false; }

    //Convert from tf_connection_frame to tf_output_frame
    if (!listener_->canTransform(tf_output_frame, tf_connection_frame, time)) {
      return false;
    }

    tf::StampedTransform tf_T_O_C; // transform connection to output
    Eigen::Affine3d eigen_T_O_C;
    try {
      listener_->lookupTransform(tf_output_frame,
                                 tf_connection_frame,
                                 time, tf_T_O_C);
    } catch (std::exception& ex) {
      ROS_WARN_STREAM("[GeoTF] Error in tf connection" << ex.what());
      return false;
    }

    tf::transformTFToEigen(tf_T_O_C, eigen_T_O_C);

    *output = eigen_T_O_C * tf_connection_value;
    return true;
  }

  // Publishes a geolocation as a tf frame
  void publishAsTf(const std::string& geo_input_frame,
                   const Eigen::Vector3d& input,
                   const std::string& frame_name) {
    Eigen::Affine3d affine(Eigen::Affine3d::Identity());
    affine.translation() = input;
    publishAsTf(geo_input_frame, affine, frame_name);
  }

  // Publishes a geolocation as a tf frame
  void publishAsTf(const std::string& geo_input_frame,
                   const Eigen::Affine3d& input,
                   const std::string& frame_name) {
    std::string tf_connection_frame = tf_mapping_->second;
    std::string geotf_connection_frame = tf_mapping_->first;

    Eigen::Affine3d input_connection;
    bool result = convert(geo_input_frame,
                          input,
                          geotf_connection_frame,
                          &input_connection);

    if (!result) {
      std::cout << "error" << geo_input_frame << "->" << geotf_connection_frame
                << std::endl;
      return;
    }
    tf::StampedTransform tf_input;
    tf::transformEigenToTF(input_connection, tf_input);
    tf_input.stamp_ = ros::Time::now();
    tf_input.frame_id_ = tf_connection_frame;
    tf_input.child_frame_id_ = frame_name;
    broadcaster_->sendTransform(tf_input);
    std::cout << tf_input.child_frame_id_ << std::endl;
  }

  // Convets a Pose in a TF to a pose in a Geo frame
  bool convertFromTf(const std::string& tf_input_frame,
                     const Eigen::Affine3d& input,
                     const std::string& geo_output_frame,
                     Eigen::Affine3d* output,
                     const ros::Time& time = ros::Time(0.0)) {
    Eigen::Affine3d tf_connection_value;
    //convert from tf input to
    std::string tf_connection_frame = tf_mapping_->second;
    std::string geotf_connection_frame = tf_mapping_->first;

    //Convert from tf_input_frame  to tf_connection_frame
    if (!listener_->canTransform(tf_connection_frame, tf_input_frame, time)) {
      return false;
    }


    // add exception handling etc.
    tf::StampedTransform tf_T_C_I; // transform input to connection
    Eigen::Affine3d eigen_T_C_I;

    try {
      listener_->lookupTransform(tf_connection_frame,
                                 tf_input_frame,
                                 time, tf_T_C_I);
    } catch (std::exception& ex) {
      ROS_WARN_STREAM("[GeoTF] Error in tf connection" << ex.what());
      return false;
    }
    tf::transformTFToEigen(tf_T_C_I, eigen_T_C_I);

    tf_connection_value = eigen_T_C_I * input;

    // convert from corresponding
    return convert(geotf_connection_frame,
                   tf_connection_value,
                   geo_output_frame,
                   output);

  }

 private:
  bool getTransform(const std::string& input_frame,
                    const std::string& output_frame,
                    OGRCoordinateTransformationPtr* transform) {
    if (!checkTransform(input_frame, output_frame)) {
      return false;
    }

    *transform = transforms_[std::make_pair(input_frame, output_frame)];
    return true;
  }

  bool checkTransform(const std::string& input_frame,
                      const std::string& output_frame) {
    TransformId tf_id = std::make_pair(input_frame, output_frame);

    // Check if we already cached transform
    if (transforms_.count(tf_id)) {
      return true;
    }

    // check if we have both frames defined
    if (mappings_.count(tf_id.first) + mappings_.count(tf_id.second) != 2) {
      return false;
    }

    // Create transform
    OGRCoordinateTransformationPtr transform;
    transform.reset(OGRCreateCoordinateTransformation(mappings_[tf_id.first].get(),
                                                      mappings_[tf_id.second].get()));

    // if invalid
    if (transform.get() == nullptr) {
      return false;
    }

    // If all goes well
    transforms_[tf_id] = transform;
    return true;
  }

  // Coordinate frame definitions
  std::map<std::string, OGRSpatialReferencePtr> mappings_;

  // Cordinate frame transformations
  std::map<TransformId, OGRCoordinateTransformationPtr> transforms_;

  // Altitude offsets for frames to convert to WGS84
  std::map<std::string, double> altitude_offsets_;

  //first = geotf frame, second = tf frame
  // Defines that these two frames (on a geo frame, on a tf frame)
  // are equal an can be used for geo<->TF conversions
  // Note: Geoframe must be a cartesian frame.
  boost::optional<std::pair<std::string, std::string>> tf_mapping_;

  std::shared_ptr<tf::TransformListener> listener_;
  std::shared_ptr<tf::TransformBroadcaster> broadcaster_;

};
}
#endif //GDAL_ROS_GEODETIC_CONVERTER_H
