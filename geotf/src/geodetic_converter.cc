#include <geotf/geodetic_converter.h>
namespace geotf {
// Initialize frame definitions from rosparams
void GeodeticConverter::initFromRosParam(const std::string& prefix) {
  GeodeticConverter converter;
  ros::NodeHandle nh;
  if (!nh.ok() || !nh.hasParam(prefix)) {
    ROS_WARN("[GeoTF] No GeodeticTF Transformations found.");
    return;
  }
  XmlRpc::XmlRpcValue yaml_raw_data;
  nh.getParam("/geotf", yaml_raw_data);

  auto& frame_definitions = yaml_raw_data["Frames"];
  for (auto it = frame_definitions.begin();
       it != frame_definitions.end(); ++it) {

    const std::string frame_name = it->first;
    auto& xmlnode = it->second;

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

      addFrameByENUOrigin(frame_name, latOrigin, lonOrigin, altOrigin);
    }
  }

  // Get TF Mapping
  if (yaml_raw_data.hasMember("TF_Mapping")) {
    auto& tf_mapping = yaml_raw_data["TF_Mapping"];
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

  listener_ = std::make_shared<tf::TransformListener>();
  broadcaster_ = std::make_shared<tf::TransformBroadcaster>();
}

// Adds a coordinate frame by its EPSG identifier
// see https://spatialreference.org/ref/epsg/
// Example: CH1903+ has epsg id 2056
//          https://spatialreference.org/ref/epsg/2056/
bool GeodeticConverter::addFrameByEPSG(const std::string& name, const int& id) {
  if (mappings_.count(name)) {
    return false;
  }

  // Create Spatial Reference from EPSG ID
  auto spatial_ref = std::make_shared<OGRSpatialReference>();
  OGRErr err = spatial_ref->importFromEPSG(id);
  if (err != OGRERR_NONE) {
    std::cout << "ERROR" << err << std::endl;
    return false;
  }

  mappings_.insert(std::make_pair(name, spatial_ref));

  ROS_INFO_STREAM("[GeoTF] Added EPSG Code " << id <<
                                             " as frame " << name);
  return true;
}

// Add a coordinate frame by a Geo Coordinate System
// Prominent example: WGS84
bool GeodeticConverter::addFrameByGCSCode(const std::string& name,
                                          const std::string& gcscode) {
  if (mappings_.count(name)) {
    return false;
  }

  // Create Spatial Reference from well known code
  auto spatial_ref = std::make_shared<OGRSpatialReference>();
  OGRErr err = spatial_ref->SetWellKnownGeogCS(gcscode.c_str());
  if (err != OGRERR_NONE) {
    std::cout << "ERROR" << err << std::endl;
    return false;
  }

  mappings_.insert(std::make_pair(name, spatial_ref));
  ROS_INFO_STREAM("[GeoTF] Added GCS Code " << gcscode <<
                                            " as frame " << name);
  return true;
}

// Add a frame by UTM Zone
// zone is the UTM zone (e.g. switzerland is in Zone 32)
// north is true for northern hemisphere zones.
bool GeodeticConverter::addFramebyUTM(const std::string& name,
                                      const uint zone,
                                      const bool north) {

  // Create Spatial Reference from UTM Zone
  auto spatial_ref = std::make_shared<OGRSpatialReference>();

  spatial_ref->SetWellKnownGeogCS("WGS84");
  spatial_ref->SetUTM(zone, north);

  mappings_.insert(std::make_pair(name, spatial_ref));
  ROS_INFO_STREAM("[GeoTF] Added UTM " << zone << "/"
                                       << (north ? "N" : "S") <<
                                       " as frame " << name);
  return true;
}

// Writes a list of all frame definition to console
void GeodeticConverter::writeDebugInfo() const {

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
// Location (lat, lon, alt)
// Where (lon,lat,alt) are defined w.r.t. WGS84
bool GeodeticConverter::addFrameByENUOrigin(const std::string& name,
                                            const double lat,
                                            const double lon,
                                            const double alt) {
  // Create Spatial Reference from ENU origin
  auto spatial_ref = std::make_shared<OGRSpatialReference>();

  // ENU Frame based on GPS coordinates
  spatial_ref->SetWellKnownGeogCS("WGS84");
  spatial_ref->SetOrthographic(lat, lon, 0.0, 0.0);

  altitude_offsets_.insert(std::make_pair(name, alt));

  ROS_INFO_STREAM(
      "[GeoTF] Added ENUOrigin " << lat << "/" << lon << "/" << alt <<
                                 " as frame " << name);
  mappings_.insert(std::make_pair(name, spatial_ref));
  return true;
}

bool GeodeticConverter::addFrameByWKT(const std::string& name,
                                      const std::string& wktformat) {
  // Create Spatial Reference from well known code
  auto spatial_ref = std::make_shared<OGRSpatialReference>();

  std::vector<char> mutable_cstr
      (wktformat.c_str(), wktformat.c_str() + wktformat.size() + 1);
  char* data = mutable_cstr.data();

  spatial_ref->importFromWkt(&data);
  mappings_.insert(std::make_pair(name, spatial_ref));
}

// Checks if two geo frames can be converted
bool GeodeticConverter::canConvert(const std::string& input_frame,
                                   const std::string& output_frame) {
  return checkTransform(input_frame, output_frame);
}

// Converts Pose from one input_frame to output_frame
// Both frames are assumed to be geoframes
// Currently, Attitude is not adjusted.
bool GeodeticConverter::convert(const std::string& input_frame,
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
bool GeodeticConverter::convert(const std::string& input_frame,
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
    output->z() += altitude_offsets_.at(input_frame);
  }

  // if input system is a geographic coordinate system, switch x and y.
  // We assume that IsGeographic is true for non-ENU systems
  // GDAL default is x = lon, y = lat, but we want it the other way around
  // GDAL default for enu is x=e, y= n, which we do not want to switch
  if (transform->GetSourceCS()->IsGeographic()) {
    std::swap(output->x(), output->y());
  }

  bool transformed = transform->Transform(1,
                                          output->data(),
                                          output->data() + 1,
                                          output->data() + 2);

  if (!transformed) {
    return false;
  }

  // reverse switch if necessary
  if (transform->GetTargetCS()->IsGeographic()) {
    std::swap(output->x(), output->y());
  }
  // add static offset for output frame if it has one
  if (altitude_offsets_.count(output_frame)) {
    output->z() -= altitude_offsets_.at(output_frame);
  }

  return true;
}

// Converts a Pose in a geoframe to a pose in a tf frame
bool GeodeticConverter::convertToTf(const std::string& geo_input_frame,
                                    const Eigen::Affine3d& input,
                                    const std::string& tf_output_frame,
                                    Eigen::Affine3d* output,
                                    const ros::Time& time) {
  if (!tf_mapping_) {
    ROS_WARN("[GeoTf] No TF mapping defined, canceling convertToTf");
    return false;
  }

  std::string tf_connection_frame = tf_mapping_->second;
  std::string geotf_connection_frame = tf_mapping_->first;
  Eigen::Affine3d tf_connection_value;

  // Convert from whatever geo frame to geotf_connection_frame
  bool result = convert(geo_input_frame,
                        input,
                        geotf_connection_frame,
                        &tf_connection_value);

  if (!result) { return false; }

  // Convert from tf_connection_frame to tf_output_frame.
  if (!listener_->canTransform(tf_output_frame, tf_connection_frame, time)) {
    return false;
  }

  tf::StampedTransform tf_T_O_C; // transform connection to output.
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
bool GeodeticConverter::publishAsTf(const std::string& geo_input_frame,
                                    const Eigen::Vector3d& input,
                                    const std::string& frame_name) {
  Eigen::Affine3d affine(Eigen::Affine3d::Identity());
  affine.translation() = input;
  return publishAsTf(geo_input_frame, affine, frame_name);
}

// Publishes a geolocation as a tf frame
bool GeodeticConverter::publishAsTf(const std::string& geo_input_frame,
                                    const Eigen::Affine3d& input,
                                    const std::string& frame_name) {
  if (!tf_mapping_) {
    ROS_WARN("[GeoTf] No TF mapping defined, canceling convertAsTf");
    return false;
  }

  std::string tf_connection_frame = tf_mapping_->second;
  std::string geotf_connection_frame = tf_mapping_->first;

  Eigen::Affine3d input_connection;
  bool result = convert(geo_input_frame,
                        input,
                        geotf_connection_frame,
                        &input_connection);

  if (!result) {
    return false;
  }

  tf::StampedTransform tf_input;
  tf::transformEigenToTF(input_connection, tf_input);
  tf_input.stamp_ = ros::Time::now();
  tf_input.frame_id_ = tf_connection_frame;
  tf_input.child_frame_id_ = frame_name;
  broadcaster_->sendTransform(tf_input);
  return true;
}

// Convets a Pose in a TF to a pose in a Geo frame
bool GeodeticConverter::convertFromTf(const std::string& tf_input_frame,
                                      const Eigen::Affine3d& input,
                                      const std::string& geo_output_frame,
                                      Eigen::Affine3d* output,
                                      const ros::Time& time) {
  if (!tf_mapping_) {
    ROS_WARN("[GeoTf] No TF mapping defined, canceling convertFromTf");
    return false;
  }

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

bool GeodeticConverter::getTransform(const std::string& input_frame,
                                     const std::string& output_frame,
                                     OGRCoordinateTransformationPtr* transform) {
  if (!checkTransform(input_frame, output_frame)) {
    return false;
  }

  *transform = transforms_.at(std::make_pair(input_frame, output_frame));
  return true;
}

bool GeodeticConverter::checkTransform(const std::string& input_frame,
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
  OGRCoordinateTransformationPtr transform
      (OGRCreateCoordinateTransformation(mappings_.at(tf_id.first).get(),
                                         mappings_.at(tf_id.second).get()));

  // if invalid
  if (transform.get() == nullptr) {
    return false;
  }

  // If all goes well
  transforms_.insert(std::make_pair(tf_id, transform));
  return true;
}
}