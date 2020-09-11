#include <geotf/geodetic_converter.h>
#include <numpy_eigen/boost_python_headers.hpp>

using gc = geotf::GeodeticConverter;

// Thin wrapper for exposing default argument
void initFromRosParamNoArgs(gc &geodetic_converter) {
  geodetic_converter.initFromRosParam();
}

void initFromRosParamStr(gc &geodetic_converter, const std::string &prefix) {
  geodetic_converter.initFromRosParam(prefix);
}

Eigen::Matrix4d convertFromTfDefault(gc &geodetic_converter,
                                     const std::string &geo_input_frame,
                                     const Eigen::Matrix4d &input,
                                     const std::string &tf_output_frame) {
  Eigen::Affine3d input_aff, output_aff;
  Eigen::Matrix4d output;
  // Convert Matrix to Affine3d
  input_aff = input;

  if (!geodetic_converter.convertFromTf(geo_input_frame, input_aff,
                                        tf_output_frame, &output_aff)) {
    std::cout << "something fucked up" << std::endl;
  }

  // Convert output back to matrix form
  output = output_aff.matrix();
  return output;
}

Eigen::Matrix4d convertFromTfTime(gc &geodetic_converter,
                                  const std::string &geo_input_frame,
                                  const Eigen::Matrix4d &input,
                                  const std::string &tf_output_frame,
                                  const ros::Time &time) {
  Eigen::Affine3d input_aff, output_aff;
  input_aff = input;
  geodetic_converter.convertFromTf(geo_input_frame, input_aff, tf_output_frame,
                                   &output_aff, time);
  return output_aff.matrix();
}

// Wrapper for overloaded functions (workaround for output pointer)
Eigen::Vector3d convertVec(gc &geodetic_converter,
                           const std::string &input_frame,
                           const Eigen::Vector3d &input,
                           const std::string &output_frame) {
  Eigen::Vector3d output;
  geodetic_converter.convert(input_frame, input, output_frame, &output);
  return output;
}

Eigen::Matrix4d convertAff(gc &geodetic_converter,
                           const std::string &input_frame,
                           const Eigen::Matrix4d &input,
                           const std::string &output_frame) {
  Eigen::Affine3d input_aff, output_aff;
  input_aff = input;
  geodetic_converter.convert(input_frame, input_aff, output_frame, &output_aff);
  return output_aff.matrix();
}

bool publishAsTfAff(gc &geodetic_converter, const std::string &geo_input_frame,
                    const Eigen::Matrix4d &input,
                    const std::string &frame_name) {
  Eigen::Affine3d input_aff;
  input_aff = input;

  return geodetic_converter.publishAsTf(geo_input_frame, input_aff, frame_name);
}

Eigen::Matrix4d convertToTfAff(gc &geodetic_converter,
                               const std::string &geo_input_frame,
                               const Eigen::Matrix4d &input,
                               const std::string &tf_output_frame,
                               const ros::Time &time) {
  Eigen::Affine3d input_aff, output_aff;
  input_aff = input;

  geodetic_converter.convertToTf(geo_input_frame, input_aff, tf_output_frame,
                                 &output_aff, time);

  return output_aff.matrix();
}

void exportGeodeticConverter() {
  using namespace boost::python;

  // Create objects for overloaded functions
  bool (gc::*publishAsTfVec)(const std::string &geo_input_frame,
                             const Eigen::Vector3d &input,
                             const std::string &frame_name) = &gc::publishAsTf;

  class_<gc, boost::shared_ptr<gc>>("GeodeticConverter", init<>())
      .def("initFromRosParam", initFromRosParamNoArgs)
      .def("initFromRosParam", initFromRosParamStr)
      .def("addFrameByEPSG", &gc::addFrameByEPSG)
      .def("addFrameByUTM", &gc::addFrameByUTM)
      .def("addFrameByGCSCode", &gc::addFrameByGCSCode)
      .def("addFrameByENUOrigin", &gc::addFrameByENUOrigin)
      .def("addFrameByWKT", &gc::addFrameByWKT)
      .def("removeFrame", &gc::removeFrame)
      .def("hasFrame", &gc::hasFrame)
      .def("canConvert", &gc::canConvert)
      .def("convert", convertAff)
      .def("convert", convertVec)
      .def("convertToTf", convertToTfAff)
      .def("convertFromTf", convertFromTfDefault)
      .def("convertFromTf", convertFromTfTime)
      .def("publishAffAsTf", publishAsTfAff)
      .def("publishVecAsTf", publishAsTfVec)
      .def("writeDebugInfo", &gc::writeDebugInfo);
}
