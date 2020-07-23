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

Eigen::Affine3d convertFromTfDefault(gc &geodetic_converter,
                                     const std::string &geo_input_frame,
                                     const Eigen::Affine3d &input,
                                     const std::string &tf_output_frame) {
  Eigen::Affine3d output;
  geodetic_converter.convertToTf(geo_input_frame, input, tf_output_frame,
                                 &output);
  return output;
}

Eigen::Affine3d convertFromTfTime(gc &geodetic_converter,
                                  const std::string &geo_input_frame,
                                  const Eigen::Affine3d &input,
                                  const std::string &tf_output_frame,
                                  const ros::Time &time) {
  Eigen::Affine3d output;
  geodetic_converter.convertToTf(geo_input_frame, input, tf_output_frame,
                                 &output, time);
  return output;
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

Eigen::Affine3d convertAff(gc &geodetic_converter,
                           const std::string &input_frame,
                           const Eigen::Affine3d &input,
                           const std::string &output_frame) {
  Eigen::Affine3d output;
  geodetic_converter.convert(input_frame, input, output_frame, &output);
  return output;
}

void exportGeodeticConverter() {
  using namespace boost::python;

  // Create objects for overloaded functions
  bool (gc::*publishAsTfVec)(const std::string &geo_input_frame,
                             const Eigen::Vector3d &input,
                             const std::string &frame_name) = &gc::publishAsTf;

  bool (gc::*publishAsTfAff)(const std::string &geo_input_frame,
                             const Eigen::Affine3d &input,
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
      .def("convertToTf", &gc::convertToTf)
      .def("convertFromTf", convertFromTfDefault)
      .def("convertFromTf", convertFromTfTime)
      .def("publishAsTf", publishAsTfAff)
      .def("publishAsTf", publishAsTfVec)
      .def("writeDebugInfo", &gc::writeDebugInfo);
}
