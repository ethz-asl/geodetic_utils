#include <geotf/geodetic_converter.h>
#include <numpy_eigen/boost_python_headers.hpp>

using gc = geotf::GeodeticConverter;

// No idea if this works.. am tired..
void initFromRosParamNoArgs(gc &geodetic_converter) {
  geodetic_converter.initFromRosParam();
}

void initFromRosParamStr(gc &geodetic_converter, const std::string& prefix) {
  geodetic_converter.initFromRosParam(prefix);
}

void exportGeodeticConverter() {
  using namespace boost::python;

  bool (gc::*convert_aff)(const std::string &, const Eigen::Affine3d &,
                          const std::string &, Eigen::Affine3d *) =
      &gc::convert;
  bool (gc::*convert_vec)(const std::string &, const Eigen::Vector3d &,
                          const std::string &, Eigen::Vector3d *) =
      &gc::convert;

  class_<gc, boost::shared_ptr<gc>>("GeodeticConverter", init<>())
      .def("initFromRosParam", initFromRosParamNoArgs)
      .def("initFromRosParam", initFromRosParamStr)
      .def("addFrameByEPSG", &gc::addFrameByEPSG)
      .def("addFramebyUTM", &gc::addFramebyUTM)
      .def("addFrameByGCSCode", &gc::addFrameByGCSCode)
      .def("addFrameByENUOrigin", &gc::addFrameByENUOrigin)
      .def("addFrameByWKT", &gc::addFrameByWKT)
      .def("removeFrame", &gc::removeFrame)
      .def("hasFrame", &gc::hasFrame)
      .def("canConvert", &gc::canConvert)
      .def("convert", convert_aff)
      .def("convert", convert_vec)
      .def("convertToTf", &gc::convertToTf);
}