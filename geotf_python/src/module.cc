#include <numpy_eigen/boost_python_headers.hpp>
#include <boost/python/suite/indexing/vector_indexing_suite.hpp>
using namespace boost::python;

void exportGeodeticConverter();

BOOST_PYTHON_MODULE(libgeotf_python)
{
  exportGeodeticConverter();
}