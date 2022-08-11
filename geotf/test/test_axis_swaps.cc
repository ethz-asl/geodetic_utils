#include <geotf/geodetic_converter.h>
#include <gtest/gtest.h>

TEST(AxisSwapTest, UTMtoGPS) {
  geotf::GeodeticConverter converter;
  converter.addFrameByUTM("UTM", 32, true);
  converter.addFrameByGCSCode("GPS", "WGS84");

  // values obtained independently from swiss map.
  const Eigen::Vector3d gps_pos(46.34019, 9.15765, 2632.7);
  const Eigen::Vector3d utm_pos(512132, 5131858, 2632.7); // E N U

  Eigen::Vector3d utm_pos_converted;
  converter.convert("GPS", gps_pos, "UTM", &utm_pos_converted);

  // round for pos x and y is valid because we only have the original coordinate
  // to the meter from the map.
  ASSERT_EQ(round(utm_pos.x()), round(utm_pos_converted.x()));
  ASSERT_EQ(round(utm_pos.y()), round(utm_pos_converted.y()));
  ASSERT_EQ(utm_pos.z(), utm_pos_converted.z());
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
