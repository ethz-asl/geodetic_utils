#ifndef GEODETIC_CONV_H_
#define GEODETIC_CONV_H_

#include "math.h"
#include <Eigen/Dense>

namespace geodetic_conv {
// Geodetic system parameters
static const double semimajor_axis = 6378137;
static const double semiminor_axis = 6356752.3142;
static const double first_eccentricity_squared = 6.69437999014 * 0.001;
static const double second_eccentricity_squared = 6.73949674228 * 0.001;
static const double flattening = 1 / 298.257223563;

class GeodeticConverter
{
 public:

  // Default initialisation/copy constructor and assignment operator are OK.

  ~GeodeticConverter()
  {
  }

  void initialiseReference(const double lat, const double lon, const double altitude)
  {
    // Save NED origin
    initial_latitude = deg2Rad(lat);
    initial_longitude = deg2Rad(lon);
    initial_height = altitude;

    // Compute ECEF of NED origin
    geodetic2Ecef(lat, lon, altitude, initial_ecef_x_, initial_ecef_y_, initial_ecef_z_);

    // Compute ECEF to NED and NED to ECEF matrices
    double phiP = atan2(initial_ecef_z_, sqrt(pow(initial_ecef_x_, 2) + pow(initial_ecef_y_, 2)));

    ecef_to_ned_matrix_ = nRe(phiP, initial_longitude);
    ned_to_ecef_matrix_ = nRe(initial_latitude, initial_longitude).transpose();
  }

  void geodetic2Ecef(const double lat, const double lon, const double altitude, double& x,
                     double& y, double& z)
  {
    // Convert geodetic coordinates to ECEF.
    // http://code.google.com/p/pysatel/source/browse/trunk/coord.py?r=22
    double lat_rad = deg2Rad(lat);
    double lon_rad = deg2Rad(lon);
    double xi = sqrt(1 - first_eccentricity_squared * sin(lat_rad) * sin(lat_rad));
    x = (semimajor_axis / xi + altitude) * cos(lat_rad) * cos(lon_rad);
    y = (semimajor_axis / xi + altitude) * cos(lat_rad) * sin(lon_rad);
    z = (semimajor_axis / xi * (1 - first_eccentricity_squared) + altitude) * sin(lat_rad);
  }

  void ecef2Geodetic(const double x, const double y, const double z, double& lat, double& lon,
                     double& altitude)
  {
    // Convert ECEF coordinates to geodetic coordinates.
    // J. Zhu, "Conversion of Earth-centered Earth-fixed coordinates
    // to geodetic coordinates," IEEE Transactions on Aerospace and
    // Electronic Systems, vol. 30, pp. 957-961, 1994.

    double r = sqrt(x * x + y * y);
    double Esq = semimajor_axis * semimajor_axis - semiminor_axis * semiminor_axis;
    double F = 54 * semiminor_axis * semiminor_axis * z * z;
    double G = r * r + (1 - first_eccentricity_squared) * z * z - first_eccentricity_squared * Esq;
    double C = (first_eccentricity_squared * first_eccentricity_squared * F * r * r) / pow(G, 3);
    double S = cbrt(1 + C + sqrt(C * C + 2 * C));
    double P = F / (3 * pow((S + 1 / S + 1), 2) * G * G);
    double Q = sqrt(1 + 2 * first_eccentricity_squared * first_eccentricity_squared * P);
    double r_0 = -(P * first_eccentricity_squared * r) / (1 + Q)
        + sqrt(
            0.5 * semimajor_axis * semimajor_axis * (1 + 1.0 / Q)
                - P * (1 - first_eccentricity_squared) * z * z / (Q * (1 + Q)) - 0.5 * P * r * r);
    double U = sqrt(pow((r - first_eccentricity_squared * r_0), 2) + z * z);
    double V = sqrt(
        pow((r - first_eccentricity_squared * r_0), 2) + (1 - first_eccentricity_squared) * z * z);
    double Z_0 = semiminor_axis * semiminor_axis * z / (semimajor_axis * V);
    altitude = U * (1 - semiminor_axis * semiminor_axis / (semimajor_axis * V));
    lat = rad2Deg(atan((z + second_eccentricity_squared * Z_0) / r));
    lon = rad2Deg(atan2(y, x));
  }

  void ecef2Ned(const double x, const double y, const double z, double& north, double& east,
                double& down)
  {
    // Converts ECEF coordinate position into local-tangent-plane NED.
    // Coordinates relative to given ECEF coordinate frame.

    Eigen::Vector3d vect, ret;
    vect(0) = x - initial_ecef_x_;
    vect(1) = y - initial_ecef_y_;
    vect(2) = z - initial_ecef_z_;
    ret = ecef_to_ned_matrix_ * vect;
    north = ret(0);
    east = ret(1);
    down = -ret(2);
  }

  void ned2Ecef(const double north, const double east, const double down, double& x, double& y,
                double& z)
  {
    // NED (north/east/down) to ECEF coordinates
    Eigen::Vector3d ned, ret;
    ned(0) = north;
    ned(1) = east;
    ned(2) = -down;
    ret = ned_to_ecef_matrix_ * ned;
    x = ret(0) + initial_ecef_x_;
    y = ret(1) + initial_ecef_y_;
    z = ret(2) + initial_ecef_z_;
  }

  void geodetic2Ned(const double lat, const double lon, const double altitude, double& north,
                    double& east, double& down)
  {
    // Geodetic position to local NED frame
    double x, y, z;
    geodetic2Ecef(lat, lon, altitude, x, y, z);
    ecef2Ned(x, y, z, north, east, down);
  }

  void ned2Geodetic(const double north, const double east, const double down, double& lat,
                    double& lon, double& altitude)
  {
    // Local NED position to geodetic coordinates
    double x, y, z;
    ned2Ecef(north, east, down, x, y, z);
    ecef2Geodetic(x, y, z, lat, lon, altitude);
  }

  void geodetic2Enu(const double lat, const double lon, const double altitude, double& east,
                    double& north, double& up)
  {
    // Geodetic position to local ENU frame
    double x, y, z;
    geodetic2Ecef(lat, lon, altitude, x, y, z);

    double aux_north, aux_east, aux_down;
    ecef2Ned(x, y, z, aux_north, aux_east, aux_down);

    east = aux_east;
    north = aux_north;
    up = -aux_down;
  }

  void enu2Geodetic(const double east, const double north, const double up, double& lat,
                    double& lon, double& altitude)
  {
    // Local ENU position to geodetic coordinates

    const double aux_north = north;
    const double aux_east = east;
    const double aux_down = -up;
    double x, y, z;
    ned2Ecef(aux_north, aux_east, aux_down, x, y, z);
    ecef2Geodetic(x, y, z, lat, lon, altitude);
  }

  double initial_latitude;
  double initial_longitude;
  double initial_height;

 private:
  inline Eigen::Matrix3d nRe(const double lat_radians, const double lon_radians)
  {
    const double sLat = sin(lat_radians);
    const double sLon = sin(lon_radians);
    const double cLat = cos(lat_radians);
    const double cLon = cos(lon_radians);

    Eigen::Matrix3d ret;
    ret(0, 0) = -sLat * cLon;
    ret(0, 1) = -sLat * sLon;
    ret(0, 2) = cLat;
    ret(1, 0) = -sLon;
    ret(1, 1) = cLon;
    ret(1, 2) = 0.0;
    ret(2, 0) = cLat * cLon;
    ret(2, 1) = cLat * sLon;
    ret(2, 2) = sLat;

    return ret;
  }

  inline
  double rad2Deg(const double radians)
  {
    return (radians / M_PI) * 180.0;
  }

  inline
  double deg2Rad(const double degrees)
  {
    return (degrees / 180.0) * M_PI;
  }

  double initial_ecef_x_;
  double initial_ecef_y_;
  double initial_ecef_z_;
  Eigen::Matrix3d ecef_to_ned_matrix_;
  Eigen::Matrix3d ned_to_ecef_matrix_;

}; // class GeodeticConverter

}; // namespace geodetic_conv

#endif // GEODETIC_CONV_H_
