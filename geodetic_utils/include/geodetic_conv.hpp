#ifndef GEODETIC_CONV_H_
#define GEODETIC_CONV_H_

#include "math.h"
#include <Eigen/Dense>

namespace geodetic_conv {
    static const double a = 6378137;
    static const double b = 6356752.3142;
    static const double esq = 6.69437999014 * 0.001;
    static const double e1sq = 6.73949674228 * 0.001;
    static const double f = 1 / 298.257223563;

    class GeodeticConverter {
    public:
        GeodeticConverter( const double lat,
                           const double lon,
                           const double altitude )
        {
            // Save NED origin
            _init_lat = deg2Rad( lat );
            _init_lon = deg2Rad( lon );
            _init_h = altitude;

            // Compute ECEF of NED origin
            geodetic2Ecef( lat, lon, altitude,
                        _init_ecef_x,
                        _init_ecef_y,
                        _init_ecef_z );

            // Compute ECEF to NED and NED to ECEF matrices
            double phiP = atan2( _init_ecef_z, sqrt( pow( _init_ecef_x, 2 ) +
                                                    pow( _init_ecef_y, 2 ) ) );

            _ecef_to_ned_matrix = nRe( phiP, _init_lon );
            _ned_to_ecef_matrix = nRe( _init_lat, _init_lon ).transpose();
        }

        // Default copy constructor and assignment operator are OK.

        ~GeodeticConverter() {}

        void
        geodetic2Ecef( const double lat,
                       const double lon,
                       const double altitude,
                       double& x,
                       double& y,
                       double& z )
        {
            // Convert geodetic coordinates to ECEF.
            // http://code.google.com/p/pysatel/source/browse/trunk/coord.py?r=22
            double lat_rad = deg2Rad( lat );
            double lon_rad = deg2Rad( lon );
            double xi = sqrt(1 - esq * sin( lat_rad ) * sin( lat_rad ) );
            x = ( a / xi + altitude ) * cos( lat_rad ) * cos( lon_rad );
            y = ( a / xi + altitude ) * cos( lat_rad ) * sin( lon_rad );
            z = ( a / xi * ( 1 - esq ) + altitude ) * sin( lat_rad );
        }

        void
        ecef2Geodetic( const double x,
                       const double y,
                       const double z,
                       double& lat,
                       double& lon,
                       double& altitude )
        {
            // Convert ECEF coordinates to geodetic coordinates.
            // J. Zhu, "Conversion of Earth-centered Earth-fixed coordinates
            // to geodetic coordinates," IEEE Transactions on Aerospace and
            // Electronic Systems, vol. 30, pp. 957-961, 1994.

            double r = sqrt( x * x + y * y );
            double Esq = a * a - b * b;
            double F = 54 * b * b * z * z;
            double G = r * r + (1 - esq) * z * z - esq * Esq;
            double C = (esq * esq * F * r * r) / pow( G, 3 );
            double S = cbrt( 1 + C + sqrt( C * C + 2 * C ) );
            double P = F / (3 * pow( (S + 1 / S + 1), 2 ) * G * G);
            double Q = sqrt( 1 + 2 * esq * esq * P );
            double r_0 =  -(P * esq * r) / (1 + Q) + sqrt( 0.5 * a * a * (1 + 1.0 / Q) - P * (1 - esq) * z * z / (Q * (1 + Q)) - 0.5 * P * r * r);
            double U = sqrt( pow( (r - esq * r_0), 2 ) + z * z );
            double V = sqrt( pow( (r - esq * r_0), 2 ) + (1 - esq) * z * z );
            double Z_0 = b * b * z / (a * V);
            altitude = U * (1 - b * b / (a * V));
            lat = rad2Deg( atan( (z + e1sq * Z_0) / r ) );
            lon = rad2Deg( atan2( y, x ) );
        }


        void
        ecef2Ned( const double x,
                  const double y,
                  const double z,
                  double& north,
                  double& east,
                  double& depth )
        {
            // Converts ECEF coordinate position into local-tangent-plane NED.
            // Coordinates relative to given ECEF coordinate frame.

            Eigen::Vector3d vect, ret;
            vect(0) = x - _init_ecef_x;
            vect(1) = y - _init_ecef_y;
            vect(2) = z - _init_ecef_z;
            ret = _ecef_to_ned_matrix * vect;
            north = ret(0);
            east = ret(1);
            depth = -ret(2);
        }


        void
        ned2Ecef( const double north,
                  const double east,
                  const double depth,
                  double& x,
                  double& y,
                  double& z )
        {
            // NED (north/east/down) to ECEF coordinates
            Eigen::Vector3d ned, ret;
            ned(0) = north;
            ned(1) = east;
            ned(2) = -depth;
            ret = _ned_to_ecef_matrix * ned;
            x = ret(0) + _init_ecef_x;
            y = ret(1) + _init_ecef_y;
            z = ret(2) + _init_ecef_z;
        }


        void
        geodetic2Ned( const double lat,
                      const double lon,
                      const double altitude,
                      double& north,
                      double& east,
                      double& depth )
        {
            // Geodetic position to local NED frame
            double x, y, z;
            geodetic2Ecef( lat, lon, altitude,
                        x, y, z );
            ecef2Ned( x, y, z,
                    north, east, depth );
        }


        void
        ned2Geodetic( const double north,
                      const double east,
                      const double depth,
                      double& lat,
                      double& lon,
                      double& altitude )
        {
            // Local NED position to geodetic coordinates
            double x, y, z;
            ned2Ecef( north, east, depth,
                    x, y, z );
            ecef2Geodetic( x, y, z,
                        lat, lon, altitude );
        }


        void
        geodetic2Enu( const double lat,
                      const double lon,
                      const double altitude,
                      double& east,
                      double& north,
                      double& up )
        {
            // Geodetic position to local ENU frame
            double x, y, z;
            geodetic2Ecef( lat, lon, altitude,
                           x, y, z );

            double aux_north, aux_east, aux_depth;
            ecef2Ned( x, y, z,
                      aux_north, aux_east, aux_depth );

            east  =  aux_east;
            north =  aux_north;
            up    = -aux_depth;
        }


        void
        enu2Geodetic( const double east,
                      const double north,
                      const double up,
                      double& lat,
                      double& lon,
                      double& altitude )
        {
            // Local ENU position to geodetic coordinates

            const double aux_north =  north;
            const double aux_east  =  east;
            const double aux_depth = -up;
            double x, y, z;
            ned2Ecef( aux_north, aux_east, aux_depth,
                      x, y, z );
            ecef2Geodetic( x, y, z,
                           lat, lon, altitude );
        }

    private:
        double _init_lat;
        double _init_lon;
        double _init_h;
        double _init_ecef_x;
        double _init_ecef_y;
        double _init_ecef_z;
        Eigen::Matrix3d _ecef_to_ned_matrix;
        Eigen::Matrix3d _ned_to_ecef_matrix;

        inline
        Eigen::Matrix3d
        nRe( const double lat_radians,
             const double lon_radians )
        {
            const double sLat = sin( lat_radians );
            const double sLon = sin( lon_radians );
            const double cLat = cos( lat_radians );
            const double cLon = cos( lon_radians );

            Eigen::Matrix3d ret;
            ret(0, 0) = -sLat*cLon;   ret(0, 1) = -sLat*sLon;   ret(0, 2) = cLat;
            ret(1, 0) = -sLon;        ret(1, 1) =  cLon;        ret(1, 2) = 0.0;
            ret(2, 0) =  cLat*cLon;   ret(2, 1) =  cLat*sLon;   ret(2, 2) = sLat;

            return ret;
        }

        inline
        double
        cbrt( const double x )
        {
            if( x >= 0.0 )
                return pow( x, 1.0/3.0 );
            else
                return -pow( fabs(x), 1.0/3.0 );
        }

        inline
        double
        rad2Deg( const double radians )
        {
            return ( radians / M_PI ) * 180.0;
        }


        inline
        double
        deg2Rad( const double degrees )
        {
            return ( degrees / 180.0 ) * M_PI;
        }

    }; // class GeodeticConverter

}; // namespace geodetic_conv

#endif // GEODETIC_CONV_H_
