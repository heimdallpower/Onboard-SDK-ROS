#pragma once

#include <utility>

#define C_PI (double)3.141592653589793

inline constexpr double deg2rad(double deg) { return (deg) * ((C_PI) / (180.0)); }
inline constexpr double rad2deg(double rad) { return (rad) * (180.0) / (C_PI); }

namespace DJISDKGeometry
{
  static constexpr double earth_equatorial_radius{6378137.0};
  static constexpr double earth_eccentricity{0.0818};

  double wrapToPi(double angle);
  double RTKYawMeasurement2ENUYaw(double raw_rtk_yaw_radians);

  void gpsConvertENU(double &ENU_x, double &ENU_y,
                     double gps_t_lon, double gps_t_lat,
                     double gps_r_lon, double gps_r_lat);

  std::pair<double, double> GPS2ENU_WGS84(
    double lon_GPS,
    double lat_GPS,
    double ref_lon_GPS,
    double ref_lat_GPS
  );

} // namespace DJISDKGeometry
