#pragma once

#include <geometry_msgs/Quaternion.h>
#include <tf/tf.h>

namespace DJISDKGeometry
{
  static constexpr double earth_equatorial_radius = 6378137.0;
  static constexpr double earth_eccentricity = 0.0818;

  static const tf::Matrix3x3 R_ENU2NED(0, 1, 0, 1, 0, 0, 0, 0, -1);
  static const tf::Matrix3x3 R_FLU2FRD(1, 0, 0, 0, -1, 0, 0, 0, -1);

  double wrapTo2Pi(double angle);
  double wrapToPi(double angle);
  
  geometry_msgs::Quaternion RTKYawQuaternion(double rtk_yaw_radians);

  void gpsConvertENU(double &ENU_x, double &ENU_y,
                     double gps_t_lon, double gps_t_lat,
                     double gps_r_lon, double gps_r_lat);

  void GPS2ENU_WGS84(
    double& x_ENU,
    double& y_ENU,
    double lon_GPS,
    double lat_GPS,
    double ref_lon_GPS,
    double ref_lat_GPS
  );
} // namespace DJISDKGeometry
