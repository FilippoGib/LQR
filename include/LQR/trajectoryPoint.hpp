#pragma once

#include <pcl/point_types.h>
#include <pcl/register_point_struct.h>

struct EIGEN_ALIGN16 TrajectoryPoint
{
  double s_m;
  double x_m;
  double y_m;
  double psi_rad;
  double kappa_radpm;
  double vx_mps;
  double ax_mps2;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW  // Ensures proper alignment
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT(TrajectoryPoint,

  (double, s_m, s_m)
  (double, x_m, x_m)
  (double, y_m, y_m)
  (double, psi_rad, psi_rad)
  (double, kappa_radpm, kappa_radpm)
  (double, vx_mps, vx_mps)
  (double, ax_mps2, ax_mps2)
)

