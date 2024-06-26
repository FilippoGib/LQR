#pragma once

#include <pcl/point_types.h>
#include <pcl/register_point_struct.h>
#include <pcl/kdtree/kdtree_flann.h>

struct EIGEN_ALIGN16 TrajectoryPoint
{
  //double s_m; //distance ran along the reference #we don't need at the moment
  double x_m; //x position
  double y_m; //y position
  double psi_rad; //yaw in radiants
  //double kappa_radpm; #we don't need it at the moment
  //double vx_mps; #we don't need it at the moment
  //double ax_mps2; #we don't need it at the moment

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW  // Ensures proper alignment
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT(TrajectoryPoint,

  //(double, s_m, s_m)
  (double, x_m, x_m)
  (double, y_m, y_m)
  (double, psi_rad, psi_rad)
  // (double, kappa_radpm, kappa_radpm)
  // (double, vx_mps, vx_mps)
  // (double, ax_mps2, ax_mps2)
)

