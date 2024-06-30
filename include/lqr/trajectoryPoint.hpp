#ifndef TRAJECTORYPOINT_H
#define TRAJECTORYPOINT_H
#define PCL_NO_PRECOMPILE
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/memory.h>
#include <pcl/pcl_macros.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/register_point_struct.h>

struct EIGEN_ALIGN16 TrajectoryPoint
{
  //float s_m; //distance ran along the reference #we don't need at the moment
  PCL_ADD_POINT4D;
  float psi_rad; //yaw in radiants
  //float kappa_radpm; #we don't need it at the moment
  //float vx_mps; #we don't need it at the moment
  //float ax_mps2; #we don't need it at the moment

  PCL_MAKE_ALIGNED_OPERATOR_NEW  // Ensures proper alignment
};

POINT_CLOUD_REGISTER_POINT_STRUCT(TrajectoryPoint,

  //(float, s_m, s_m)
  (float, x, x)
  (float, y, y)
  (float, z, z)
  (float, psi_rad, psi_rad)
  // (float, kappa_radpm, kappa_radpm)
  // (float, vx_mps, vx_mps)
  // (float, ax_mps2, ax_mps2)
)

//to ensure it can compute the distance between two points
namespace flann {
  template <>
  struct L2_Simple<TrajectoryPoint> {
    typedef bool is_kdtree_distance;
    typedef TrajectoryPoint ElementType;
    typedef float ResultType;

    template <typename Iterator1, typename Iterator2>
    ResultType operator()(Iterator1 a, Iterator2 b, size_t size, ResultType = -1) const{
      ResultType result = ResultType();
      for(size_t i = 0; i < size; ++i)
      {
        const ResultType diff = a[i] - b[i];
        result += diff * diff;
      }
      return result;
    }
    ResultType operator()(const TrajectoryPoint& a, const TrajectoryPoint& b, size_t size) const {
      const float dx = a.x - b.x;
      const float dy = a.y -b.y;
      return dx * dx + dy * dy;
    }
  };
}

template class pcl::KdTreeFLANN<TrajectoryPoint, flann::L2_Simple<float> >;

#endif //TRAJECTORYPOINT_H
