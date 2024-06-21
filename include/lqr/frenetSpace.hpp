#pragma once

#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <iostream>
#include <vector>
#include <ctime>
#include <cstdlib>
#include "trajectoryPoint.hpp"

class FrenetPoint {
public:
    double s;              // distance covered so far
    double d;              // lateral deviation from the trajectory
    double yaw_dev;        // angular deviation i.e. difference between odometry_yaw and the tangent to the reference
    double d_prime;        // first degree derivative of d -> velocity of lateral deviation
    double yaw_dev_prime;  // first derivative of yaw_dev -> velocity of angular deviation, we find it in the odometry

    /// @brief operator<< overloading because reasons (it's cool af)
    friend std::ostream& operator<<(std::ostream& os, const FrenetPoint& p) {
        return os << "(s=" << p.s << 
              ", d=" << p.d << 
              ", y=" << p.yaw_dev << 
              ", d'=" << p.yaw_dev_prime << 
              ", y'=" << p.yaw_dev_prime << ")";
    }
};

class FrenetSpace {
private:
    pcl::KdTreeFLANN<TrajectoryPoint> kdtree;
    
    /// @brief Function to initialize the kdTree of the frenet space
    /// @param cloud An already created point cloud
    void initTree(const pcl::PointCloud<TrajectoryPoint>::Ptr& cloud);

    /// @brief Function to initialize the kdTree of the frenet space
    /// @param points A vector of points
    void initTree(const std::vector<TrajectoryPoint>& points);

public:
    /// @brief Constructor
    /// @param cloud An already created point cloud
    FrenetSpace(const pcl::PointCloud<TrajectoryPoint>::Ptr& cloud);

    /// @brief Constructor
    /// @param cloud A vector of points
    FrenetSpace(const std::vector<TrajectoryPoint>& points);

    /// @brief Search for k-nearest neighbors for the given query point.
    /// @param searchPoint the given query point
    /// @param returnPoints (output) pointer to a vector that will accommodate the resultant K nearest points 
    /// @param returnDistances (output) pointer to the vector of distances between the query point and K nearest points 
    /// @param K the number of neighbors to search for. Default 1.
    /// @param verbose If true, it will print the results.
    /// @return number of neighbors found, 0 otherwise.
    int nearestNeighbour(const TrajectoryPoint& searchPoint,
                         std::vector<TrajectoryPoint>& returnPoints,
                         std::vector<float>& returnDistances,
                         int K = 1, 
                         bool verbose = false);

    /// @brief Get the frenet point corresponding to a query odometry point.
    /// @param odometryPoint the given query point
    /// @param frenetPoint (output) the resultant frenet point. 
    /// @return 1 if found, 0 otherwise.
    int getFrenetPoint(const TrajectoryPoint odometryPoint, FrenetPoint& frenetPoint, double odometryYaw, geometry_msgs::msg::Vector3 linearVelocity, double yawAngularVelocity);
};
