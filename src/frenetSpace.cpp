#include "LQR/frenetSpace.hpp"
#include <cmath>

void FrenetSpace::initTree(const pcl::PointCloud<TrajectoryPoint>::Ptr& cloud) {
    kdtree.setInputCloud(cloud);
    std::cout << "INFO: Frenet Space initialized\n";
}

void FrenetSpace::initTree(const std::vector<TrajectoryPoint>& points) {
    pcl::PointCloud<TrajectoryPoint>::Ptr cloud(new pcl::PointCloud<TrajectoryPoint>);

    for (const auto &point : points) {
        cloud->points.push_back(point);
    }
    cloud->width = cloud->points.size();
    cloud->height = 1;
    initTree(cloud);
}

FrenetSpace::FrenetSpace(const pcl::PointCloud<TrajectoryPoint>::Ptr& cloud) {
    initTree(cloud);
}

FrenetSpace::FrenetSpace(const std::vector<TrajectoryPoint>& points) {
    initTree(points);
}

int FrenetSpace::nearestNeighbour(const TrajectoryPoint& searchPoint,
                                  std::vector<TrajectoryPoint>& returnPoints,
                                  std::vector<float>& returnDistances,
                                  int K,
                                  bool verbose) {

    std::vector<int> pointIdxKNNSearch(K);

    returnDistances.resize(K);

    const auto cloud = kdtree.getInputCloud();

    if (verbose) {
        std::cout << "INFO: K nearest neighbor search with K=" << K << " in a tree with size=" << cloud->size() << std::endl;
    }

    int number = kdtree.nearestKSearch(searchPoint, K, pointIdxKNNSearch, returnDistances);
    for (int i = 0; i < number; ++i) {
        if (verbose) {
            std::cout << "INFO: Output KNN search (" << i << "): " << (*cloud)[pointIdxKNNSearch[i]].x_m
                      << " " << (*cloud)[pointIdxKNNSearch[i]].y_m
                      << " (squared distance: " << returnDistances[i] << ")" << std::endl;
        }
        returnPoints.push_back((*cloud)[pointIdxKNNSearch[i]]);
    }
    return number;
}

int FrenetSpace::getFrenetPoint(const TrajectoryPoint odometryPoint, FrenetPoint& frenetPoint, double odometryYaw, geometry_msgs::msg::Vector3 linearVelocity, double yawAngularVelocity) {

    std::vector<TrajectoryPoint> pn;
    std::vector<float> pn_d;

    int n = nearestNeighbour(odometryPoint, pn, pn_d, 1, false);

    if (n > 0) {


        //frenetPoint.s = pn[0].s_m;  // Assuming frenetPoint.s is computed this way, apparently we don't need it at this point
        frenetPoint.d = pn_d[0];
        frenetPoint.yaw_dev = (pn_d[0].psi_rad - odometryYaw) % M_1_PI; //deviazione angolare normalizzata a +- PI greco
        frenetPoint.d_prime = findNormalComponent(linearVelocity, pn_d[0].psi_rad);
        frenetPoint.yaw_dev_prime = yawAngularVelocity;
    }
    return n;
}

geometry_msgs::msg::Vector3 findPerpendicularUnitVector(double heading) {
    geometry_msgs::msg::Vector3 perpendicularVersor;
    perpendicularVersor.x = -sin(heading);
    perpendicularVersor.y = cos(heading);
    perpendicularVersor.z = 0.0; // Assuming heading is in the XY plane

    // Normalize the vector to make it a unit vector
    double magnitude = std::sqrt(perpendicularVersor.x * perpendicularVersor.x +
                                 perpendicularVersor.y * perpendicularVersor.y +
                                 perpendicularVersor.z * perpendicularVersor.z);
    
    perpendicularVersor.x /= magnitude;
    perpendicularVersor.y /= magnitude;
    perpendicularVersor.z /= magnitude;
    
    return perpendicularVersor;
}

double findNormalComponent(const geometry_msgs::msg::Vector3 linearVelocity, double heading) {
    // Find the perpendicular unit vector to the heading direction
    geometry_msgs::msg::Vector3 perpendicularUnitVector = findPerpendicularUnitVector(heading);

    // Compute the dot product of linear velocity and the perpendicular unit vector
    double normalComponent = linearVelocity.x * perpendicularUnitVector.x +
                             linearVelocity.y * perpendicularUnitVector.y +
                             linearVelocity.z * perpendicularUnitVector.z;

    return normalComponent;
}


// Explicitly instantiate the template
template class pcl::KdTreeFLANN<TrajectoryPoint>;