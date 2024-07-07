#include "lqr/frenetSpace.hpp"
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
            std::cout << "INFO: Output KNN search (" << i << "): " << (*cloud)[pointIdxKNNSearch[i]].x
                      << " " << (*cloud)[pointIdxKNNSearch[i]].y
                      << " (squared distance: " << returnDistances[i] << ")" << std::endl;
        }
        returnPoints.push_back((*cloud)[pointIdxKNNSearch[i]]);
    }
    return number;
}

Eigen::Vector3d findPerpendicularUnitVector(double heading) {
    // Compute the perpendicular vector
    Eigen::Vector3d perpendicularVersor(-sin(heading), cos(heading), 0.0);

    // Normalize the vector to make it a unit vector
    perpendicularVersor.normalize();

    return perpendicularVersor;
}

// Function to find the normal component of a linear velocity with respect to a heading
double findNormalComponent(const Eigen::Vector3d& linearVelocity, double heading) {
    // Find the perpendicular unit vector to the heading direction
    Eigen::Vector3d perpendicularUnitVector = findPerpendicularUnitVector(heading);

    // Compute the dot product of linear velocity and the perpendicular unit vector
    double normalComponent = linearVelocity.dot(perpendicularUnitVector);

    return normalComponent;
}

double normalizeAngle(double angle) {
    const double PI = 3.14159265358979323846;
    angle = fmod(angle + PI, 2.0 * PI);
    if (angle < 0) {
        angle += 2.0 * PI;
    }
    return angle - PI;
}

int FrenetSpace::getFrenetPoint(const TrajectoryPoint odometryPoint, FrenetPoint& frenetPoint, double odometryYaw, Eigen::Vector3d linearVelocity, double yawAngularVelocity) {

    std::vector<TrajectoryPoint> pn;
    std::vector<float> pn_d;

    int n = nearestNeighbour(odometryPoint, pn, pn_d, 1, false);

    if (n > 0) {

        //frenetPoint.s = pn[0].s_m;  // Assuming frenetPoint.s is computed this way, apparently we don't need it at this point
        frenetPoint.d = pn_d[0];
        double deviationAngle = pn[0].psi_rad - odometryYaw;
        deviationAngle = normalizeAngle(deviationAngle);
        frenetPoint.yaw_dev =  deviationAngle;
        frenetPoint.d_prime = findNormalComponent(linearVelocity, pn[0].psi_rad);
        frenetPoint.yaw_dev_prime = yawAngularVelocity;
    }
    return n;
}