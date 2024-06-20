#include "LQR/frenetSpace.hpp"

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

int FrenetSpace::getFrenetPoint(const TrajectoryPoint odometryPoint, FrenetPoint& frenetPoint) {

    std::vector<TrajectoryPoint> pn;
    std::vector<float> pn_d;

    int n = nearestNeighbour(odometryPoint, pn, pn_d, 1, false);

    if (n > 0) {


        frenetPoint.s = pn[0].s_m;  // Assuming frenetPoint.s is computed this way
        frenetPoint.d = pn_d[0];
        frenetPoint.yaw_dev = ;
        frenetPoint.d_prime = 0;
        frenetPoint.yaw_dev_prime = 0;
    }
    return n;
}

// Explicitly instantiate the template
template class pcl::KdTreeFLANN<TrajectoryPoint>;