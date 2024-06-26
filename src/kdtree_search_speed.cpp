#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <iostream>
#include <vector>
#include <ctime>
#include <cstdlib>

int main(int argc, char *argv[]) {
    std::cout<<"KD-TREE KNN SPEED TEST. ";
    if (argc<3){
        std::cout<<"The first argument should be the point cloud size, the second one K."<<std::endl;
        return 0;
    }

    srand(time(NULL));

    std::cout<<"Loading data..."<<std::endl;

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

    cloud->width =atoi(argv[1]);
    cloud->height = 1;
    cloud->points.resize(cloud->width * cloud->height);

    for (std::size_t i = 0; i < cloud->size(); ++i) {

        (*cloud)[i].x = 1024.0f * rand() / (RAND_MAX + 1.0f);
        (*cloud)[i].y = 1024.0f * rand() / (RAND_MAX + 1.0f);
        (*cloud)[i].z = 1024.0f * rand() / (RAND_MAX + 1.0f);
    }

    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    kdtree.setInputCloud(cloud);
    pcl::PointXYZ searchPoint;
    searchPoint.x = 1024.0f * rand() / (RAND_MAX + 1.0f);
    searchPoint.y = 1024.0f * rand() / (RAND_MAX + 1.0f);
    searchPoint.z = 1024.0f * rand() / (RAND_MAX + 1.0f);


    int K = atoi(argv[2]);

    std::vector<int> pointIdxKNNSearch(K);
    std::vector<float> pointKNNSquaredDistance(K);

    std::cout << "K nearest neighbor search with K=" << K<< " in a tree with size="<<kdtree.getInputCloud()->size() << ". ";
    clock_t start_time = clock();
    if (kdtree.nearestKSearch(searchPoint, K, pointIdxKNNSearch, pointKNNSquaredDistance) > 0)
    {
        std::cout << "Operation performed in "<< (double) (clock()-start_time)/CLOCKS_PER_SEC<<" seconds."<<std::endl;
    }
    return 1;
}