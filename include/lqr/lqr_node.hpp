#ifndef LQR_NODE_H
#define LQR_NODE_H

#include <rclcpp/rclcpp.hpp>
#include "trajectoryPoint.hpp"
#include "frenetSpace.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>
#include "geometry_msgs/msg/vector3.hpp"
#include "mmr_base/msg/speed_profile_point.hpp"
#include "mmr_base/msg/speed_profile_points.hpp"
#include <ackermann_msgs/msg/ackermann_drive.hpp>

#include <sensor_msgs/msg/imu.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <cmath>
#include <optional>

using std::placeholders::_1;
using std::placeholders::_2;
using namespace std::chrono_literals;

class LQRNode : public rclcpp::Node 
{
    public:
        LQRNode();
        void initialization();
        void loadParameters();
        void odometryCallback(const nav_msgs::msg::Odometry::ConstSharedPtr& odometryFastLioOdom, const sensor_msgs::msg::Imu::ConstSharedPtr& imuData); 
        void trajectoryCallback(mmr_base::msg::SpeedProfilePoints::SharedPtr trajectory);

    private:

        rclcpp::TimerBase::SharedPtr timer;
        rclcpp::Publisher<ackermann_msgs::msg::AckermannDrive>::SharedPtr controlsPub;

        message_filters::Subscriber<nav_msgs::msg::Odometry> odometryFastLioOdomSub;
        message_filters::Subscriber<sensor_msgs::msg::Imu> imuDataSub;
        
        typedef message_filters::sync_policies::ApproximateTime<nav_msgs::msg::Odometry, sensor_msgs::msg::Imu> ApproxSyncPolicy;
        std::shared_ptr<message_filters::Synchronizer<ApproxSyncPolicy>> odom_sync;

        rclcpp::Subscription<mmr_base::msg::SpeedProfilePoints>::SharedPtr trajectorySub;

        std::string param_topicControls;
        std::string param_topicOdometry; 
        std::string param_topicTrajectory; 

        TrajectoryPoint odometryPoint;
        std::optional<FrenetSpace> frenetSpace;
        Eigen::Vector3d linearVelocity;
        double yawAngularVelocity;
        double linearSpeed;
        double yaw;
        bool debugging = false;
        int debugging_counter = 1000;
};

#endif
