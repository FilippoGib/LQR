#ifndef LQR_NODE_H
#define LQR_NODE_H

#include <rclcpp/rclcpp.hpp>
#include "trajectoryPoint.hpp"
#include "frenetSpace.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include <cmath>

class LQRNode : public rclcpp::Node 
{
    public:
        LQRNode();
        void initialization();
        void loadParameters();
        void odometryCallback(nav_msgs::msg::Odometry::SharedPtr odometry); 
        void trajectoryCallback(mmr_base::msg::SpeedProfilePoints::SharedPtr trajectory)

    private:

        rclcpp::Publisher</*to be defined*/>::SharedPtr controlsPub;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometrySub;
        rclcpp::Subscription<mmr_base::msg::SpeedProfilePoints>::SharedPtr trajectorySub;

        std::string param_topicControls;
        std::string param_topicOdometry; //topic we subscribe to, to get the odometry
        std::string param_topicTrajectory; //topic we subscribe to, to get the odometry

        TrajectoryPoint odometryPoint; //odometry in a trajectoryPoint format so that the kd-tree works with homogeneous points
        FrenetSpace frenetSpace;
        geometry_msgs::msg::Vector3 linearVelocity;
        double yawAngularVelocity;
        double linearSpeed;
        double yaw;
        bool frenetSpaceIsInitialized = false;
};

#endif