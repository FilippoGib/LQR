#ifndef LQR_NODE_H
#define LQR_NODE_H

#include <rclcpp/rclcpp.hpp>
#include "trajectoryPoint.hpp"
#include "c2f.hpp"
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
        void trajectoryCallback(visualization_msgs::msg::MarkerArray::SharedPtr trajectory)
        void cart2frenet();

    private:

        rclcpp::Publisher</*to be defined*/>::SharedPtr  controlsPub;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometrySub;
        rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr bordersCompletedPub;

        std::string param_topicControls;
        std::string param_topicOdometry; //topic we subscribe to, to get the odometry
        std::string param_topicCenterlineCompleted; //topic we subscribe to, to get the odometry

        TrajectoryPoint TPOdometry; //odometry in a trajectoryPoint format
        FrenetPoint frenetOdometry; //our odometry in the frenet space
        geometry_msgs::msg::Vector3 angularVelocity;
        geometry_msgs::msg::Vector3 linearVelocity;
        double linearSpeed;
        double yaw_;
}

#endif