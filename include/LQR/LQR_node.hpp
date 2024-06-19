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
        void cart2frenet();

    private:

        rclcpp::Publisher</*to be defined*/>::SharedPtr  controlsPub;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometrySub;

        std::string param_topicOdometry; //topic we subscribe to, to get the odometry
        std::string param_topicControls;

        TrajectoryPoint TPOdometry; //odometry in a trajectoryPoint format
        FrenetPoint frenetOdometry; //our odometry in the frenet space
        double linear_speed_;
        geometry_msgs::msg::Vector3 angular_velocity_;
        double yaw_;
    
}

#endif