#ifndef LQR_NODE_H
#define LQR_NODE_H

#include <rclcpp/rclcpp.hpp>
#include "LQR/trajectoryPoint.hpp"

//TODO: put this into the include file
class FrenetPoint {
public:
    double s;              // distance covered so far
    double d;              // lateral deviation from the trajectory
    double yaw_dev;        // angular deviation i.e. difference between odometry_yaw and the tangent to the reference
    double d_prime;        // first degree derivative of d -> lateral deviation velocity
    double yaw_dev_prime;  // first degree derivative of yaw_dev -> angular deviation velocity WE HAVE IT IN THE ODOMETRY, NO NEED FOR CALCULATIONS
};

class LQRNode : public rclcpp::Node 
{
    public:
        LQRNode();
        void initialization();
        void loadParameters();
        void odometryCallback(nav_msgs::msg::Odometry::SharedPtr odometry); 
        void cart2frenet();

    private:
        TrajectoryPoint TPOdometry; //odometry in a trajectoryPoint format
        FrenetPoint frenetOdometry; //our odometry in the frenet space
        std::string param_topicOdometry; //topic we subscribe to, to get the odometry
        std::string param_topicControls;
    
}

#endif