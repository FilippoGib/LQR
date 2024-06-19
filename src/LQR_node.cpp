#include "LQR/LQR_node.hpp"

LQRNode::LQRNode() : Node("LQR")
{
	RCLCPP_INFO(this->get_logger(),"LQR NODE CREATED");

	this->timer = this->create_wall_timer(500ms, std::bind(&LQRNode::initialization, this));
}

void LQRNode::loadParameters()
{
	this->declare_parameter<std::string>("node/topicOdometry", "");
	this->param_topicOdometry = this->get_parameter("node/topicOdometry").get_value<std::string>();

	this->declare_parameter<std::string>("node/topicControls", "");
	this->param_topicControls = this->get_parameter("node/topicControls").get_value<std::string>();
}

void LQRNode::initialization()
{
	this->loadParameters();

	this->controlsPub = this->create_publisher</*to be defined*/>(this->param_topicControls, 1);
	this->odometrySub = this->create_subscription<nav_msgs::msg::Odometry>(this->param_topicOdometry, 1, std::bind(&LQRNode::odometryCallback, this, std::placeholders::_1));
	
	this->timer->cancel();
}

//first we convert the Odometry into a TrajectoryPoint so that the kd-tree can work with homogeneous points
void LQRNode::odometryCallback(nav_msgs::msg::Odometry::SharedPtr odometry)
{
	this->linearVelocity = odometry->twist.twist.linear;
	this->linearSpeed = std::sqrt(this->linearVelocity.x * this->linearVelocity.x +
									this->linearVelocity.y * this->linearVelocity.y +
									this->linearVelocity.z * this->linearVelocity.z);

	// Extract angular velocity
	this->angularVelocity = odometry->twist.twist.angular;

	// Extract heading (yaw angle)
	tf2::Quaternion quat;
	tf2::fromMsg(odometry->pose.pose.orientation, quat);
	double roll, pitch;
	tf2::Matrix3x3(quat).getRPY(roll, pitch, this->yaw);

	this->odometry.x_m = odometry->pose.pose.position.x;
	this->odometry.y_m = odometry->pose.pose.position.y;
	this->odometry.s_m = 0; //we don't need it apparently
	this->odometry.psi_rad = this->yaw;

	cart2frenet(); //TODO: implementation
}