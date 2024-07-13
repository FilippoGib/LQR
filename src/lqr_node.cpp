#include "lqr/lqr_node.hpp"

LQRNode::LQRNode() : Node("LQR")
{
	RCLCPP_INFO(this->get_logger(),"LQR NODE CREATED");

	this->timer = this->create_wall_timer(500ms, std::bind(&LQRNode::initialization, this));
}

void LQRNode::loadParameters()
{
	this->declare_parameter<std::string>("node/topicOdometry", "Odometry");
	this->param_topicOdometry = this->get_parameter("node/topicOdometry").get_value<std::string>();

	this->declare_parameter<std::string>("node/topicTrajectory", "planning/speedProfilePoints");
	this->param_topicTrajectory = this->get_parameter("node/topicTrajectory").get_value<std::string>();

	this->declare_parameter<std::string>("node/topicControls", "controls/controls");
	this->param_topicControls = this->get_parameter("node/topicControls").get_value<std::string>();
}

void LQRNode::initialization()
{
	this->loadParameters();

	auto qos = rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_sensor_data));
	qos.best_effort();

	this->controlsPub = this->create_publisher<ackermann_msgs::msg::AckermannDrive>(this->param_topicControls, 1);
	this->odometrySub = this->create_subscription<nav_msgs::msg::Odometry>(this->param_topicOdometry, qos, std::bind(&LQRNode::odometryCallback, this, std::placeholders::_1));
	this->trajectorySub = this->create_subscription<mmr_base::msg::SpeedProfilePoints>(this->param_topicTrajectory, 1, std::bind(&LQRNode::trajectoryCallback, this, std::placeholders::_1));

	this->timer->cancel();
}

Eigen::Vector3d toEigen(const geometry_msgs::msg::Vector3& v) {
    return Eigen::Vector3d(v.x, v.y, v.z);
}

//first we convert the Odometry into a TrajectoryPoint so that the kd-tree can work with homogeneous points
void LQRNode::odometryCallback(nav_msgs::msg::Odometry::SharedPtr odometry)
{
	//RCLCPP_INFO(this->get_logger(), "Odometry callback entered\n");

	if(!this->frenetSpace.has_value())
		return;
	
	//RCLCPP_INFO(this->get_logger(), "frenetSpace has value\n");

	if(this->debugging)
		return;

	if(this->debugging_counter < 600) //scarto le prime tot odometrie così sono sicuro che la macchina non sia ferma
	{
		this->debugging_counter += 1;
		return;
	}
	
	RCLCPP_INFO(this->get_logger(), "Odometry callback entered\n");

	this->debugging = true;

	this->linearVelocity = toEigen(odometry->twist.twist.linear);

	this->linearSpeed = this->linearVelocity.norm(); //ritorna la norma del vettore, non è che lo normalizza

	// Extract angular velocity
	this->yawAngularVelocity = odometry->twist.twist.angular.z;

	// Extract heading (yaw angFrenetSpacele)
	tf2::Quaternion quat;
	tf2::fromMsg(odometry->pose.pose.orientation, quat);
	double roll, pitch;
	tf2::Matrix3x3(quat).getRPY(roll, pitch, this->yaw);

	this->odometryPoint.x= odometry->pose.pose.position.x;
	this->odometryPoint.y= odometry->pose.pose.position.y;
	this->odometryPoint.psi_rad = this->yaw;

	RCLCPP_INFO(this->get_logger(), "Odometry point is: x = %f, y = %f, psi_rad = %f, angularVelocity = %f, speed = %f\n", this->odometryPoint.x, this->odometryPoint.y, this->odometryPoint.psi_rad, odometry->twist.twist.angular.z, this->linearSpeed);

	FrenetPoint frenetOdometry; //our goal

	FrenetSpace& frenet_space = this->frenetSpace.value(); //modo per accedere al valore di un optional

	RCLCPP_INFO(this->get_logger(), "Calling getFrenetPoint\n");

	int n = frenet_space.getFrenetPoint(this->odometryPoint, frenetOdometry, this->yaw, this->linearVelocity,this->yawAngularVelocity); 

	if (n > 0)	
		RCLCPP_INFO(this->get_logger(), "FrenetPoint successfully returned: s = %f, d = %f, yaw_dev = %f, d_prime = %f, yaw_dev_prime = %f\n", frenetOdometry.s, frenetOdometry.d,frenetOdometry.yaw_dev,frenetOdometry.d_prime,frenetOdometry.yaw_dev_prime);

	//TODO: matmul per ottenere l'output
}

void LQRNode::trajectoryCallback(mmr_base::msg::SpeedProfilePoints::SharedPtr trajectory)
{
	if(this->frenetSpace.has_value())
	{
		return;
	}
     pcl::PointCloud<TrajectoryPoint>::Ptr cloud(new pcl::PointCloud<TrajectoryPoint>);

    for (const auto &point : trajectory->points) {
		TrajectoryPoint p;
		p.x= point.point.x;
		p.y= point.point.y;
		p.psi_rad = point.ackerman_point.steering_angle;

        cloud->points.push_back(p);
    }
    cloud->width = cloud->points.size();
    cloud->height = 1;
	std::cout << "Calling frenetSpace constructor" <<std::endl;
    this->frenetSpace = FrenetSpace(cloud);
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  auto lqrNode = std::make_shared<LQRNode>();

  rclcpp::spin(lqrNode);

  return 0;
}

