#include "lqr/lqr_node.hpp"

LQRNode::LQRNode() : Node("LQR")
{
	RCLCPP_INFO(this->get_logger(),"LQR NODE CREATED");

	this->timer = this->create_wall_timer(500ms, std::bind(&LQRNode::initialization, this));
}

void LQRNode::loadParameters()
{
	// this->declare_parameter<std::string>("topicOdometry", "Odometry");
	// this->param_topicOdometry = this->get_parameter("topicOdometry").get_value<std::string>();

	this->declare_parameter<std::string>("topicTrajectory", "planning/speedProfilePoints");
	this->param_topicTrajectory = this->get_parameter("topicTrajectory").get_value<std::string>();

	this->declare_parameter<std::string>("topicControls", "controls/controls");
	this->param_topicControls = this->get_parameter("topicControls").get_value<std::string>();
}

void LQRNode::initialization()
{
	this->loadParameters();

	this->controlsPub = this->create_publisher<ackermann_msgs::msg::AckermannDrive>(this->param_topicControls, 1);
	//this->odometrySub = this->create_subscription<nav_msgs::msg::Odometry>(this->param_topicOdometry, 1, std::bind(&LQRNode::odometryCallback, this, std::placeholders::_1));

	this->odometryFastLioOdomSub.subscribe(this, "/Odometry/fastLioOdom");
	this->imuDataSub.subscribe(this, "/imu/data");
	this->odom_sync = std::make_shared<message_filters::TimeSynchronizer<nav_msgs::msg::Odometry, sensor_msgs::msg::Imu>>(odometryFastLioOdomSub, imuDataSub, 10);
  	this->odom_sync->registerCallback(std::bind(&LQRNode::odometryCallback, this, std::placeholders::_1, std::placeholders::_2));

	this->trajectorySub = this->create_subscription<mmr_base::msg::SpeedProfilePoints>(this->param_topicTrajectory, 1, std::bind(&LQRNode::trajectoryCallback, this, std::placeholders::_1));

	this->timer->cancel();
}

Eigen::Vector3d toEigen(const geometry_msgs::msg::Vector3& v) {
    return Eigen::Vector3d(v.x, v.y, v.z);
}

//first we convert the Odometry into a TrajectoryPoint so that the kd-tree can work with homogeneous points
void LQRNode::odometryCallback(const nav_msgs::msg::Odometry::ConstSharedPtr& odometryFastLioOdom, const sensor_msgs::msg::Imu::ConstSharedPtr& imuData)
{
	RCLCPP_INFO(this->get_logger(), "Odometry callback entered\n");

	if(!this->frenetSpace.has_value())
		return;
	
	//RCLCPP_INFO(this->get_logger(), "frenetSpace has value\n");

	if(this->debugging)
		return;

	if(this->debugging_counter < 700) //scarto le prime tot odometrie così sono sicuro che la macchina non sia ferma
	{
		this->debugging_counter += 1;
		return;
	}

	this->debugging = true;

	this->linearVelocity = toEigen(odometryFastLioOdom->twist.twist.linear); //il vettore velocità lineare lo estraiamo dalla odometria di fastlio

	this->linearSpeed = this->linearVelocity.norm(); //ritorna la norma del vettore, non è che lo normalizza

	// Extract angular velocity
	this->yawAngularVelocity = imuData->angular_velocity.z; //la velocità angolare la estraiamo da imudata

	//l'heading lo estraiamo da fastLios
	tf2::Quaternion quat;
	tf2::fromMsg(odometryFastLioOdom->pose.pose.orientation, quat);
	double roll, pitch;
	tf2::Matrix3x3(quat).getRPY(roll, pitch, this->yaw);

	this->odometryPoint.x= odometryFastLioOdom->pose.pose.position.x;
	this->odometryPoint.y= odometryFastLioOdom->pose.pose.position.y;
	this->odometryPoint.psi_rad = this->yaw;

	RCLCPP_INFO(this->get_logger(), "Odometry point is: x = %f, y = %f, psi_rad = %f, yawAngularVelocity = %f, speed = %f\n", this->odometryPoint.x, this->odometryPoint.y, this->odometryPoint.psi_rad, imuData->angular_velocity.z, this->linearSpeed);

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

