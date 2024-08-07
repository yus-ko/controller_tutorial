#include <potbot_lib/pid.h>
#include <potbot_lib/utility_ros.h>
#include <ros/ros.h>
#include <tf2/utils.h>
#include <potbot_msgs/ObstacleArray.h>
#include <dynamic_reconfigure/server.h>
#include <controller_tutorial/controller_tutorialConfig.h>
#include <geometry_msgs/PoseArray.h>
#include <random>

std::vector<potbot_lib::controller::PID> g_robot;
using namespace potbot_lib::utility;

void inipose_callback(const geometry_msgs::PoseWithCovarianceStamped& msg)
{
	nav_msgs::Odometry ini_pose;
	ini_pose.header.frame_id = "map";
	ini_pose.pose = msg.pose;
	for (auto& robo : g_robot) to_agent(ini_pose, robo);
}

void goal_callback(const geometry_msgs::PoseStamped& msg)
{
	potbot_lib::Pose p;
	p.position.x = msg.pose.position.x;
	p.position.y = msg.pose.position.y;
	p.position.z = msg.pose.position.z;
	get_rpy(msg.pose.orientation, p.rotation.x, p.rotation.y, p.rotation.z);
	for (auto& robo : g_robot) robo.setTargetPoint(p);
}

void param_callback(const controller_tutorial::controller_tutorialConfig& param, uint32_t level)
{
	for (auto& robo : g_robot)
	{
		robo.setGain(param.gain_p, param.gain_i, param.gain_d);
		robo.setMargin(param.stop_margin_angle, param.stop_margin_distance);
		robo.setLimit(param.max_linear_velocity, param.max_angular_velocity);
	}
}

int main(int argc,char **argv){
	ros::init(argc,argv,"controller_tutorial");

	ros::NodeHandle n("~");

	double control_frequency = 50.0;
	double norm_noise_mean_x = 0;
	double norm_noise_variance_x = 0.0001;
	double norm_noise_mean_y = 0;
	double norm_noise_variance_y = 0.0001;
	double norm_noise_mean_yaw = 0;
	double norm_noise_variance_yaw = 0.0001;
	double norm_noise_mean_linear_velocity = 0;
	double norm_noise_variance_linear_velocity = 0.0001;
	double norm_noise_mean_angular_velocity = 0;
	double norm_noise_variance_angular_velocity = 0.0001;
	
	n.getParam("control_frequency", control_frequency);
	n.getParam("norm_noise_mean_x", norm_noise_mean_x);
	n.getParam("norm_noise_variance_x", norm_noise_variance_x);
	n.getParam("norm_noise_mean_y", norm_noise_mean_y);
	n.getParam("norm_noise_variance_y", norm_noise_variance_y);
	n.getParam("norm_noise_mean_yaw", norm_noise_mean_yaw);
	n.getParam("norm_noise_variance_yaw", norm_noise_variance_yaw);
	n.getParam("norm_noise_mean_linear_velocity", norm_noise_mean_linear_velocity);
	n.getParam("norm_noise_variance_linear_velocity", norm_noise_variance_linear_velocity);
	n.getParam("norm_noise_mean_angular_velocity", norm_noise_mean_angular_velocity);
	n.getParam("norm_noise_variance_angular_velocity", norm_noise_variance_angular_velocity);

	g_robot.resize(1);

	ros::NodeHandle nh;

	ros::Publisher pub_odom					= nh.advertise<nav_msgs::Odometry>("odom", 1);
	ros::Publisher pub_odom_truth			= nh.advertise<nav_msgs::Odometry>("odom/truth", 1);

	ros::Subscriber sub_inipose				= nh.subscribe("initialpose",1,inipose_callback);
	ros::Subscriber sub_goal				= nh.subscribe("move_base_simple/goal",1,goal_callback);

	dynamic_reconfigure::Server<controller_tutorial::controller_tutorialConfig> server;
	dynamic_reconfigure::Server<controller_tutorial::controller_tutorialConfig>::CallbackType f;
	f = boost::bind(param_callback, _1, _2);
	server.setCallback(f);
	
	ros::Rate rate(control_frequency);

	nav_msgs::Odometry robot_pose;
	robot_pose.header.frame_id = "map";
	robot_pose.pose.pose = get_pose(0,0,0,0,0,0);	//ロボット初期位置 x y z r p y

	for (auto& robo : g_robot)
	{
		robo.deltatime = 1.0/control_frequency;
		to_agent(robot_pose, robo);
	}

	std::random_device rd;
    std::default_random_engine generator(rd());
	std::normal_distribution<double> distribution_x(norm_noise_mean_x, sqrt(norm_noise_variance_x));
	std::normal_distribution<double> distribution_y(norm_noise_mean_y, sqrt(norm_noise_variance_y));
	std::normal_distribution<double> distribution_yaw(norm_noise_mean_yaw, sqrt(norm_noise_variance_yaw));
    std::normal_distribution<double> distribution_linear_velocity(norm_noise_mean_linear_velocity, sqrt(norm_noise_variance_linear_velocity));
	std::normal_distribution<double> distribution_angular_velocity(norm_noise_mean_angular_velocity, sqrt(norm_noise_variance_angular_velocity));

	while (ros::ok())
	{
		for (auto& robo : g_robot)
		{
			robo.calculateCommand();
			robo.update();
		}
		
		to_msg(g_robot.front(), robot_pose);

		pub_odom_truth.publish(robot_pose);
		
		robot_pose.pose.pose.position.x += distribution_x(generator);
		robot_pose.pose.pose.position.y += distribution_y(generator);
		double yaw = tf2::getYaw(robot_pose.pose.pose.orientation);
		robot_pose.pose.pose.orientation = get_quat(0,0,yaw+distribution_yaw(generator));
		robot_pose.twist.twist.linear.x += distribution_linear_velocity(generator);
		robot_pose.twist.twist.angular.z += distribution_angular_velocity(generator);

		pub_odom.publish(robot_pose);
        
		ros::spinOnce();
		rate.sleep();
		
	}

	return 0;
}