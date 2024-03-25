#include <ros/ros.h>
#include <potbot_lib/DiffDriveController.h>
#include <dynamic_reconfigure/server.h>
#include <controller_tutorial/controller_tutorialConfig.h>
#include <geometry_msgs/PoseArray.h>

potbot_lib::Controller::DiffDriveController g_robot;
ros::Publisher g_pub_cmd;

void odom_callback(const nav_msgs::Odometry& msg)
{
	g_robot.set_msg(msg);
	g_robot.pid_control();
	nav_msgs::Odometry robot_pose;
	g_robot.to_msg(robot_pose);

	g_pub_cmd.publish(robot_pose.twist.twist);
}

void param_callback(const controller_tutorial::controller_tutorialConfig& param, uint32_t level)
{
	g_robot.set_gain(	param.gain_p,
							param.gain_i,
							param.gain_d);

	g_robot.set_margin(	param.stop_margin_angle,
							param.stop_margin_distance);

	g_robot.set_limit(	param.max_linear_velocity,
							param.max_angular_velocity);
}

int main(int argc,char **argv){
	ros::init(argc,argv,"controller_tutorial");

	ros::NodeHandle n("~");

	double control_frequency = 50.0;
	int particle_num = 100;
	double norm_noise_mean_linear_velocity = 0;
	double norm_noise_variance_linear_velocity = 0.1;
	double norm_noise_mean_angular_velocity = 0;
	double norm_noise_variance_angular_velocity = 0.1;
	
	n.getParam("control_frequency", control_frequency);
	n.getParam("particle_num", particle_num);
	n.getParam("norm_noise_mean_linear_velocity", norm_noise_mean_linear_velocity);
	n.getParam("norm_noise_variance_linear_velocity", norm_noise_variance_linear_velocity);
	n.getParam("norm_noise_mean_angular_velocity", norm_noise_mean_angular_velocity);
	n.getParam("norm_noise_variance_angular_velocity", norm_noise_variance_angular_velocity);

	ros::NodeHandle nh;

	g_pub_cmd					= nh.advertise<geometry_msgs::Twist>("/robot_5/cmd_vel", 1);
	ros::Subscriber sub_odom				= nh.subscribe("/robot_5/odom",1,odom_callback);

	dynamic_reconfigure::Server<controller_tutorial::controller_tutorialConfig> server;
	dynamic_reconfigure::Server<controller_tutorial::controller_tutorialConfig>::CallbackType f;
	f = boost::bind(param_callback, _1, _2);
	server.setCallback(f);
	
	g_robot.set_target(1,1,0);

	ros::spin();

	return 0;
}