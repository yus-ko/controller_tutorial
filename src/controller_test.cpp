#include <ros/ros.h>
#include <potbot_lib/DiffDriveController.h>
#include <dynamic_reconfigure/server.h>
#include <controller_tutorial/controller_testConfig.h>
#include <geometry_msgs/PoseArray.h>

potbot_lib::Controller::DiffDriveController g_robot;
ros::Publisher g_pub_cmd;
bool g_subscribed_goal = false;

void odom_callback(const nav_msgs::Odometry& msg)
{
	g_robot.set_msg(msg);
	if (!g_subscribed_goal)
		g_robot.set_target(	msg.pose.pose.position.x,
							msg.pose.pose.position.y,
							potbot_lib::utility::get_Yaw(msg.pose.pose.orientation));
	g_robot.pid_control();
	nav_msgs::Odometry robot_pose;
	g_robot.to_msg(robot_pose);

	g_pub_cmd.publish(robot_pose.twist.twist);
}

void goal_callback(const geometry_msgs::PoseStamped& msg)
{
	g_subscribed_goal = true;
	g_robot.set_target(	msg.pose.position.x,
						msg.pose.position.y,
						potbot_lib::utility::get_Yaw(msg.pose.orientation));
}

void param_callback(const controller_tutorial::controller_testConfig& param, uint32_t level)
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
	ros::init(argc,argv,"controller_test");

	ros::NodeHandle n("~");

	std::string topic_cmd = "cmd_vel";
	std::string topic_odom = "odom";
	std::string topic_goal = "move_base_simple/goal";

	n.getParam("topic_cmd", topic_cmd);
	n.getParam("topic_odom", topic_odom);
	n.getParam("topic_goal", topic_goal);

	ros::NodeHandle nh;

	g_pub_cmd					= nh.advertise<geometry_msgs::Twist>(topic_cmd, 1);
	ros::Subscriber sub_odom				= nh.subscribe(topic_odom,1,odom_callback);
	ros::Subscriber sub_goal				= nh.subscribe(topic_goal,1,goal_callback);

	dynamic_reconfigure::Server<controller_tutorial::controller_testConfig> server;
	dynamic_reconfigure::Server<controller_tutorial::controller_testConfig>::CallbackType f;
	f = boost::bind(param_callback, _1, _2);
	server.setCallback(f);
	
	g_robot.set_target(1,1,0);

	ros::spin();

	return 0;
}