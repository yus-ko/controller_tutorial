#include <ros/ros.h>
#include <potbot_lib/DiffDriveController.h>
#include <potbot_lib/PathPlanner.h>
#include <dynamic_reconfigure/server.h>
#include <controller_tutorial/controller_testConfig.h>
#include <geometry_msgs/PoseArray.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Path.h>

potbot_lib::Controller::DiffDriveController g_robot;
ros::Publisher g_pub_cmd, g_pub_lookahead, g_pub_path;
bool g_subscribed_goal = false;
std::string g_fullpath_to_pathcsv;

void odom_callback(const nav_msgs::Odometry& msg)
{
	g_robot.set_msg(msg);
	if (!g_subscribed_goal)
		g_robot.set_target(	msg.pose.pose.position.x,
							msg.pose.pose.position.y,
							potbot_lib::utility::get_Yaw(msg.pose.pose.orientation));

	// g_robot.pid_control();
	g_robot.pure_pursuit();

	nav_msgs::Odometry robot_pose;
	g_robot.to_msg(robot_pose);

	visualization_msgs::Marker lookahead_msg;
	g_robot.get_lookahead(lookahead_msg);
	lookahead_msg.header = msg.header;

	g_pub_cmd.publish(robot_pose.twist.twist);
	g_pub_lookahead.publish(lookahead_msg);

}

void goal_callback(const geometry_msgs::PoseStamped& msg)
{
	g_subscribed_goal = true;
	g_robot.set_target(msg.pose);

	nav_msgs::Path path_msg;
	potbot_lib::PathPlanner::get_path_msg_from_csv(path_msg, g_fullpath_to_pathcsv);
	path_msg.header = msg.header;

	double goalx = msg.pose.position.x;
	double goaly = msg.pose.position.y;
	double goalyaw = potbot_lib::utility::get_Yaw(msg.pose.orientation);
	for (auto& p:path_msg.poses)
	{
		double x = p.pose.position.x;
		double y = p.pose.position.y;
		geometry_msgs::Pose p_rotated;
		p.pose.position.x = x*cos(goalyaw) - y*sin(goalyaw) + goalx;
		p.pose.position.y = x*sin(goalyaw) + y*cos(goalyaw) + goaly;
	}
	
	g_pub_path.publish(path_msg);
}

void path_callback(const nav_msgs::Path& msg)
{
	g_robot.set_target_path(msg);
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
	
	g_robot.set_distance_to_lookahead_point(param.distance_to_lookahead_point);

	g_fullpath_to_pathcsv = param.path_csvfile;
}

int main(int argc,char **argv){
	ros::init(argc,argv,"controller_test");

	ros::NodeHandle n("~");

	std::string topic_cmd		= "cmd_vel";
	std::string topic_odom		= "odom";
	std::string topic_goal 		= "move_base_simple/goal";

	n.getParam("path_csvfile",	g_fullpath_to_pathcsv);
	n.getParam("topic_cmd",		topic_cmd);
	n.getParam("topic_odom",	topic_odom);
	n.getParam("topic_goal",	topic_goal);

	ros::NodeHandle nh;

	g_pub_cmd					= nh.advertise<geometry_msgs::Twist>(topic_cmd, 1);
	g_pub_lookahead				= nh.advertise<visualization_msgs::Marker>("LookAhead", 1);
	g_pub_path					= nh.advertise<nav_msgs::Path>("Path", 1);
	ros::Subscriber sub_odom	= nh.subscribe(topic_odom,1,odom_callback);
	ros::Subscriber sub_goal	= nh.subscribe(topic_goal,1,goal_callback);
	ros::Subscriber sub_path	= nh.subscribe("Path",1,path_callback);

	dynamic_reconfigure::Server<controller_tutorial::controller_testConfig> server;
	dynamic_reconfigure::Server<controller_tutorial::controller_testConfig>::CallbackType f;
	f = boost::bind(param_callback, _1, _2);
	server.setCallback(f);

	ros::spin();

	return 0;
}