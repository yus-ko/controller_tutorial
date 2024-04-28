#include <ros/ros.h>
#include <potbot_lib/DiffDriveController.h>
#include <potbot_lib/PathPlanner.h>
#include <dynamic_reconfigure/server.h>
#include <controller_tutorial/controller_testConfig.h>
#include <geometry_msgs/PoseArray.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Path.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/buffer.h>

class ControllerTest
{
private:
	potbot_lib::Controller::DiffDriveController robot_;
	ros::Publisher pub_cmd_, pub_lookahead_, pub_path_;
	ros::Subscriber sub_odom_, sub_goal_, sub_path_;
	bool subscribed_goal_ = false;
	std::string fullpath_to_pathcsv_, controller_name_ = "pid";
	tf2_ros::TransformBroadcaster tf_broadcaster_;
	tf2_ros::Buffer tf_buffer_;
	geometry_msgs::PoseStamped goal_;
	dynamic_reconfigure::Server<controller_tutorial::controller_testConfig> server_;

	void __odom_callback(const nav_msgs::Odometry& msg);
	void __goal_callback(const geometry_msgs::PoseStamped& msg);
	void __path_callback(const nav_msgs::Path& msg);
	void __param_callback(const controller_tutorial::controller_testConfig& param, uint32_t level);

	void __broadcast_frame(tf2_ros::TransformBroadcaster& bc, std::string parent_frame_id, std::string child_frame_id, geometry_msgs::Pose pose);

public:
	ControllerTest();
	~ControllerTest(){};
};

ControllerTest::ControllerTest()
{
	ros::NodeHandle n("~");

	std::string topic_cmd		= "cmd_vel";
	std::string topic_odom		= "odom";
	std::string topic_goal 		= "move_base_simple/goal";

	n.getParam("path_csvfile",	fullpath_to_pathcsv_);
	n.getParam("topic_cmd",		topic_cmd);
	n.getParam("topic_odom",	topic_odom);
	n.getParam("topic_goal",	topic_goal);

	ros::NodeHandle nh;

	pub_cmd_					= nh.advertise<geometry_msgs::Twist>(topic_cmd, 1);
	pub_lookahead_				= nh.advertise<visualization_msgs::Marker>("LookAhead", 1);
	pub_path_					= nh.advertise<nav_msgs::Path>("Path", 1);
	sub_odom_					= nh.subscribe(topic_odom,1, &ControllerTest::__odom_callback, this);
	sub_goal_					= nh.subscribe(topic_goal,1, &ControllerTest::__goal_callback, this);
	sub_path_					= nh.subscribe("Path",1, &ControllerTest::__path_callback, this);

	dynamic_reconfigure::Server<controller_tutorial::controller_testConfig>::CallbackType f;
	f = boost::bind(&ControllerTest::__param_callback, this, _1, _2);
	server_.setCallback(f);

	static tf2_ros::TransformListener tfListener(tf_buffer_);
}

void ControllerTest::__broadcast_frame(tf2_ros::TransformBroadcaster& bc, std::string parent_frame_id, std::string child_frame_id, geometry_msgs::Pose pose)
{
    geometry_msgs::TransformStamped transformStamped;
    transformStamped.header.frame_id = parent_frame_id; // 親フレーム
    transformStamped.child_frame_id = child_frame_id; // 新しいフレーム
    transformStamped.transform.translation.x = pose.position.x; // x座標
    transformStamped.transform.translation.y = pose.position.y; // y座標
    transformStamped.transform.translation.z = pose.position.z; // z座標
    transformStamped.transform.rotation.x = pose.orientation.x; // 回転クォータニオンのx成分
    transformStamped.transform.rotation.y = pose.orientation.y; // 回転クォータニオンのy成分
    transformStamped.transform.rotation.z = pose.orientation.z; // 回転クォータニオンのz成分
    transformStamped.transform.rotation.w = pose.orientation.w; // 回転クォータニオンのw成分

	transformStamped.header.stamp = ros::Time::now();
	bc.sendTransform(transformStamped);
}

void ControllerTest::__odom_callback(const nav_msgs::Odometry& msg)
{
	if (!subscribed_goal_) return;

	robot_.set_msg(msg);
	
	if (controller_name_ == "time_state")
	{
		__broadcast_frame(tf_broadcaster_, "map", "path_frame", goal_.pose);
		nav_msgs::Odometry robot_from_odom = msg;
		robot_from_odom.header.stamp = ros::Time(0);
		geometry_msgs::PoseStamped robot_from_path = potbot_lib::utility::get_tf(tf_buffer_, robot_from_odom, "path_frame");
		potbot_lib::utility::print_Pose(robot_from_path);
		robot_.set_msg(robot_from_path);
		robot_.time_state_control();

		if (robot_.get_current_line_following_process() == potbot_lib::Controller::PROCESS_STOP) subscribed_goal_ = false;
	}
	else if (controller_name_ == "pid")
	{
		robot_.pid_control();
		if (robot_.get_current_process() == potbot_lib::Controller::PROCESS_STOP) subscribed_goal_ = false;
	}
<<<<<<< HEAD
	else if (controller_name_ == "pure_pursuit")
	{
		robot_.pure_pursuit();
=======
	
	// robot_.set_msg(msg);
	// // robot_.pid_control();
	// robot_.pure_pursuit();
	
>>>>>>> f4a8bd35d4e547a2c83790121866dad6015c1a2d

		visualization_msgs::Marker lookahead_msg;
		robot_.get_lookahead(lookahead_msg);
		lookahead_msg.header = msg.header;
		pub_lookahead_.publish(lookahead_msg);

		if (robot_.get_current_line_following_process() == potbot_lib::Controller::PROCESS_STOP) subscribed_goal_ = false;
	}

	
	nav_msgs::Odometry robot_pose;
	robot_.to_msg(robot_pose);
	pub_cmd_.publish(robot_pose.twist.twist);
}

void ControllerTest::__goal_callback(const geometry_msgs::PoseStamped& msg)
{
	goal_ = msg;
	subscribed_goal_ = true;
	robot_.set_target(msg.pose);

	if (controller_name_ == "pure_pursuit")
	{
		nav_msgs::Path path_msg;
		potbot_lib::PathPlanner::get_path_msg_from_csv(path_msg, fullpath_to_pathcsv_);
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
		
		pub_path_.publish(path_msg);
	}
}

void ControllerTest::__path_callback(const nav_msgs::Path& msg)
{
	robot_.set_target_path(msg);
}

void ControllerTest::__param_callback(const controller_tutorial::controller_testConfig& param, uint32_t level)
{
	robot_.set_gain(	param.gain_p,
						param.gain_i,
						param.gain_d);

	robot_.set_time_state_gain(	param.time_state_gain_k1,
								param.time_state_gain_k1);

	robot_.set_margin(	param.stop_margin_angle,
						param.stop_margin_distance);

	robot_.set_limit(	param.max_linear_velocity,
						param.max_angular_velocity);
	
	robot_.set_distance_to_lookahead_point(param.distance_to_lookahead_point);

	fullpath_to_pathcsv_ = param.path_csvfile;
	controller_name_ = param.controller_name;
}

int main(int argc,char **argv){
	ros::init(argc,argv,"controller_test");

	ControllerTest ct;
	ros::spin();

	return 0;
}