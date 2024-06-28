#include <ros/ros.h>
#include <potbot_lib/dwa_controller_ros.h>
#include <potbot_lib/apf_path_planner_ros.h>
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
	potbot_lib::controller::DWAControllerROS robot_;
	ros::Publisher pub_cmd_, pub_lookahead_, pub_path_, pub_path_marker_, pub_best_path_, pub_split_path_;
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

	n.getParam("path_csvfile",	fullpath_to_pathcsv_);

	ros::NodeHandle nh;

	pub_cmd_					= nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
	pub_lookahead_				= nh.advertise<visualization_msgs::Marker>("LookAhead", 1);
	pub_path_					= nh.advertise<nav_msgs::Path>("Path/raw", 1);
	pub_path_marker_ 			= nh.advertise<visualization_msgs::MarkerArray>("multi_path", 1);
	pub_best_path_ 				= nh.advertise<nav_msgs::Path>("best_path", 1);
	pub_split_path_ 			= nh.advertise<nav_msgs::Path>("Path/split", 1);

	sub_odom_					= nh.subscribe("odom",1, &ControllerTest::__odom_callback, this);
	sub_goal_					= nh.subscribe("goal",1, &ControllerTest::__goal_callback, this);
	sub_path_					= nh.subscribe("Path/raw",1, &ControllerTest::__path_callback, this);

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

	robot_.setMsg(msg);
	
	if (controller_name_ == "time_state")
	{
		__broadcast_frame(tf_broadcaster_, "map", "path_frame", goal_.pose);
		nav_msgs::Odometry robot_from_odom = msg;
		robot_from_odom.header.stamp = ros::Time(0);
		geometry_msgs::PoseStamped robot_from_path = potbot_lib::utility::get_tf(tf_buffer_, robot_from_odom, "path_frame");
		potbot_lib::utility::print_Pose(robot_from_path);
		robot_.setMsg(robot_from_path);

		// robot_.time_state_control();
		robot_.p166_41();

		if (robot_.getCurrentLineFollowingProcess() == potbot_lib::controller::PROCESS_STOP) subscribed_goal_ = false;
	}
	else if (controller_name_ == "pid")
	{
		robot_.pidControl();
		if (robot_.getCurrentProcess() == potbot_lib::controller::PROCESS_STOP) subscribed_goal_ = false;
	}
	else if (controller_name_ == "pure_pursuit")
	{
		robot_.setInitializePose(false);
		// robot_.purePursuit();

		// visualization_msgs::Marker lookahead_msg;
		// robot_.get_lookahead(lookahead_msg);
		// lookahead_msg.header = msg.header;
		// pub_lookahead_.publish(lookahead_msg);

		if (robot_.getCurrentLineFollowingProcess() == potbot_lib::controller::PROCESS_STOP) subscribed_goal_ = false;
	}
	else if (controller_name_ == "dwa")
	{
		robot_.calculateCommand();

		nav_msgs::Path path_msg;
		robot_.getBestPath(path_msg);
		pub_best_path_.publish(path_msg);

		robot_.getSplitPath(path_msg);
		pub_split_path_.publish(path_msg);
		
		visualization_msgs::MarkerArray multi_path_msg;
		robot_.getPlans(multi_path_msg);
		pub_path_marker_.publish(multi_path_msg);
	}

	
	nav_msgs::Odometry robot_pose;
	robot_.toMsg(robot_pose);
	pub_cmd_.publish(robot_pose.twist.twist);
}

void ControllerTest::__goal_callback(const geometry_msgs::PoseStamped& msg)
{
	goal_ = msg;
	subscribed_goal_ = true;
	robot_.setTarget(msg.pose.position.x, msg.pose.position.y, tf2::getYaw(msg.pose.orientation));

	if (controller_name_ == "pure_pursuit" || controller_name_ == "dwa")
	{
		nav_msgs::Path path_msg;
		potbot_lib::path_planner::getPathMsgFromCsv(path_msg, fullpath_to_pathcsv_);
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
	std::vector<potbot_lib::Pose> path;
	for (const auto&  p:msg.poses)
	{
		potbot_lib::Pose pose;
		pose.position.x = p.pose.position.x;
		pose.position.y = p.pose.position.y;
		pose.rotation.z = tf2::getYaw(p.pose.orientation);
		path.push_back(pose);
	}
	
	robot_.setTargetPath(path);
	robot_.setDwaTargetPath(msg);
}

void ControllerTest::__param_callback(const controller_tutorial::controller_testConfig& param, uint32_t level)
{
	robot_.setGain(		param.gain_p,
						param.gain_i,
						param.gain_d);

	robot_.setTimeStateGain(	param.time_state_gain_k1,
								param.time_state_gain_k1);

	robot_.setMargin(	param.stop_margin_angle,
						param.stop_margin_distance);

	robot_.setLimit(	param.max_linear_velocity,
						param.max_angular_velocity);
	
	robot_.setDistanceToLookaheadPoint(param.distance_to_lookahead_point);

	fullpath_to_pathcsv_ = param.path_csvfile;
	controller_name_ = param.controller_name;

	robot_.setTimeIncrement(param.dwa_time_increment);
	robot_.setLinearVelocityMin(param.min_linear_velocity);
	robot_.setLinearVelocityMax(param.max_linear_velocity);
	robot_.setLinearVelocityIncrement(param.dwa_linear_velocity_increment);
	robot_.setAngularVelocityMin(param.min_angular_velocity);
	robot_.setAngularVelocityMax(param.max_angular_velocity);
	robot_.setAngularVelocityIncrement(param.dwa_angular_velocity_increment);
}

int main(int argc,char **argv){
	ros::init(argc,argv,"controller_test");

	ControllerTest ct;
	ros::spin();

	return 0;
}