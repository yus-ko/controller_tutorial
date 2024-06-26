#include <potbot_lib/diff_drive_controller_ros.h>
#include <potbot_lib/interactive_marker_manager.h>

class FollowMarker
{
private:
	ros::Publisher pub_cmd_;
	ros::Subscriber sub_odom_;
	potbot_lib::controller::DiffDriveControllerROS* ddr_;
	potbot_lib::InteractiveMarkerManager* ims_;

	void odomCallback(const nav_msgs::Odometry& msg);

public:
	FollowMarker(potbot_lib::controller::DiffDriveControllerROS* ddr, potbot_lib::InteractiveMarkerManager *imm);
	~FollowMarker(){};
};

FollowMarker::FollowMarker(potbot_lib::controller::DiffDriveControllerROS* ddr, potbot_lib::InteractiveMarkerManager *imm)
{
	ddr_ = ddr;
	ims_ = imm;

	visualization_msgs::Marker marker_target;
	marker_target.type = visualization_msgs::Marker::SPHERE;
	marker_target.scale.x = 0.08;
	marker_target.scale.y = 0.08;
	marker_target.scale.z = 0.08;
	marker_target.color.r = 0.7;
	marker_target.color.g = 0.7;
	marker_target.color.b = 0.0;
	marker_target.color.a = 1.0;
	marker_target.pose = potbot_lib::utility::get_Pose();
	ims_->initInteractiveMarkers(marker_target);
	ims_->initInteractiveMarkerServer();

	// ros::NodeHandle n("~");

	ros::NodeHandle nh;

	pub_cmd_					= nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);

	sub_odom_					= nh.subscribe("odom",1, &FollowMarker::odomCallback, this);

	// dynamic_reconfigure::Server<controller_tutorial::controller_testConfig>::CallbackType f;
	// f = boost::bind(&FollowMarker::__param_callback, this, _1, _2);
	// server_.setCallback(f);

	// static tf2_ros::TransformListener tfListener(tf_buffer_);
}

void FollowMarker::odomCallback(const nav_msgs::Odometry& msg)
{
	const geometry_msgs::Point target = ims_->getMarker()->front().pose.position;
	// ROS_INFO_STREAM(target);

	ddr_->setTargetPath(ims_->getMarkerTrajectories()->front());
	ddr_->setMsg(msg);
	ddr_->purePursuit();
	ddr_->publishLookahead();

	nav_msgs::Odometry robot;
	ddr_->toMsg(robot);
	
	// double x = msg.pose.pose.position.x;
	// double y = msg.pose.pose.position.y;
	// double yaw = tf2::getYaw(msg.pose.pose.orientation);

	// double l_d = hypot(target.y-y,target.x-x);

	// geometry_msgs::Twist cmd;
	// if (l_d > 0.2)
	// {
	// 	double alpha = atan2(target.y-y,target.x-x) - yaw;
	// 	double v = 0.2;
		
	// 	double omega = 2.0*v*sin(alpha)/l_d;

		
	// 	cmd.linear.x = v;
	// 	cmd.angular.z = omega;
	// }

	pub_cmd_.publish(robot.twist.twist);
	
}

int main(int argc,char **argv){
	ros::init(argc,argv,"marker_controller");

	potbot_lib::InteractiveMarkerManager imm("target");
	potbot_lib::controller::DiffDriveControllerROS ddr("controller");
	FollowMarker fmc(&ddr, &imm);
	ros::spin();

	return 0;
}