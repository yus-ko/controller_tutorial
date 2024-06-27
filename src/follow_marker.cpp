#include <potbot_lib/pure_pursuit.h>
#include <potbot_lib/diff_drive_controller_ros.h>
#include <potbot_lib/interactive_marker_manager.h>
#include <pluginlib/class_loader.h>

class FollowMarker
{
	private:
		ros::Publisher pub_cmd_;
		ros::Subscriber sub_odom_;
		boost::shared_ptr<potbot_lib::controller::BaseController> ddr_;
		potbot_lib::InteractiveMarkerManager* ims_;

		void odomCallback(const nav_msgs::Odometry& msg);

	public:
		FollowMarker(potbot_lib::InteractiveMarkerManager* imm);
		~FollowMarker(){};
};

FollowMarker::FollowMarker(potbot_lib::InteractiveMarkerManager* imm)
{

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

	ros::NodeHandle n("~");
	
	pluginlib::ClassLoader<potbot_lib::controller::BaseController> loader("potbot_lib", "potbot_lib::controller::BaseController");
	std::string plugin_name = "potbot_lib/PurePursuit";
	n.getParam("controller_name", plugin_name);
	try
	{
		ddr_ = loader.createInstance(plugin_name);
		ddr_->initialize("controller");
	}
	catch(pluginlib::PluginlibException& ex)
	{
		ROS_ERROR("failed to load plugin. Error: %s", ex.what());
	}

	ros::NodeHandle nh;
	pub_cmd_					= nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
	sub_odom_					= nh.subscribe("odom",1, &FollowMarker::odomCallback, this);
}

void FollowMarker::odomCallback(const nav_msgs::Odometry& msg)
{
	const geometry_msgs::Point target = ims_->getMarker()->front().pose.position;
	ddr_->setTargetPath(ims_->getMarkerTrajectories()->front());
	ddr_->setRobot(msg);
	geometry_msgs::Twist cmd;
	ddr_->calculateCommand(cmd);

	pub_cmd_.publish(cmd);
}

int main(int argc,char **argv){
	ros::init(argc,argv,"marker_controller");

	potbot_lib::InteractiveMarkerManager imm("target");
	FollowMarker fmc(&imm);

	ros::spin();

	return 0;
}