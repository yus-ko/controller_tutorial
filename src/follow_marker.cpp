#include <potbot_lib/base_controller.h>
#include <potbot_lib/interactive_marker_manager.h>
#include <potbot_lib/apf_path_planner_ros.h>
#include <pluginlib/class_loader.h>
#include <dynamic_reconfigure/server.h>
#include <controller_tutorial/FollowMarkerConfig.h>

class FollowMarker
{
	private:
		ros::Publisher pub_cmd_, pub_path_;
		ros::Subscriber sub_odom_;
		boost::shared_ptr<potbot_lib::controller::BaseController> ddr_;
		potbot_lib::InteractiveMarkerManager* ims_;

		std::string path_csvfile_ = "";
		std::vector<geometry_msgs::PoseStamped> path_from_csv_;

		std::string repeat_mode_ = "none";
		bool following_reverse_ = true;

		geometry_msgs::Pose target_pre;

		dynamic_reconfigure::Server<controller_tutorial::FollowMarkerConfig> *dsrv_;

		void odomCallback(const nav_msgs::Odometry& msg);
		void reconfigureCB(const controller_tutorial::FollowMarkerConfig& param, uint32_t level);

	public:
		FollowMarker(potbot_lib::InteractiveMarkerManager* imm);
		~FollowMarker(){};
};

FollowMarker::FollowMarker(potbot_lib::InteractiveMarkerManager* imm)
{

	ims_ = imm;

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
	pub_cmd_ = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
	pub_path_ = nh.advertise<nav_msgs::Path>("path", 1);
	sub_odom_ = nh.subscribe("odom",1, &FollowMarker::odomCallback, this);

	dsrv_ = new dynamic_reconfigure::Server<controller_tutorial::FollowMarkerConfig>(n);
	dynamic_reconfigure::Server<controller_tutorial::FollowMarkerConfig>::CallbackType cb = boost::bind(&FollowMarker::reconfigureCB, this, _1, _2);
	dsrv_->setCallback(cb);

	target_pre = potbot_lib::utility::get_Pose(1e100,1e100,1e100,1e100,1e100,1e100);
}

void FollowMarker::odomCallback(const nav_msgs::Odometry& msg)
{
	const auto visual_markers = ims_->getVisualMarker();
	const auto target = visual_markers->begin()->marker.pose;
	const auto target_path = &visual_markers->begin()->trajectory;

	if (target != target_pre || (repeat_mode_ != "none" && ddr_->reachedTarget()))
	{
		if (target_path->empty())
		{
			nav_msgs::Path path_msg;
			path_msg.header = msg.header;
			path_msg.poses = path_from_csv_;

			double sx = target.position.x;
			double sy = target.position.y;
			double syaw = potbot_lib::utility::get_Yaw(target.orientation);
			for (auto& p:path_msg.poses)
			{
				double x = p.pose.position.x;
				double y = p.pose.position.y;
				geometry_msgs::Pose p_rotated;
				p.pose.position.x = x*cos(syaw) - y*sin(syaw) + sx;
				p.pose.position.y = x*sin(syaw) + y*cos(syaw) + sy;
			}

			if (repeat_mode_ == "reverse" && ddr_->reachedTarget())
			{
				if (following_reverse_)
				{
					std::reverse(path_msg.poses.begin(), path_msg.poses.end());
				}
				following_reverse_ = !following_reverse_;
			}

			pub_path_.publish(path_msg);

			ddr_->setTargetPose(path_msg.poses.back().pose);
			ddr_->setTargetPath(path_msg.poses);
		}
		else
		{
			std::vector<geometry_msgs::PoseStamped> target_path_tmp = *target_path;
			if (repeat_mode_ == "reverse" && ddr_->reachedTarget())
			{
				if (following_reverse_)
				{
					std::reverse(target_path_tmp.begin(), target_path_tmp.end());
				}
				following_reverse_ = !following_reverse_;
			}
			
			nav_msgs::Path path_msg;
			path_msg.header = msg.header;
			path_msg.poses = target_path_tmp;
			pub_path_.publish(path_msg);

			ddr_->setTargetPose(target_path_tmp.back().pose);
			ddr_->setTargetPath(target_path_tmp);
		}
	}
	target_pre = target;

	ddr_->setRobot(msg);
	geometry_msgs::Twist cmd;
	ddr_->calculateCommand(cmd);

	pub_cmd_.publish(cmd);
}

void FollowMarker::reconfigureCB(const controller_tutorial::FollowMarkerConfig& param, uint32_t level)
{
	path_csvfile_ = param.path_csvfile;
	nav_msgs::Path path_msg;
	potbot_lib::path_planner::getPathMsgFromCsv(path_msg, path_csvfile_);
	path_from_csv_ = path_msg.poses;

	repeat_mode_ = param.repeat_mode;
}

int main(int argc,char **argv){
	ros::init(argc,argv,"marker_controller");

	potbot_lib::InteractiveMarkerManager imm("target");
	FollowMarker fmc(&imm);

	ros::spin();

	return 0;
}