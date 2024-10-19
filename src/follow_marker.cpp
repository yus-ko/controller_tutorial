#include <potbot_base/base_controller.h>
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
		boost::shared_ptr<potbot_base::Controller> ddr_;
		pluginlib::ClassLoader<potbot_base::Controller> loader_;
		potbot_lib::InteractiveMarkerManager* ims_;

		std::string path_csvfile_ = "";
		std::vector<geometry_msgs::PoseStamped> path_from_csv_;

		bool publish_command_ = true;

		std::string repeat_mode_ = "none";
		bool following_reverse_ = true;

		geometry_msgs::Pose target_pre_;
		size_t target_path_size_pre_;
		bool trajectory_recording_pre_;

		dynamic_reconfigure::Server<controller_tutorial::FollowMarkerConfig> *dsrv_;

		void odomCallback(const nav_msgs::Odometry& msg);
		void reconfigureCB(const controller_tutorial::FollowMarkerConfig& param, uint32_t level);

	public:
		FollowMarker(potbot_lib::InteractiveMarkerManager* imm, tf2_ros::Buffer* tf);
		~FollowMarker();
};

FollowMarker::FollowMarker(potbot_lib::InteractiveMarkerManager* imm, tf2_ros::Buffer* tf) : loader_("potbot_base", "potbot_base::Controller")
{

	ims_ = imm;

	ros::NodeHandle n("~");
	
	std::string plugin_name = "potbot_lib/PurePursuit";
	n.getParam("controller_name", plugin_name);
	try
	{
		ddr_ = loader_.createInstance(plugin_name);
		ddr_->initialize("controller", tf);
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

	target_pre_ = potbot_lib::utility::get_pose(1e100,1e100,1e100,1e100,1e100,1e100);
}

FollowMarker::~FollowMarker()
{
	if (dsrv_) delete dsrv_;
}

bool comp(const geometry_msgs::Pose& p1, const geometry_msgs::Pose& p2)
{
	return
		p1.position.x == p2.position.x &&
		p1.position.y == p2.position.y &&
		p1.position.z == p2.position.z &&
		p1.orientation.x == p2.orientation.x &&
		p1.orientation.y == p2.orientation.y &&
		p1.orientation.z == p2.orientation.z &&
		p1.orientation.w == p2.orientation.w;
}

void FollowMarker::odomCallback(const nav_msgs::Odometry& msg)
{
	const auto visual_markers = ims_->getVisualMarker();
	const auto target = visual_markers->begin()->marker.pose;
	const auto target_path = &visual_markers->begin()->trajectory;
	const auto trajectory_recording = visual_markers->begin()->trajectory_recording;

	if (!comp(target, target_pre_) ||
		target_path_size_pre_ != target_path->size() ||
		trajectory_recording_pre_ != trajectory_recording ||
		(repeat_mode_ != "none" && ddr_->reachedTarget()))
	{
		if (trajectory_recording)
		{
			if (!target_path->empty())
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

				geometry_msgs::PoseStamped goal;
				goal.pose = target;
				// ddr_->setTargetPose(target_path_tmp.back());
				
				ddr_->setTargetPath(target_path_tmp);
				ddr_->setTargetPose(goal);
			}
			else
			{
				nav_msgs::Path path_msg;
				path_msg.header = msg.header;
				pub_path_.publish(path_msg);
				geometry_msgs::PoseStamped pose;
				pose.header = msg.header;
				pose.pose = msg.pose.pose;
				ddr_->setTargetPose(pose);
				ddr_->setTargetPath(path_msg.poses);
			}
		}
		else
		{
			if (!path_from_csv_.empty())
			{
				nav_msgs::Path path_msg;
				path_msg.header = msg.header;
				path_msg.poses = path_from_csv_;

				double sx = target.position.x;
				double sy = target.position.y;
				double syaw = tf2::getYaw(target.orientation);
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

				ddr_->setTargetPose(path_msg.poses.back());
				ddr_->setTargetPath(path_msg.poses);
			}
		}
	}
	target_pre_ = target;
	target_path_size_pre_ = target_path->size();
	trajectory_recording_pre_ = trajectory_recording;

	ddr_->setRobot(msg);
	geometry_msgs::Twist cmd;
	ddr_->calculateCommand(cmd);

	if (publish_command_) pub_cmd_.publish(cmd);
}

void FollowMarker::reconfigureCB(const controller_tutorial::FollowMarkerConfig& param, uint32_t level)
{
	publish_command_ = param.publish_command;

	path_csvfile_ = param.path_csvfile;
	nav_msgs::Path path_msg;
	potbot_lib::path_planner::getPathMsgFromCsv(path_msg, path_csvfile_);
	path_from_csv_ = path_msg.poses;

	repeat_mode_ = param.repeat_mode;
}

int main(int argc,char **argv){
	ros::init(argc,argv,"marker_controller");

	tf2_ros::Buffer tf_buffer_;
	tf2_ros::TransformListener tfListener(tf_buffer_);

	potbot_lib::InteractiveMarkerManager imm("target");

	FollowMarker fmc(&imm, &tf_buffer_);

	ros::spin();

	return 0;
}