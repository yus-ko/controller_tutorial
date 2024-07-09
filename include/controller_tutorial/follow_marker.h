#include <potbot_lib/base_controller.h>
#include <potbot_lib/interactive_marker_manager.h>
#include <potbot_lib/apf_path_planner_ros.h>
#include <pluginlib/class_loader.h>
#include <dynamic_reconfigure/server.h>
#include <controller_tutorial/FollowMarkerConfig.h>

#include <nav_core/base_local_planner.h>

class FollowMarker
{
	private:
		ros::Publisher pub_cmd_, pub_path_;
		ros::Subscriber sub_odom_;
		boost::shared_ptr<potbot_lib::controller::BaseController> ddr_;
        boost::shared_ptr<nav_core::BaseLocalPlanner> nbc_;
		potbot_lib::InteractiveMarkerManager* ims_;

		std::string path_csvfile_ = "";
		std::vector<geometry_msgs::PoseStamped> path_from_csv_;

		std::string repeat_mode_ = "none";
		bool following_reverse_ = true;

		geometry_msgs::Pose target_pre_;
		size_t target_path_size_pre_;
		bool trajectory_recording_pre_;

		dynamic_reconfigure::Server<controller_tutorial::FollowMarkerConfig> *dsrv_;

		void odomCallback(const nav_msgs::Odometry& msg);
		void reconfigureCB(const controller_tutorial::FollowMarkerConfig& param, uint32_t level);

	public:
		FollowMarker(potbot_lib::InteractiveMarkerManager* imm);
		~FollowMarker(){};
};
