#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>

ros::Publisher g_pub_cmd;
double g_max_linear_velocity = 0.5;
double g_max_angular_velocity = 1.0;
std::string g_game_pad_name = "xbox_one_controller";

void joy_callback(const sensor_msgs::Joy& msg)
{
	double left_stick_x			= 0.0;
	double left_stick_y			= 0.0;
	double right_stick_x		= 0.0;
	double right_stick_y		= 0.0;
	double left_triger			= 1.0;
	double right_triger			= 1.0;
	double d_pad_x				= 0.0;
	double d_pad_y				= 0.0;

	bool button_a				= false;
	bool button_b				= false;
	bool button_x				= false;
	bool button_y				= false;
	bool button_left			= false;
	bool button_right			= false;
	bool button_select			= false;
	bool button_start			= false;
	bool button_home			= false;
	bool button_left_stick		= false;
	bool button_right_stick		= false;

	if (g_game_pad_name == "xbox_one_controller")
	{
		left_stick_x		= msg.axes[0];	//左 正方向 1~-1
		left_stick_y		= msg.axes[1];	//上 正方向 1~-1
		right_stick_x		= msg.axes[3];
		right_stick_y		= msg.axes[4];
		left_triger			= msg.axes[2];
		right_triger		= msg.axes[5];
		d_pad_x				= msg.axes[6];	//左 正方向 1~-1
		d_pad_y				= msg.axes[7];	//上 正方向 1~-1

		button_a			= msg.buttons[0];
		button_b			= msg.buttons[1];
		button_x			= msg.buttons[2];
		button_y			= msg.buttons[3];
		button_left			= msg.buttons[4];
		button_right		= msg.buttons[5];
		button_select		= msg.buttons[6];
		button_start		= msg.buttons[7];
		button_home			= msg.buttons[8];
		button_left_stick	= msg.buttons[9];
		button_right_stick	= msg.buttons[10];
	}
	else if (g_game_pad_name == "dualshock4")
	{
		left_stick_x		= msg.axes[0];	//左 正方向 1~-1
		left_stick_y		= msg.axes[1];	//上 正方向 1~-1
		right_stick_x		= msg.axes[3];
		right_stick_y		= msg.axes[4];
		left_triger			= msg.axes[2];
		right_triger		= msg.axes[5];
		d_pad_x				= msg.axes[6];	//左 正方向 1~-1
		d_pad_y				= msg.axes[7];	//上 正方向 1~-1

		button_a			= msg.buttons[0];
		button_b			= msg.buttons[1];
		button_x			= msg.buttons[3];
		button_y			= msg.buttons[2];
		button_left			= msg.buttons[4];
		button_right		= msg.buttons[5];
		button_select		= msg.buttons[8];
		button_start		= msg.buttons[9];
		button_home			= msg.buttons[10];
		button_left_stick	= msg.buttons[11];
		button_right_stick	= msg.buttons[12];
	}
	else if (g_game_pad_name == "elecom_JC-U3912T")
	{
		left_stick_x		= msg.axes[0];	//左 正方向 1~-1
		left_stick_y		= msg.axes[1];	//上 正方向 1~-1
		right_stick_x		= msg.axes[4];
		right_stick_y		= msg.axes[3];
		d_pad_x				= msg.axes[4];	//左 正方向 1~-1
		d_pad_y				= msg.axes[5];	//上 正方向 1~-1

		if (msg.buttons[4]) left_triger = -1.0;
		else 				left_triger = 1.0;

		if (msg.buttons[5]) right_triger = -1.0;
		else 				right_triger = 1.0;

		button_a			= msg.buttons[2];
		button_b			= msg.buttons[3];
		button_x			= msg.buttons[0];
		button_y			= msg.buttons[1];
		button_left			= msg.buttons[6];
		button_right		= msg.buttons[7];
		button_select		= msg.buttons[10];
		button_start		= msg.buttons[11];
		button_left_stick	= msg.buttons[8];
		button_right_stick	= msg.buttons[9];
	}
	
	

	double max_linear_velocity = 0.5;
	double max_angular_velocity = 1.0;

	max_linear_velocity = 0.5*g_max_linear_velocity * -0.5*(right_triger-1.0) + 0.5*g_max_linear_velocity;
	max_angular_velocity = 0.5*g_max_angular_velocity * -0.5*(right_triger-1.0) + 0.5*g_max_angular_velocity;

	double linear_velocity		= 0;
	double angular_velocity		= 0;

	if (!button_right)
	{
		if (d_pad_x != 0 || d_pad_y != 0)
		{
			linear_velocity = d_pad_y*max_linear_velocity;
			angular_velocity = d_pad_x*max_angular_velocity;
		}
		else
		{
			linear_velocity = left_stick_y*max_linear_velocity;
			angular_velocity = left_stick_x*max_angular_velocity;
		}
	}

	geometry_msgs::Twist cmd;
	cmd.linear.x = linear_velocity;
	cmd.angular.z = angular_velocity;

	g_pub_cmd.publish(cmd);
}

int main(int argc,char **argv){
	ros::init(argc,argv,"joy_to_cmd");

	ros::NodeHandle n("~");

	std::string topic_cmd		= "cmd_vel";
	std::string topic_joy		= "joy";

	n.getParam("topic_cmd",					topic_cmd);
	n.getParam("topic_joy",					topic_joy);
	n.getParam("max_linear_velocity",		g_max_linear_velocity);
	n.getParam("max_angular_velocity",		g_max_angular_velocity);
	n.getParam("game_pad_name",				g_game_pad_name);


	ros::NodeHandle nh;

	g_pub_cmd					= nh.advertise<geometry_msgs::Twist>(topic_cmd, 1);
	ros::Subscriber sub_odom	= nh.subscribe(topic_joy,1,joy_callback);

	ros::spin();

	return 0;
}