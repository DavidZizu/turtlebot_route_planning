#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <math.h>

class Square {
public:
	Square();
	void getParams();
	void startMotion();
private:
	
	//parameters set by the user
	std::string robot_name;
	double linear_length, linear_velocity, angular_velocity;

	//some required private members
	ros::Publisher pub_vel;
	geometry_msgs::Twist vel_to_publish;

	int count;
	bool rotation;
	double angle;

	void moveForward();
	void rotate();
};

Square::Square() {
	getParams();
	angle = 0;
	count = 0;
	rotation = false;
	ros::NodeHandle nh;
	pub_vel = nh.advertise<geometry_msgs::Twist>(robot_name + "mobile_base/commands/velocity", 1);
}

void Square::startMotion() {
	while (ros::ok() && count < 8 ) {
		ROS_INFO_STREAM("Moving forward");
		moveForward();
		ROS_INFO_STREAM("Rotating in place");
		rotate();
	}
}

void Square::rotate() {
	vel_to_publish.angular.z = angular_velocity;
	vel_to_publish.linear.x = 0;
	rotation = true;

	double time = M_PI / (2 * angular_velocity); 

	ros::Time start = ros::Time::now();

	while (ros::ok() && ros::Time::now() - start < ros::Duration(time))
		pub_vel.publish(vel_to_publish);

	vel_to_publish.linear.z = 0;
	pub_vel.publish(vel_to_publish); 

	count++;
}

void Square::moveForward() {
	vel_to_publish.linear.x = linear_velocity;
	vel_to_publish.angular.z = 0;

	double time = linear_length / linear_velocity;

	ros::Time start = ros::Time::now();

	while (ros::ok() && ros::Time::now() - start < ros::Duration(time))
		pub_vel.publish(vel_to_publish);
	
	vel_to_publish.linear.x = 0;
	pub_vel.publish(vel_to_publish); 

	count++;
}

void Square::getParams() {
	//display some general info
	ROS_INFO_STREAM("By default" << "\n\tlinear_length: 1 m" << "\n\trobot_name: omitted" << "\n\tlinear_velocity: 0.2 m/s" << "\n\tangular_velocity: 0.5 m/s");

	//set the parameters
	if (!ros::param::get("robot_name", robot_name))
		robot_name = "";
	else
		robot_name += "/";

	if (!ros::param::get("linear_length", linear_length))
		linear_length = 1;

	if (!ros::param::get("linear_velocity", linear_velocity))
		linear_velocity = 0.2;

	if (!ros::param::get("angular_velocity", angular_velocity))
		angular_velocity = 0.5;
}

int main(int argc, char ** argv) {
	ros::init(argc, argv, "draw_square");
	Square draw_square;

	draw_square.startMotion();
	ROS_INFO_STREAM("Done");
}
