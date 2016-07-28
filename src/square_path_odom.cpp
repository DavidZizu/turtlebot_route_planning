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
	double length, velocity;

	//some required private members
	ros::Publisher pub_vel;
	ros::Subscriber sub_vel;
	geometry_msgs::Twist vel_to_publish;

	//callback function
	void odomCallback(const nav_msgs::OdometryConstPtr &odom);

	int count;
	bool rotation;
	double angle;
	std::pair<double, double> coord;

	void moveForward();
	void rotate();
};

Square::Square() {
	getParams();
	angle = 0;
	count = 0;
	coord.first = length;
	coord.second = 0;
	rotation = false;
	ros::NodeHandle nh;
	pub_vel = nh.advertise<geometry_msgs::Twist>(robot_name + "mobile_base/commands/velocity", 1);
	sub_vel = nh.subscribe(robot_name + "odom", 1, &Square::odomCallback, this);
}

void Square::odomCallback(const nav_msgs::OdometryConstPtr &odom) {

	if (!rotation) {
		if (coord.first == length && coord.second == 0)
			if (odom->pose.pose.position.x < coord.first)
				pub_vel.publish(vel_to_publish);
			else {
				vel_to_publish.linear.x = 0;
				pub_vel.publish(vel_to_publish);
				coord.second = length;
				rotation = true;
				ROS_INFO_STREAM("Coordinates: (" << odom->pose.pose.position.x << ", " << odom->pose.pose.position.y << ")");
			}
		else if (coord.first == length && coord.second == length)
			if (odom->pose.pose.position.y < coord.second)
				pub_vel.publish(vel_to_publish);
			else {
				vel_to_publish.linear.x = 0;
				pub_vel.publish(vel_to_publish);
				coord.first = 0;
				rotation = true;
				ROS_INFO_STREAM("Coordinates: (" << odom->pose.pose.position.x << ", " << odom->pose.pose.position.y << ")");
			}
		else if (coord.first == 0 && coord.second == length)
			if (odom->pose.pose.position.x > coord.first)
				pub_vel.publish(vel_to_publish);
			else {
				vel_to_publish.linear.x = 0;
				pub_vel.publish(vel_to_publish);
				coord.second = 0;
				rotation = true;
				ROS_INFO_STREAM("Coordinates: (" << odom->pose.pose.position.x << ", " << odom->pose.pose.position.y << ")");
			}
		else if (coord.first == 0 && coord.second == 0)
			if (odom->pose.pose.position.y > 0)
				pub_vel.publish(vel_to_publish);
			else {
				vel_to_publish.linear.x = 0;
				pub_vel.publish(vel_to_publish);
				rotation = true;
				ROS_INFO_STREAM("Coordinates: (" << odom->pose.pose.position.x << ", " << odom->pose.pose.position.y << ")");		
			}
	}
	else {
		if (angle < 0 ? (odom->pose.pose.orientation.w < cos((angle + M_PI / 2) / 2) && odom->pose.pose.orientation.w < 0.999) : (odom->pose.pose.orientation.w > cos((angle + M_PI / 2) / 2) && odom->pose.pose.orientation.w > 0.005)) { 
			pub_vel.publish(vel_to_publish);
			//ROS_INFO_STREAM("Here, angle: " << angle << " ; w: " << odom->pose.pose.orientation.w);
		}
		else {
			vel_to_publish.angular.z = 0;
			pub_vel.publish(vel_to_publish);

			ROS_INFO_STREAM("Angle: " << (180 / M_PI) * (2 * acos(odom->pose.pose.orientation.w)));

			angle += M_PI / 2;
			if (angle == M_PI)
				angle *= -1;
			rotation = false;
		}
	}
}


void Square::startMotion() {
	while (ros::ok() && count < 8) {
		ROS_INFO_STREAM("Moving forward");
		moveForward();
		ROS_INFO_STREAM("Rotating in place");
		rotate();
	}
}

void Square::rotate() {
	vel_to_publish.angular.z = 0.5;
	vel_to_publish.linear.x = 0;

	pub_vel.publish(vel_to_publish);

	while (ros::ok() && rotation)
		ros::spinOnce();

	count++;
}

void Square::moveForward() {
	vel_to_publish.linear.x = velocity;
	vel_to_publish.angular.z = 0;
	
	while (ros::ok() && !rotation)
		ros::spinOnce(); 

	count++;
}

void Square::getParams() {
	//display some general info
	ROS_INFO_STREAM("By default" << "\n\tlength: 1 m" << "\n\trobot_name: omitted" << "\n\tvelocity: 0.2 m/s");

	//set the parameters
	if (!ros::param::get("robot_name", robot_name))
		robot_name = "";
	else
		robot_name += "/";

	if (!ros::param::get("length", length))
		length = 1;

	if (!ros::param::get("velocity", velocity))
		velocity = 0.2;
}

int main(int argc, char ** argv) {
	ros::init(argc, argv, "draw_square");
	Square draw_square;

	//ros::Rate rate(10);

	draw_square.startMotion();
	ROS_INFO_STREAM("Done");
}
