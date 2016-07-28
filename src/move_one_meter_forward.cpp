#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>

class Move {
public:
	ros::NodeHandle nh;
	geometry_msgs::Twist vel_to_publish;
	bool move;
	ros::Publisher pub_vel;
	ros::Subscriber sub_vel;

	Move();

	void odomCallback(const nav_msgs::OdometryConstPtr &odom);
};

Move::Move() {
	vel_to_publish.linear.x = 0.2;
	move = true;
	pub_vel = nh.advertise<geometry_msgs::Twist>("mobile_base/commands/velocity", 1);
	sub_vel = nh.subscribe("odom", 1, &Move::odomCallback, this);
}

void Move::odomCallback(const nav_msgs::OdometryConstPtr &odom) {
	if (odom->pose.pose.position.x < 1)
		pub_vel.publish(vel_to_publish);
	else {
		vel_to_publish.linear.x = 0;
		pub_vel.publish(vel_to_publish);
		move = false;
	}
}

int main(int argc, char ** argv) {
	ros::init(argc, argv, "move_one_meter_odom_based");
	Move movement;

	while (ros::ok() && movement.move)
		ros::spinOnce();

}
