#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <iostream>
#include <fstream>

std::ofstream file;

void odomCallback(const nav_msgs::OdometryConstPtr &odom);

int main(int argc, char ** argv) {
	ros::init(argc, argv, "pose_based_on_odom");
	ros::NodeHandle nh;

	ros::Subscriber sub_odom = nh.subscribe("odom", 10, &odomCallback);
	ros::Rate rate(100);

	file.open("/home/david/DG/path.csv", std::ios::out | std::ios::trunc);

	ros::spin();

	file.close();
}

void odomCallback(const nav_msgs::OdometryConstPtr &odom) {
	file << odom->pose.pose.position.x << ", " << odom->pose.pose.position.y << "\n";
}
