#include <iostream>
#include <math.h>
#include <random>
#include <Eigen/Dense>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include "quadrotor_msgs/PositionCommand.h"

ros::Subscriber cmd_sub_;
ros::Publisher odom_pub_;

quadrotor_msgs::PositionCommand cmd_;
double init_x_, init_y_, init_z_;

bool rcv_cmd_ = false;
void rcvPosCmdCallBack(const quadrotor_msgs::PositionCommand cmd)
{
	rcv_cmd_ = true;
	cmd_ = cmd;
}

void pubOdom()
{
	nav_msgs::Odometry odom;
	odom.header.stamp = ros::Time::now();
	odom.header.frame_id = "world";

	if (rcv_cmd_)
	{
		odom.pose.pose.position.x = cmd_.position.x;
		odom.pose.pose.position.y = cmd_.position.y;
		odom.pose.pose.position.z = cmd_.position.z;

		Eigen::Vector3d alpha = Eigen::Vector3d(cmd_.acceleration.x, cmd_.acceleration.y, cmd_.acceleration.z) + 9.8 * Eigen::Vector3d(0, 0, 1);
		Eigen::Vector3d xC(cos(cmd_.yaw), sin(cmd_.yaw), 0);
		Eigen::Vector3d yC(-sin(cmd_.yaw), cos(cmd_.yaw), 0);
		Eigen::Vector3d xB = (yC.cross(alpha)).normalized();
		Eigen::Vector3d yB = (alpha.cross(xB)).normalized();
		Eigen::Vector3d zB = xB.cross(yB);
		Eigen::Matrix3d R;
		R.col(0) = xB;
		R.col(1) = yB;
		R.col(2) = zB;
		Eigen::Quaterniond q(R);
		odom.pose.pose.orientation.w = q.w();
		odom.pose.pose.orientation.x = q.x();
		odom.pose.pose.orientation.y = q.y();
		odom.pose.pose.orientation.z = q.z();

		odom.twist.twist.linear.x = cmd_.velocity.x;
		odom.twist.twist.linear.y = cmd_.velocity.y;
		odom.twist.twist.linear.z = cmd_.velocity.z;

		odom.twist.twist.angular.x = cmd_.acceleration.x;
		odom.twist.twist.angular.y = cmd_.acceleration.y;
		odom.twist.twist.angular.z = cmd_.acceleration.z;
	}
	else
	{
		odom.pose.pose.position.x = init_x_;
		odom.pose.pose.position.y = init_y_;
		odom.pose.pose.position.z = init_z_;

		odom.pose.pose.orientation.w = 1;
		odom.pose.pose.orientation.x = 0;
		odom.pose.pose.orientation.y = 0;
		odom.pose.pose.orientation.z = 0;

		odom.twist.twist.linear.x = 0.0;
		odom.twist.twist.linear.y = 0.0;
		odom.twist.twist.linear.z = 0.0;

		odom.twist.twist.angular.x = 0.0;
		odom.twist.twist.angular.y = 0.0;
		odom.twist.twist.angular.z = 0.0;
	}

	odom_pub_.publish(odom);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "fake_drone");
	ros::NodeHandle nh("~");

	nh.param("init_x", init_x_, 0.0);
	nh.param("init_y", init_y_, 0.0);
	nh.param("init_z", init_z_, 0.0);

	cmd_sub_ = nh.subscribe("cmd", 1, rcvPosCmdCallBack);
	odom_pub_ = nh.advertise<nav_msgs::Odometry>("odom", 1);

	ros::Rate rate(100);
	bool status = ros::ok();
	while (status)
	{
		pubOdom();
		ros::spinOnce();
		status = ros::ok();
		rate.sleep();
	}

	return 0;
}