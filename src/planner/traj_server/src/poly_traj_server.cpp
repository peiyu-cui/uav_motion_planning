#include <ros/ros.h>
#include <quadrotor_msgs/PolynomialTrajectory.h>
#include <quadrotor_msgs/PositionCommand.h>
#include <nav_msgs/Odometry.h>
#include <traj_utils/poly_traj.hpp>
#include <Eigen/Eigen>
#include <vector>

using namespace std;

bool has_trajectory_ = false;
int trajectory_id_, num_order_, num_segment_;

PolyTraj traj_;
nav_msgs::Odometry odom_;
quadrotor_msgs::PositionCommand cmd_;

ros::Time start_time_;

ros::Timer cmd_timer_;
ros::Subscriber traj_sub_, odom_sub_;
ros::Publisher cmd_pub_;

void cmdPubCallback(const ros::TimerEvent &event)
{
  if (!has_trajectory_)
    return;

  cmd_.header.frame_id = "world";
  cmd_.header.stamp = odom_.header.stamp;
  double t = max(0.0, (odom_.header.stamp - start_time_).toSec());

  Eigen::Vector3d pos, vel, acc;

  pos = traj_.evaluatePos(t);
  vel = traj_.evaluateVel(t);
  acc = traj_.evaluateAcc(t);

  cmd_.position.x = pos[0];
  cmd_.position.y = pos[1];
  cmd_.position.z = pos[2];

  cmd_.velocity.x = vel[0];
  cmd_.velocity.y = vel[1];
  cmd_.velocity.z = vel[2];

  cmd_.acceleration.x = acc[0];
  cmd_.acceleration.y = acc[1];
  cmd_.acceleration.z = acc[2];

  cmd_.yaw = 0.0;
  cmd_.yaw_dot = 0.01;

  cmd_pub_.publish(cmd_);
}

void trajCallback(const quadrotor_msgs::PolynomialTrajectoryConstPtr &msg)
{
  has_trajectory_ = true;
  traj_.reset();

  start_time_ = msg->header.stamp; // start time of the trajectory

  trajectory_id_ = msg->trajectory_id;
  num_order_ = msg->num_order;
  num_segment_ = msg->num_segment;

  for (int i = 0; i < num_segment_; i++)
  {
    vector<double> cx, cy, cz;
    for (int j = 0; j < num_order_ + 1; j++)
    {
      cx.push_back(msg->coef_x[i * (num_order_ + 1) + j]);
      cy.push_back(msg->coef_y[i * (num_order_ + 1) + j]);
      cz.push_back(msg->coef_z[i * (num_order_ + 1) + j]);
    }
    traj_.addSegment(cx, cy, cz, msg->time[i]);
  }

  traj_.init();
}

void odomCallback(const nav_msgs::Odometry::ConstPtr &msg)
{
  odom_ = *msg;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "poly_traj_server");
  ros::NodeHandle nh("~");

  cmd_timer_ = nh.createTimer(ros::Duration(0.01), cmdPubCallback);
  traj_sub_ = nh.subscribe("traj", 1, &trajCallback);
  odom_sub_ = nh.subscribe("odom", 1, &odomCallback);

  cmd_pub_ = nh.advertise<quadrotor_msgs::PositionCommand>("cmd", 1);

  ros::spin();

  return 0;
}