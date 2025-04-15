#include <ros/ros.h>
#include <quadrotor_mpc/get_param.hpp>
#include <quadrotor_mpc/mpc_controller.hpp>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "controller_node");
  ros::NodeHandle nh;

  GetParam param(nh);
  Controller controller(nh, param);

  ros::spin();
}