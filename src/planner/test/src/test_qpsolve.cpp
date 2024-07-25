#include <traj_optimization/minimum_control.h>


int main(int argc, char** argv) {
  ros::init(argc, argv, "test_qp_solve");
  ros::NodeHandle nh;

  traj_optimization::MinimumControl::Ptr min_jerk_ptr = std::make_shared<traj_optimization::MinimumControl>();
  
  Eigen::VectorXd pos_1d(4);
  pos_1d << 1.0, 2.0, 3.0, 4.0;
  Eigen::Vector2d bound_vel;
  bound_vel << 0.0, 0.0;
  Eigen::Vector2d bound_acc;
  bound_acc << 0.0, 0.0;
  Eigen::VectorXd time_vec(3);
  time_vec << 1.0, 1.0, 1.0;
  min_jerk_ptr->solve(pos_1d, bound_vel, bound_acc, time_vec);

  ros::spin();
  return 0;
  
}