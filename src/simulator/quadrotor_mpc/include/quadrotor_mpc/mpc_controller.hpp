#ifndef _MPC_CONTROLLER_HPP
#define _MPC_CONTROLLER_HPP

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <quadrotor_msgs/PolynomialTrajectory.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <airsim_ros_pkgs/Takeoff.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <quadrotor_mpc/controller_common.hpp>
#include <quadrotor_mpc/get_param.hpp>
#include <minco_utils/traj_visualizer.hpp>
#include <quadrotor_mpc/primal_solver.hpp>
#include <Eigen/Eigen>

class Controller
{
private:
  ros::NodeHandle nh_;

  ros::Subscriber odom_sub_, imu_sub_, traj_sub_;
  ros::Publisher ctrl_pub_;

  ros::Timer traj_vis_timer_, ctrl_pub_timer_, ctrl_solve_timer_;
  ros::Time state_time;

  StateVector state;
  tf::Quaternion quat;
  Eigen::Matrix3d rota;
  Eigen::Matrix<double, kStateDim, -1> state_des;  // desired state (pos vel acc)
  Eigen::Matrix4Xd control;                        // current control (desired jerk)

  bool has_odom_ = false;
  bool has_take_off_ = false;
  bool has_traj_ = false;
  double ctrl_cnt;

  GetParam param_;
  minco::TrajVisualizer traj_visualizer;
  Trajectory<5> reference;
  PrimalSolver primalsolver;

  inline void trajCallback(const quadrotor_msgs::PolynomialTrajectoryConstPtr& traj)
  {
    reference.clear();

    int num_order = traj->num_order;
    int num_segment = traj->num_segment;

    for (int i = 0; i < num_segment; i++)
    {
      Eigen::Matrix<double, 3, 6> coeff;

      for (int j = 0; j < num_order + 1; j++)
      {
        // NOTE: MINCO high order first
        coeff.row(0)[num_order - j] = traj->coef_x[i * (num_order + 1) + j];
        coeff.row(1)[num_order - j] = traj->coef_y[i * (num_order + 1) + j];
        coeff.row(2)[num_order - j] = traj->coef_z[i * (num_order + 1) + j];
      }

      reference.emplace_back(traj->time[i], coeff);
    }

    primalsolver.setRefTraj(reference);  // update reference trajectory
    has_traj_ = true;
  }

  inline void odomCallback(const nav_msgs::Odometry& odom_message)
  {
    // ROS_INFO("Received odom message!");
    tf::Pose pose;
    auto& pos = odom_message.pose.pose.position;
    auto& vel = odom_message.twist.twist.linear;
    state.segment(0, 3) << pos.x, pos.y, pos.z;
    state.segment(4, 3) << vel.x, vel.y, vel.z;

    tf::poseMsgToTF(odom_message.pose.pose, pose);
    quat = pose.getRotation();
    state(3) = tf::getYaw(quat);
    Eigen::Quaterniond q_eigen;
    q_eigen.w() = quat.w();
    q_eigen.x() = quat.x();
    q_eigen.y() = quat.y();
    q_eigen.z() = quat.z();
    rota = q_eigen.toRotationMatrix();

    static tf::TransformBroadcaster br;
    tf::Transform transform;
    transform.setOrigin(tf::Vector3(state(0), state(1), state(2)));
    tf::Quaternion q;
    q.setRPY(0.0, 0.0, 0.0);
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world_enu", "drone_1_intermediate"));
    has_odom_ = true;
  }

  inline void imuCallback(const sensor_msgs::Imu& imu_message)
  {
    // ROS_INFO("Received imu message!");
    auto& acc = imu_message.linear_acceleration;
    auto& orient = imu_message.orientation;

    Eigen::Quaterniond q;
    q.w() = orient.w;
    q.x() = orient.x;
    q.y() = orient.y;
    q.z() = orient.z;

    // from body(frd) to world(ned)
    Eigen::Vector3d acc_ned;
    acc_ned << acc.x, acc.y, acc.z;
    acc_ned = q.toRotationMatrix() * acc_ned;

    // from world(ned) to world(enu)
    state.segment(8, 3) << acc_ned(1), acc_ned(0), -(acc_ned(2) + param_.gravity);

    double yaw, pitch, roll;
    tf::Matrix3x3(quat).getEulerYPR(yaw, pitch, roll);

    auto& angular_vel = imu_message.angular_velocity;

    Eigen::Matrix3d S_inv;
    double sin_roll = sinf(roll);
    double cos_roll = cosf(roll);
    double cos_pitch = cosf(pitch);
    double tan_pitch = tanf(pitch);
    S_inv << 1.0f, sin_roll * tan_pitch, cos_roll * tan_pitch, 0.0f, cos_roll, -sin_roll, 0.0f, sin_roll / cos_pitch, cos_roll / cos_pitch;

    Eigen::Vector3d gyro;
    gyro << angular_vel.x, angular_vel.y, angular_vel.z;
    auto euler_rate = S_inv * gyro;

    state(7) = -euler_rate(2);
    state(11) = 0.0f;
  }

  inline void trajVisCallback(const ros::TimerEvent& event)
  {
    traj_visualizer.visualizeTraj(reference);
  }

  // inline void ctrlPubCallback(const ros::TimerEvent &event)
  // {
  //   mavros_msgs::AttitudeTarget ctrl_msg;
  //   ctrl_msg.header.stamp = ros::Time::now();
  //   ctrl_msg.thrust = 0.1174;
  //   ctrl_msg.body_rate.x = 0.0;
  //   ctrl_msg.body_rate.y = 0.0;
  //   ctrl_msg.body_rate.z = 0.3;
  //   ctrl_msg.orientation.w = 1.0;
  //   ctrl_pub_.publish(ctrl_msg);
  // }

  inline void ctrlPubCallback(const ros::TimerEvent& event)
  {
    ctrl_cnt = (ros::Time::now() - state_time).toSec() / param_.dt;
    if (ctrl_cnt > param_.ctrl_horizon)
      return;

    StateVector state_curr;
    ControlVector ctrl_curr;
    Eigen::Vector3d thrust_des;
    Eigen::Vector3d jerk_des;
    Eigen::Vector3d zb;
    Eigen::Vector3d dzb;
    Eigen::Vector3d omega;
    int cnt_pre = floor(ctrl_cnt);
    int cnt_next = ceil(ctrl_cnt);

    // linear interpelation
    state_curr = (state_des.col(cnt_next) - state_des.col(cnt_pre)) * (ctrl_cnt - cnt_pre) + state_des.col(cnt_pre);
    ctrl_curr = (control.col(cnt_next) - control.col(cnt_pre)) * (ctrl_cnt - cnt_pre) + control.col(cnt_pre);

    thrust_des = state_curr.segment(8, 3);
    thrust_des(2) += param_.gravity;

    jerk_des = ctrl_curr.segment(0, 3);

    zb = state_curr.segment(8, 3);
    zb(2) += param_.gravity;

    dzb = (Eigen::Matrix3d::Identity() - (zb * zb.transpose()) / zb.squaredNorm()).transpose() / zb.norm() * jerk_des;

    double yaw = state_curr(3);
    double d_yaw = state_curr(7);
    double sin_yaw = sin(yaw);
    double cos_yaw = cos(yaw);

    zb.normalize();
    omega << dzb(0) * sin_yaw - dzb(1) * cos_yaw - dzb(2) * (zb(0) * sin_yaw - zb(1) * cos_yaw) / (1.0f + zb(2)),
        dzb(0) * cos_yaw + dzb(1) * sin_yaw - dzb(2) * (zb(0) * cos_yaw + zb(1) * sin_yaw) / (1.0f + zb(2)),
        (zb(1) * dzb(0) - zb(0) * dzb(1)) / (1.0f + zb(2)) + d_yaw;

    double throttle = zb.dot(thrust_des) / param_.gravity * param_.hover_throttle;
    mavros_msgs::AttitudeTarget msg;

    msg.header.stamp = ros::Time::now();
    msg.body_rate.x = omega(0);
    msg.body_rate.y = omega(1);
    msg.body_rate.z = omega(2);
    msg.thrust = throttle;
    msg.orientation.w = 1.0;

    // std::cout << "curr_control: " << throttle << std::endl;
    ctrl_pub_.publish(msg);
  }

  inline void ctrlSolveCallback(const ros::TimerEvent& event)
  {
    if (!has_take_off_ || !has_odom_ || !has_traj_)
      return;

    ros::Time curr = ros::Time::now();
    StateVector state_curr = state;
    primalsolver.updateInitial(state_curr);
    primalsolver.solve(state_curr.head(3));
    state_des = primalsolver.getStateSeq(param_.pred_horizon);
    control = primalsolver.getControlSeq(param_.pred_horizon);
    state_time = curr;
    ctrl_cnt = 0.0;
  }

public:
  Controller(ros::NodeHandle nh, const GetParam& param)
    : nh_(nh)
    , param_(param)
    , traj_visualizer(nh, "drone_1")
    , reference(param_.reference)
    , has_odom_(false)
    , has_take_off_(false)
    , has_traj_(false)
    , ctrl_cnt(param_.ctrl_horizon)
    , primalsolver(param_.pred_horizon, param_.dt, param_.gravity)
  {
    traj_sub_ = nh_.subscribe("traj", 1, &Controller::trajCallback, this);
    odom_sub_ = nh_.subscribe("odom", 1, &Controller::odomCallback, this);
    imu_sub_ = nh_.subscribe("imu", 1, &Controller::imuCallback, this);

    ctrl_pub_ = nh_.advertise<mavros_msgs::AttitudeTarget>("ctrl", 1);

    state.setZero();

    traj_vis_timer_ = nh_.createTimer(ros::Duration(param_.pub_dt * 4.0), &Controller::trajVisCallback, this);
    ctrl_pub_timer_ = nh_.createTimer(ros::Duration(param_.pub_dt), &Controller::ctrlPubCallback, this);
    ctrl_solve_timer_ = nh_.createTimer(ros::Duration(param_.solve_dt), &Controller::ctrlSolveCallback, this);

    // takeoff
    ros::service::waitForService("takeoff");
    ros::ServiceClient takeoff_client = nh.serviceClient<airsim_ros_pkgs::Takeoff>("takeoff");
    airsim_ros_pkgs::Takeoff takeoff_srv;
    takeoff_srv.request.waitOnLastTask = true;
    takeoff_client.call(takeoff_srv);
    has_take_off_ = true;

    ros::Rate r(10);
    while (ros::ok() && !has_traj_)
    {
      // ROS_INFO("Waiting for trajectory...");
      ros::spinOnce();
      r.sleep();
    }

    primalsolver.setBounds(param_.state_lb, param_.state_ub, param_.ctrl_lb, param_.ctrl_ub);
    primalsolver.setWeight(param_.Q, param_.R);
    primalsolver.setRefTraj(reference);

    StateVector start_state;
    start_state << param_.start_state.col(0).cast<double>(), 0.0f, param_.start_state.col(1).cast<double>(), 0.0f,
        param_.start_state.col(2).cast<double>(), 0.0f;
    primalsolver.updateInitial(start_state);
    primalsolver.updateRefTraj(reference, 0.0f);
    primalsolver.initMpc();
  }
};

#endif  // _MPC_CONTROLLER_HPP