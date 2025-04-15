#ifndef GET_PARAM_HPP
#define GET_PARAM_HPP

#include <ros/ros.h>
#include <Eigen/Eigen>
#include <minco_utils/minco.hpp>
#include <quadrotor_mpc/controller_common.hpp>

class GetParam
{
public:
  ros::NodeHandle nh;

  // parameter for controller
  int pred_horizon;
  int ctrl_horizon;
  int max_iter;
  int threads;

  double dt;
  double solve_dt;
  double pub_dt;
  double dual_tol;
  double primal_tol;

  double gravity;
  double hover_throttle;

  // weights for different variable
  Eigen::DiagonalMatrix<double, kStateDim> Q;
  Eigen::DiagonalMatrix<double, kControlDim> R;
  double eta;
  double theta_t;
  double theta_r;

  // bounds
  StateVector state_lb;
  StateVector state_ub;
  ControlVector ctrl_lb;
  ControlVector ctrl_ub;
  double scale_lb;
  double scale_ub;

  // reference
  Trajectory<5> reference;
  Eigen::Matrix3d start_state;
  Eigen::Matrix3d goal_state;
  Eigen::Matrix3Xd waypoints;
  Eigen::VectorXd time_vector;

  // map
  std::string host_ip;
  std::string frame_id;
  Eigen::Matrix<double, 6, 1> map_bounds;

  // robot shape
  Eigen::Matrix4Xf robot_poly;

  GetParam(ros::NodeHandle nh_) : nh(nh_)
  {
    readParam();
  };
  ~GetParam() {};

  template <typename T>
  inline void getMatrix(const std::string& name, T& matrix)
  {
    std::cout << "reading param: " << name << "..." << std::endl;
    std::vector<typename T::Scalar> data_list;

    assert(nh.getParam(name, data_list));

    assert(matrix.size() == (long int)data_list.size());

    matrix = T(data_list.data());
  }

  template <typename T>
  inline void getDynMatrix(const std::string& name, T& matrix)
  {
    std::cout << "reading param: " << name << "..." << std::endl;
    std::vector<typename T::Scalar> data_list;

    assert(nh.getParam(name, data_list));

    if (matrix.rows() == -1 || matrix.rows() == 0)
    {
      assert(data_list.size() % matrix.cols() == 0);
      matrix = Eigen::Map<T>(data_list.data(), data_list.size() / matrix.cols(), matrix.cols());
    }
    else
    {
      assert(data_list.size() % matrix.rows() == 0 || matrix.cols() == 0);
      matrix = Eigen::Map<T>(data_list.data(), matrix.rows(), data_list.size() / matrix.rows());
    }
  }

  void writeVectorToCSVColumns(const std::string& filename, const std::vector<Eigen::Vector3d>& data)
  {
    std::ofstream file;
    file.open(filename);

    if (!file.is_open())
    {
      std::cerr << "Failed to open the file." << std::endl;
      return;
    }

    // Write header (optional)
    file << "x,y,z\n";

    // Write the vector data: each row corresponds to a point, each column to a coordinate (x, y, z)
    for (const auto& vec : data)
      file << vec(0) << "," << vec(1) << "," << vec(2) << "\n";  // Write x, y, z

    file.close();  // Close the file after writing
    std::cout << "Data successfully written to " << filename << std::endl;
  }

private:
  inline void readParam()
  {
    assert(nh.getParam("predict_horizon", pred_horizon));
    assert(nh.getParam("control_horizon", ctrl_horizon));
    assert(nh.getParam("max_iteration", max_iter));
    assert(nh.getParam("threads", threads));

    assert(nh.getParam("discrete_time", dt));
    assert(nh.getParam("solve_time", solve_dt));
    assert(nh.getParam("publish_time", pub_dt));
    assert(nh.getParam("dual_tolerance", dual_tol));
    assert(nh.getParam("primal_tolerance", primal_tol));

    getMatrix("state_weight", Q.diagonal());
    getMatrix("control_weight", R.diagonal());
    assert(nh.getParam("scale_weight", eta));
    assert(nh.getParam("admm_weight_t", theta_t));
    assert(nh.getParam("admm_weight_r", theta_r));

    getMatrix("state_lower_bounds", state_lb);
    getMatrix("state_upper_bounds", state_ub);
    getMatrix("control_lower_bounds", ctrl_lb);
    getMatrix("control_upper_bounds", ctrl_ub);

    assert(nh.getParam("scale_lower_bounds", scale_lb));
    assert(nh.getParam("scale_upper_bounds", scale_ub));

    assert(nh.getParam("gravity", gravity));
    assert(nh.getParam("hover_throttle", hover_throttle));

    getMatrix("map_bounds", map_bounds);
    assert(nh.getParam("host_ip", host_ip));
    assert(nh.getParam("frame_id", frame_id));

    getMatrix("start_state", start_state);
    getMatrix("goal_state", goal_state);
    getDynMatrix("waypoints", waypoints);
    getDynMatrix("time_vector", time_vector);

    // minco::MINCO_S3NU minco;
    // std::cout << "waypoints:" << std::endl
    //           << waypoints << std::endl;
    // std::cout << "time_vector:" << std::endl
    //           << time_vector.transpose() << std::endl;
    // minco.setConditions(start_state, goal_state, time_vector.rows());
    // minco.setParameters(waypoints, time_vector);
    // minco.getTrajectory(reference);

    getDynMatrix("robot_poly", robot_poly);

    // double duration = reference.getTotalDuration();
    // std::vector<Eigen::Vector3d> path_nodes_list;
    // std::vector<Eigen::Vector3d> path_vel_list;
    // std::vector<Eigen::Vector3d> path_acc_list;
    // for (double t = 0.0; t <= duration; t += dt)
    // {
    //   Eigen::Vector3d pos = reference.getPos(t);
    //   path_nodes_list.push_back(pos);
    //   Eigen::Vector3d vel = reference.getVel(t);
    //   path_vel_list.push_back(vel);
    //   Eigen::Vector3d acc = reference.getAcc(t);
    //   path_acc_list.push_back(acc);
    // }
    // std::string filename_pos = "~/position_ref.csv";
    // std::string filename_vel = "~/vel_ref.csv";
    // std::string filename_acc = "~/acc_ref.csv";
    // writeVectorToCSVColumns(filename_pos, path_nodes_list);
    // writeVectorToCSVColumns(filename_vel, path_vel_list);
    // writeVectorToCSVColumns(filename_acc, path_acc_list);
  }
};

#endif