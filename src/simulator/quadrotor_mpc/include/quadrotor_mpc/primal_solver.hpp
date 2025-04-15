#ifndef PRIMAL_SOLVER_HPP
#define PRIMAL_SOLVER_HPP

#include <Eigen/Sparse>
#include <cmath>
#include <vector>
#include <OsqpEigen/OsqpEigen.h>
#include <minco_utils/trajectory.hpp>
#include <quadrotor_mpc/controller_common.hpp>

class PrimalSolver
{
public:
  PrimalSolver(const int& pred_horizon, const double& sample_time, const double& gravity_acc)
    : horizon(pred_horizon)
    , delta_T(sample_time)
    , gravity(gravity_acc)
    , state(NULL, kStateDim, pred_horizon)
    , control(NULL, kControlDim, pred_horizon)
  {
    if (pred_horizon > 0)
    {
      // state (1 ~ N+1), control (1 ~ N), beta (1 ~ N)
      quadprog_var_dim = (pred_horizon + 1) * kStateDim + pred_horizon * kControlDim;
      quadprog_const_dim = pred_horizon * kStateDim + quadprog_var_dim;

      solver.settings()->setVerbosity(false);
      solver.settings()->setWarmStart(true);
      solver.data()->setNumberOfVariables(quadprog_var_dim);
      solver.data()->setNumberOfConstraints(quadprog_const_dim);
      solver.settings()->setPrimalInfeasibilityTolerance(1e-3);
      solver.settings()->setMaxIteration(50);

      H.resize(quadprog_var_dim, quadprog_var_dim);
      G.resize(quadprog_var_dim, 1);
      G.setZero();

      Ac.resize(quadprog_const_dim, quadprog_var_dim);

      lb.resize(quadprog_const_dim, 1);
      ub.resize(quadprog_const_dim, 1);
      lb.setZero();
      ub.setZero();
    }
  }

  ~PrimalSolver() {};

  inline void setWeight(const Eigen::DiagonalMatrix<double, kStateDim>& state_weight,
                        const Eigen::DiagonalMatrix<double, kControlDim>& control_weight)
  {
    Q = state_weight;
    R = control_weight;
  };

  inline void setBounds(const StateVector& state_lb, const StateVector& state_ub, const ControlVector& ctrl_lb, const ControlVector& ctrl_ub)
  {
    x_lb = state_lb;
    x_ub = state_ub;
    u_lb = ctrl_lb;
    u_ub = ctrl_ub;
  };

  inline void updateRefTraj(const Trajectory<5>& traj_ref, double t0)
  {
    double t;
    x_ref.resize(kStateDim, horizon + 1);
    u_ref.resize(kControlDim, horizon + 1);
    for (unsigned int i = 0; i < horizon + 1; ++i)
    {
      Eigen::Vector4d state, dstate, d2state;
      t = t0 + delta_T * i;

      if (t < traj_ref.getTotalDuration())
      {
        auto vel = traj_ref.getVel(t);
        state << traj_ref.getPos(t).cast<double>(), atan2(vel(1), vel(0));
        dstate << traj_ref.getVel(t).cast<double>(), 0.0f;
        d2state << traj_ref.getAcc(t).cast<double>(), 0.0f;
        u_ref.col(i) << traj_ref.getJer(t).cast<double>(), 0.0f;
      }
      else
      {
        t = traj_ref.getTotalDuration() - 0.01f;
        auto vel = traj_ref.getVel(t);
        state << traj_ref.getPos(t).cast<double>(), atan2(vel(1), vel(0));
        dstate.setZero();
        d2state.setZero();
        u_ref.col(i).setZero();
      }
      x_ref.col(i) << state, dstate, d2state;
    }
    castRefToCostFunc();  // update G Matrix
  };

  inline void updateInitial(const StateVector& x_start)
  {
    lb.segment(horizon * kStateDim, kStateDim) = x_start;
    ub.segment(horizon * kStateDim, kStateDim) = x_start;
  };

  inline void initMpc()
  {
    castWeiToCostFunc();     // init H--const
    castDynToConstMat();     // init Ac--const
    castBoundsToConstVec();  // init lb ,ub--const

    solver.data()->setHessianMatrix(H);
    solver.data()->setGradient(G);
    solver.data()->setLinearConstraintsMatrix(Ac);
    solver.data()->setLowerBound(lb);
    solver.data()->setUpperBound(ub);

    if (!solver.initSolver())
      std::cout << "solver init failed!" << std::endl;

    if (solver.solveProblem() != OsqpEigen::ErrorExitFlag::NoError)
      std::cout << "solver solves with error!" << std::endl;

    solution = solver.getSolution();

    new (&state) Eigen::Map<const Eigen::MatrixXd>(solution.data(), kStateDim, horizon + 1);
    new (&control) Eigen::Map<const Eigen::MatrixXd>(solution.data() + kStateDim * (horizon + 1), kControlDim, horizon);
  };

  inline void solve(const Eigen::Vector3d& pos)
  {
    double t0 = 0.0;
    reference.locateClosestPoint(pos.cast<double>(), t0);
    updateRefTraj(reference, t0);
    solver.updateHessianMatrix(H);
    solver.updateGradient(G);
    solver.updateBounds(lb, ub);

    if (solver.solveProblem() != OsqpEigen::ErrorExitFlag::NoError)
      std::cout << "solver solves with error!" << std::endl;

    solution = solver.getSolution();

    new (&state) Eigen::Map<const Eigen::MatrixXd>(solution.data(), kStateDim, horizon + 1);
    new (&control) Eigen::Map<const Eigen::MatrixXd>(solution.data() + kStateDim * (horizon + 1), kControlDim, horizon);
  };

  inline const Eigen::Vector3d getPos(const int t)
  {
    return state.block<3, 1>(0, t + 1);
  };
  inline const Eigen::Vector3d getVel(const int t)
  {
    return state.block<3, 1>(4, t + 1);
  };
  inline const Eigen::Vector3d getAcc(const int t)
  {
    return state.block<3, 1>(8, t + 1);
  };
  inline const Eigen::Vector3d getJer(const int t)
  {
    return control.block<3, 1>(0, t + 1);
  };
  inline const double getYaw(const int t)
  {
    return state(3, t + 1);
  };
  inline const double getYawDot(const int t)
  {
    return state(7, t + 1);
  };
  inline Eigen::Matrix<double, kStateDim, -1> getStateSeq(const int n)
  {
    return state.middleCols(0, n);
  };
  inline Eigen::Matrix<double, kControlDim, -1> getControlSeq(const int n)
  {
    return control.middleCols(0, n);
  };
  inline void setRefTraj(const Trajectory<5>& traj_ref)
  {
    reference = traj_ref;
  }

private:
  OsqpEigen::Solver solver;
  Eigen::VectorXd solution;

  unsigned int quadprog_var_dim;
  unsigned int quadprog_const_dim;
  Eigen::SparseMatrix<double> H;
  Eigen::VectorXd G;
  Eigen::SparseMatrix<double> Ac;
  Eigen::VectorXd lb;
  Eigen::VectorXd ub;

  unsigned int horizon;
  Eigen::DiagonalMatrix<double, kStateDim> Q;
  Eigen::DiagonalMatrix<double, kControlDim> R;

  StateVector x0;
  Eigen::Matrix<double, kStateDim, -1> x_ref;
  Eigen::Matrix<double, kControlDim, -1> u_ref;

  double delta_T;
  StateVector x_lb;    // upper bounds of states
  StateVector x_ub;    // lower bounds of states
  ControlVector u_lb;  // upper bounds of controls
  ControlVector u_ub;  // lower bounds of controls
  double gravity;
  Trajectory<5> reference;

  Eigen::Map<const Eigen::Matrix<double, kStateDim, -1>> state;
  Eigen::Map<const Eigen::Matrix<double, kControlDim, -1>> control;

  inline void castWeiToCostFunc()
  {
    for (unsigned int i = 0; i < kStateDim; ++i)
      H.insert(i, i) = Q.diagonal()[i];
    for (unsigned int t = 0; t < horizon; ++t)
    {
      int state_idx = (t + 1) * kStateDim;
      for (unsigned int i = 0; i < kStateDim; ++i)
        H.insert(state_idx + i, state_idx + i) = Q.diagonal()[i];

      int control_idx = (horizon + 1) * kStateDim + t * kControlDim;
      for (unsigned int i = 0; i < kControlDim; ++i)
        H.insert(control_idx + i, control_idx + i) = R.diagonal()[i];
    }
  };

  inline void castRefToCostFunc()
  {
    for (unsigned int t = 0; t < horizon + 1; ++t)
      G.segment(t * kStateDim, kStateDim) = Q * (-x_ref.col(t));

    for (unsigned int t = 0; t < horizon; ++t)
      G.segment((horizon + 1) * kStateDim + t * kControlDim, kControlDim) = R * (-u_ref.col(t));
  };

  inline void castDynToConstMat()
  {
    double delta_T_p2d2 = delta_T * delta_T / 2.0f;
    double delta_T_p3d6 = delta_T_p2d2 * delta_T / 3.0f;

    for (unsigned int t = 0; t < horizon; ++t)
    {
      int state_row_idx = t * kStateDim;
      int state_col_idx = (t + 1) * kStateDim;

      // -I
      for (unsigned int i = 0; i < kSingleStateDim * 3; ++i)
        Ac.insert(state_row_idx + i, state_col_idx + i) = -1.0f;

      // A
      for (unsigned int i = 0; i < kSingleStateDim * 3; ++i)
        Ac.insert(state_row_idx + i, state_row_idx + i) = 1.0f;

      for (unsigned int i = 0; i < kSingleStateDim * 2; ++i)
        Ac.insert(state_row_idx + i, state_row_idx + kSingleStateDim + i) = delta_T;

      for (unsigned int i = 0; i < kSingleStateDim; ++i)
        Ac.insert(state_row_idx + i, state_row_idx + 2 * kSingleStateDim + i) = delta_T_p2d2;

      int control_row_idx = t * kStateDim;
      int control_col_idx = (horizon + 1) * kStateDim + t * kControlDim;

      // B
      for (unsigned int i = 0; i < kSingleStateDim; ++i)
      {
        Ac.insert(control_row_idx + i, control_col_idx + i) = delta_T_p3d6;
        Ac.insert(control_row_idx + kSingleStateDim + i, control_col_idx + i) = delta_T_p2d2;
        Ac.insert(control_row_idx + 2 * kSingleStateDim + i, control_col_idx + i) = delta_T;
      }
    }

    int const_row_idx = horizon * kStateDim;
    for (unsigned int i = 0; i < quadprog_var_dim; ++i)
    {
      Ac.insert(const_row_idx + i, i) = 1.0f;
    }
  };

  inline void castBoundsToConstVec()
  {
    int state_row_idx = horizon * kStateDim;
    int control_row_idx = state_row_idx + (horizon + 1) * kStateDim;
    for (unsigned int t = 0; t < horizon; ++t)
    {
      lb.segment(state_row_idx + (t + 1) * kStateDim, kStateDim) = x_lb;
      ub.segment(state_row_idx + (t + 1) * kStateDim, kStateDim) = x_ub;
      lb.segment(control_row_idx + t * kControlDim, kControlDim) = u_lb;
      ub.segment(control_row_idx + t * kControlDim, kControlDim) = u_ub;
    }
  };
};

#endif