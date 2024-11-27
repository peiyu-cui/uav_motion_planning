#include <traj_optimization/minimum_control.h>

namespace traj_optimization
{
void MinimumControl::getHessian(Eigen::VectorXd &time_vec)
{
    for (int i = 0; i < time_vec.size(); i++)
    {
        P_.insert(6 * i + 3, 6 * i + 3) = 36 * time_vec[i];
        P_.insert(6 * i + 3, 6 * i + 4) = 72 * pow(time_vec[i], 2);
        P_.insert(6 * i + 3, 6 * i + 5) = 120 * pow(time_vec[i], 3);
        P_.insert(6 * i + 4, 6 * i + 3) = 72 * pow(time_vec[i], 2);
        P_.insert(6 * i + 4, 6 * i + 4) = 192 * pow(time_vec[i], 3);
        P_.insert(6 * i + 4, 6 * i + 5) = 360 * pow(time_vec[i], 4);
        P_.insert(6 * i + 5, 6 * i + 3) = 120 * pow(time_vec[i], 3);
        P_.insert(6 * i + 5, 6 * i + 4) = 360 * pow(time_vec[i], 4);
        P_.insert(6 * i + 5, 6 * i + 5) = 720 * pow(time_vec[i], 5);
    }
}

void MinimumControl::getGradient()
{
    q_.setZero();
}

void MinimumControl::getConstraintMatrix(Eigen::VectorXd &time_vec)
{
    // start condition constraint
    A_.insert(0, 0) = 1.0;
    A_.insert(1, 1) = 1.0;
    A_.insert(2, 2) = 2.0;

    // waypoint constraint
    for (int i = 0; i < time_vec.size() - 1; i++)
    {
        A_.insert(3 + 4 * i, 6 * i) = 1.0;
        A_.insert(3 + 4 * i, 6 * i + 1) = time_vec[i];
        A_.insert(3 + 4 * i, 6 * i + 2) = pow(time_vec[i], 2);
        A_.insert(3 + 4 * i, 6 * i + 3) = pow(time_vec[i], 3);
        A_.insert(3 + 4 * i, 6 * i + 4) = pow(time_vec[i], 4);
        A_.insert(3 + 4 * i, 6 * i + 5) = pow(time_vec[i], 5);
    }

    // contiunous constraints
    for (int i = 0; i < time_vec.size() - 1; i++)
    {
        A_.insert(4 * (i + 1), 6 * i) = 1.0;
        A_.insert(4 * (i + 1), 6 * i + 1) = time_vec[i];
        A_.insert(4 * (i + 1), 6 * i + 2) = pow(time_vec[i], 2);
        A_.insert(4 * (i + 1), 6 * i + 3) = pow(time_vec[i], 3);
        A_.insert(4 * (i + 1), 6 * i + 4) = pow(time_vec[i], 4);
        A_.insert(4 * (i + 1), 6 * i + 5) = pow(time_vec[i], 5);
        A_.insert(4 * (i + 1), 6 * i + 6) = -1.0;

        A_.insert(4 * (i + 1) + 1, 6 * i) = 0.0;
        A_.insert(4 * (i + 1) + 1, 6 * i + 1) = 1.0;
        A_.insert(4 * (i + 1) + 1, 6 * i + 2) = 2 * time_vec[i];
        A_.insert(4 * (i + 1) + 1, 6 * i + 3) = 3 * pow(time_vec[i], 2);
        A_.insert(4 * (i + 1) + 1, 6 * i + 4) = 4 * pow(time_vec[i], 3);
        A_.insert(4 * (i + 1) + 1, 6 * i + 5) = 5 * pow(time_vec[i], 4);
        A_.insert(4 * (i + 1) + 1, 6 * i + 6) = 0.0;
        A_.insert(4 * (i + 1) + 1, 6 * i + 7) = -1.0;

        A_.insert(4 * (i + 1) + 2, 6 * i) = 0.0;
        A_.insert(4 * (i + 1) + 2, 6 * i + 1) = 0.0;
        A_.insert(4 * (i + 1) + 2, 6 * i + 2) = 2.0;
        A_.insert(4 * (i + 1) + 2, 6 * i + 3) = 6 * time_vec[i];
        A_.insert(4 * (i + 1) + 2, 6 * i + 4) = 12 * pow(time_vec[i], 2);
        A_.insert(4 * (i + 1) + 2, 6 * i + 5) = 20 * pow(time_vec[i], 3);
        A_.insert(4 * (i + 1) + 2, 6 * i + 6) = 0.0;
        A_.insert(4 * (i + 1) + 2, 6 * i + 7) = 0.0;
        A_.insert(4 * (i + 1) + 2, 6 * i + 8) = -2.0;
    }

    // end condition constraint
    A_.insert(3 + 4 * (time_vec.size() - 1), 6 * (time_vec.size() - 1)) = 1.0;
    A_.insert(3 + 4 * (time_vec.size() - 1), 6 * (time_vec.size() - 1) + 1) = time_vec[time_vec.size() - 1];
    A_.insert(3 + 4 * (time_vec.size() - 1), 6 * (time_vec.size() - 1) + 2) = pow(time_vec[time_vec.size() - 1], 2);
    A_.insert(3 + 4 * (time_vec.size() - 1), 6 * (time_vec.size() - 1) + 3) = pow(time_vec[time_vec.size() - 1], 3);
    A_.insert(3 + 4 * (time_vec.size() - 1), 6 * (time_vec.size() - 1) + 4) = pow(time_vec[time_vec.size() - 1], 4);
    A_.insert(3 + 4 * (time_vec.size() - 1), 6 * (time_vec.size() - 1) + 5) = pow(time_vec[time_vec.size() - 1], 5);

    A_.insert(4 * (time_vec.size()), 6 * (time_vec.size() - 1)) = 0.0;
    A_.insert(4 * (time_vec.size()), 6 * (time_vec.size() - 1) + 1) = 1.0;
    A_.insert(4 * (time_vec.size()), 6 * (time_vec.size() - 1) + 2) = 2 * time_vec[time_vec.size() - 1];
    A_.insert(4 * (time_vec.size()), 6 * (time_vec.size() - 1) + 3) = 3 * pow(time_vec[time_vec.size() - 1], 2);
    A_.insert(4 * (time_vec.size()), 6 * (time_vec.size() - 1) + 4) = 4 * pow(time_vec[time_vec.size() - 1], 3);
    A_.insert(4 * (time_vec.size()), 6 * (time_vec.size() - 1) + 5) = 5 * pow(time_vec[time_vec.size() - 1], 4);

    A_.insert(4 * (time_vec.size()) + 1, 6 * (time_vec.size() - 1)) = 0.0;
    A_.insert(4 * (time_vec.size()) + 1, 6 * (time_vec.size() - 1) + 1) = 0.0;
    A_.insert(4 * (time_vec.size()) + 1, 6 * (time_vec.size() - 1) + 2) = 2.0;
    A_.insert(4 * (time_vec.size()) + 1, 6 * (time_vec.size() - 1) + 3) = 6 * time_vec[time_vec.size() - 1];
    A_.insert(4 * (time_vec.size()) + 1, 6 * (time_vec.size() - 1) + 4) = 12 * pow(time_vec[time_vec.size() - 1], 2);
    A_.insert(4 * (time_vec.size()) + 1, 6 * (time_vec.size() - 1) + 5) = 20 * pow(time_vec[time_vec.size() - 1], 3);
}

void MinimumControl::getBound(Eigen::VectorXd &pos_1d,
                            Eigen::Vector2d &bound_vel,
                            Eigen::Vector2d &bound_acc)
{
    // start and end condition constraint
    lb_[0] = pos_1d[0];
    lb_[1] = bound_vel[0];
    lb_[2] = bound_acc[0];

    ub_[0] = pos_1d[0];
    ub_[1] = bound_vel[0];
    ub_[2] = bound_acc[0];

    lb_[3 + 4 * (pos_1d.size() - 2)] = pos_1d[pos_1d.size() - 1];
    lb_[3 + 4 * (pos_1d.size() - 2) + 1] = bound_vel[1];
    lb_[3 + 4 * (pos_1d.size() - 2) + 2] = bound_acc[1];

    ub_[3 + 4 * (pos_1d.size() - 2)] = pos_1d[pos_1d.size() - 1];
    ub_[3 + 4 * (pos_1d.size() - 2) + 1] = bound_vel[1];
    ub_[3 + 4 * (pos_1d.size() - 2) + 2] = bound_acc[1];

    // waypoint constraint
    for (int i = 0; i < pos_1d.size() - 2; i++)
    {
        lb_[3 + 4 * i] = pos_1d[i + 1];
        ub_[3 + 4 * i] = pos_1d[i + 1];
    }
}

bool MinimumControl::solve(Eigen::VectorXd& pos_1d,
                        Eigen::Vector2d& bound_vel,
                        Eigen::Vector2d& bound_acc,
                        Eigen::VectorXd& time_vec)
{
    int seg_num = time_vec.size();
    var_num_ = 6 * seg_num;
    constraint_num_ = 3 * 2 + 4 * (seg_num - 1);

    // setup the matrix and vector
    P_.resize(var_num_, var_num_);
    q_.resize(var_num_);
    A_.resize(constraint_num_, var_num_);
    lb_.resize(constraint_num_);
    ub_.resize(constraint_num_);

    P_.setZero();
    q_.setZero();
    A_.setZero();
    lb_.setZero();
    ub_.setZero();

    getHessian(time_vec);
    getGradient();
    getConstraintMatrix(time_vec);
    getBound(pos_1d, bound_vel, bound_acc);

    std::cout << "P: " << P_ << std::endl;
    std::cout << "q: " << q_.transpose() << std::endl;
    std::cout << "A: " << A_ << std::endl;
    std::cout << "lb: " << lb_.transpose() << std::endl;
    std::cout << "ub: " << ub_.transpose() << std::endl;

    solver_.settings()->setWarmStart(true);
    solver_.settings()->setPrimalInfeasibilityTollerance(1e-3);
    solver_.settings()->setMaxIteration(1000);

    solver_.data()->setNumberOfVariables(var_num_);
    solver_.data()->setNumberOfConstraints(constraint_num_);
    solver_.data()->setHessianMatrix(P_);
    solver_.data()->setGradient(q_);
    solver_.data()->setLinearConstraintsMatrix(A_);
    solver_.data()->setLowerBound(lb_);
    solver_.data()->setUpperBound(ub_);

    // initialize the solver
    if (!solver_.initSolver())
    {
        std::cout << "solver init failed!" << std::endl;
        return false;
    }

    // solve the qp problem
    if (!solver_.solve())
    {
        std::cout << "solver solve failed!" << std::endl;
        return false;
    }
    // get solution
    coef_1d_ = solver_.getSolution();
    // std::cout << "solution: " << coef_1d_.transpose() << std::endl;
    solver_.data()->clearHessianMatrix();
    solver_.data()->clearLinearConstraintsMatrix();
    solver_.clearSolver();
    return true;
}

void MinimumControl::reset()
{
    coef_1d_.setZero();
}

Eigen::VectorXd MinimumControl::getCoef1d()
{
    return coef_1d_;
}

} // namespace traj_optimization
