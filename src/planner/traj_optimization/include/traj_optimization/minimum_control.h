#ifndef MINIMUM_CONTROL_H_
#define MINIMUM_CONTROL_H_

#include <OsqpEigen/OsqpEigen.h>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <Eigen/Eigen>

namespace traj_optimization {
class MinimumControl
{
    private:
        /* main parameters for osqp solver */
        OsqpEigen::Solver solver_;
        Eigen::SparseMatrix<double> P_;
        Eigen::VectorXd q_;
        Eigen::SparseMatrix<double> A_;
        Eigen::VectorXd lb_;
        Eigen::VectorXd ub_;

        int var_num_;
        int constraint_num_;
        Eigen::VectorXd coef_1d_;

        /* function to setup osqp solver */
        void getHessian(Eigen::VectorXd &time_vec);
        void getGradient();
        void getConstraintMatrix(Eigen::VectorXd &time_vec);
        void getBound(Eigen::VectorXd &pos_1d,
                    Eigen::Vector2d &bound_vel,
                    Eigen::Vector2d &bound_acc);

    public:
        bool solve(Eigen::VectorXd& pos_1d,
                Eigen::Vector2d& bound_vel,
                Eigen::Vector2d& bound_acc,
                Eigen::VectorXd& time_vec);

        /* helper function */
        void reset();
        Eigen::VectorXd getCoef1d();

        MinimumControl() {};
        ~MinimumControl(){};

        typedef std::shared_ptr<MinimumControl> Ptr;

}; // class MinimumControl

} // namespace traj_optimization

#endif // MINIMUM_CONTROL_H_