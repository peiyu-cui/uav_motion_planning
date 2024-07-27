#include <path_searching/rrt_star.h>
#include <traj_optimization/minimum_control.h>
#include <plan_env/grid_map.h>

path_searching::RRTStar::Ptr rrt_star_;
traj_optimization::MinimumControl::Ptr optimizer_;

ros::Subscriber goal_sub;
ros::Subscriber odom_sub;

ros::Publisher  trajectory_pub;
visualization_msgs::Marker trajectory_marker;

nav_msgs::Odometry::ConstPtr odom_;
std::vector<Eigen::Vector3d> path;
std::vector<Eigen::Vector3d> optimal_path;
Eigen::VectorXd coef_1d;
std::vector<double> x_vec;
std::vector<double> y_vec;
std::vector<double> z_vec;


void OdomCallback(const nav_msgs::Odometry::ConstPtr& odom)
{
  odom_ = odom;
}

void GoalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  // start and end condition
  Eigen::Vector3d end_pt(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
  Eigen::Vector3d end_vel(0.0, 0.0, 0.0);
  Eigen::Vector3d end_acc(0.0, 0.0, 0.0);
  Eigen::Vector3d start_pt(odom_->pose.pose.position.x, odom_->pose.pose.position.y, odom_->pose.pose.position.z);
  Eigen::Vector3d start_vel(odom_->twist.twist.linear.x, odom_->twist.twist.linear.y, odom_->twist.twist.linear.z);
//  Eigen::Vector3d start_vel(1.0, 0.0, 0.0);
  Eigen::Vector3d start_acc(0.0, 0.0, 0.0);
  std::cout << "Start point: " << start_pt.transpose() << std::endl;
  std::cout << "End point: " << end_pt.transpose() << std::endl;
  int success = rrt_star_->search(start_pt, end_pt, path);
  optimal_path = rrt_star_->getOptimalPath();
  if (success == 1)
  {
    std::cout << "Path found, Start optimization" << std::endl;
    Eigen::VectorXd pos_x;
    Eigen::VectorXd pos_y;
    Eigen::VectorXd pos_z;
    pos_x.resize(optimal_path.size());
    pos_y.resize(optimal_path.size());
    pos_z.resize(optimal_path.size());
    for (int i = 0; i < optimal_path.size(); i++)
    {
        pos_x(i) = optimal_path[i][0];
        pos_y(i) = optimal_path[i][1];
        pos_z(i) = optimal_path[i][2];
    }
    Eigen::Vector2d bound_vel_x(start_vel[0], end_vel[0]);
    Eigen::Vector2d bound_vel_y(start_vel[1], end_vel[1]);
    Eigen::Vector2d bound_vel_z(start_vel[2], end_vel[2]);

    Eigen::Vector2d bound_acc_x(start_acc[0], end_acc[0]);
    Eigen::Vector2d bound_acc_y(start_acc[1], end_acc[1]);
    Eigen::Vector2d bound_acc_z(start_acc[2], end_acc[2]);

    // time allocate: need refine, average allocate time
    Eigen::VectorXd time_vec;
    time_vec.resize(optimal_path.size() - 1);
    for (int i = 0; i < optimal_path.size() - 1; i++)
    {
        time_vec(i) = 1.0;
    }

    // start optimization, x, y, z separately, serial optimization
    // need to change to parallel optimization
    bool success_x = optimizer_->solve(pos_x, bound_vel_x, bound_acc_x, time_vec);
    if (success_x)
    {
        coef_1d = optimizer_->getCoef1d();
        for (int i = 0; i < optimal_path.size() - 1; i++)
        {
            for (double t = 0; t < time_vec(i); t+= 0.1)
            {
                Eigen::Matrix<double, 1, 6> coef_matrix = Eigen::Matrix<double, 1, 6>::Zero();
                coef_matrix << coef_1d(6 * i), coef_1d(6 * i + 1), coef_1d(6 * i + 2), coef_1d(6 * i + 3), coef_1d(6 * i + 4), coef_1d(6 * i + 5);
                Eigen::Matrix<double, 6, 1> t_vector = Eigen::Matrix<double, 6, 1>::Zero();
                for (int i = 0; i < 6; i++)
                {
                    t_vector(i) = pow(t, i);
                }
                x_vec.push_back(coef_matrix * t_vector);
            }
        }
        x_vec.push_back(end_pt[0]);
    }
    else
    {
        std::cout << "optimize faiure!" << std::endl;
    }

    bool success_y = optimizer_->solve(pos_y, bound_vel_y, bound_acc_y, time_vec);
    if (success_y)
    {
        coef_1d = optimizer_->getCoef1d();
        for (int i = 0; i < optimal_path.size() - 1; i++)
        {
            for (double t = 0; t < time_vec(i); t+= 0.1)
            {
                Eigen::Matrix<double, 1, 6> coef_matrix;
                coef_matrix << coef_1d(6 * i), coef_1d(6 * i + 1), coef_1d(6 * i + 2), coef_1d(6 * i + 3), coef_1d(6 * i + 4), coef_1d(6 * i + 5);
                Eigen::Matrix<double, 6, 1> t_vector;
                for (int i = 0; i < 6; i++)
                {
                    t_vector(i) = pow(t, i);
                }
                y_vec.push_back(coef_matrix * t_vector);
            }
        }
        y_vec.push_back(end_pt[1]);
    }
    else
    {
        std::cout << "optimize faiure!" << std::endl;
    }

    bool success_z = optimizer_->solve(pos_z, bound_vel_z, bound_acc_z, time_vec);
    if (success_z)
    {
        coef_1d = optimizer_->getCoef1d();
        for (int i = 0; i < optimal_path.size() - 1; i++)
        {
            for (double t = 0; t < time_vec(i); t+= 0.1)
            {
                Eigen::Matrix<double, 1, 6> coef_matrix;
                coef_matrix << coef_1d(6 * i), coef_1d(6 * i + 1), coef_1d(6 * i + 2), coef_1d(6 * i + 3), coef_1d(6 * i + 4), coef_1d(6 * i + 5);
                Eigen::Matrix<double, 6, 1> t_vector;
                for (int i = 0; i < 6; i++)
                {
                    t_vector(i) = pow(t, i);
                }
                z_vec.push_back(coef_matrix * t_vector);
            }
        }
        z_vec.push_back(end_pt[2]);
    }
    else
    {
        std::cout << "optimize faiure!" << std::endl;
    }

    // visualize th trajectory
    for (int i = 0; i < x_vec.size(); i++)
    {
        geometry_msgs::Point pt;
        pt.x = x_vec[i];
        pt.y = y_vec[i];
        pt.z = z_vec[i];
        trajectory_marker.points.push_back(pt);
    }
    trajectory_pub.publish(trajectory_marker);
  }
  else
  {
    std::cout << "Path not found!" << std::endl;
  }
  path.clear();
  optimal_path.clear();
  trajectory_marker.points.clear();
  x_vec.clear();
  y_vec.clear();
  z_vec.clear();
  rrt_star_->reset();
  optimizer_->reset();
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "test_minimum_jerk");
  ros::NodeHandle nh("~");

  goal_sub = nh.subscribe<geometry_msgs::PoseStamped>("/goal", 10, &GoalCallback);
  odom_sub = nh.subscribe<nav_msgs::Odometry>("/visual_slam/odom", 10, &OdomCallback);

  trajectory_pub = nh.advertise<visualization_msgs::Marker>("/trajectory", 10);

  trajectory_marker.header.frame_id = "world";
  trajectory_marker.header.stamp = ros::Time::now();
  trajectory_marker.ns = "trajectory";
  trajectory_marker.type = visualization_msgs::Marker::LINE_STRIP;
  trajectory_marker.action = visualization_msgs::Marker::ADD;
  trajectory_marker.pose.orientation.w = 1.0;
  trajectory_marker.scale.x = 0.1;
  trajectory_marker.scale.y = 0.1;
  trajectory_marker.scale.z = 0.1;
  trajectory_marker.color.a = 1.0;
  trajectory_marker.color.r = 1.0;
  trajectory_marker.color.g = 0.0;
  trajectory_marker.color.b = 1.0;

  GridMap::Ptr grid_map = std::make_shared<GridMap>();
  grid_map->initMap(nh);
  
  rrt_star_ = std::make_shared<path_searching::RRTStar>();
  optimizer_ = std::make_shared<traj_optimization::MinimumControl>();

  rrt_star_->setParam(nh);
  rrt_star_->setGridMap(grid_map);
  rrt_star_->init();

  ros::spin();
  return 0;
}