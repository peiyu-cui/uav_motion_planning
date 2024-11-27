#include <path_searching/kino_astar.h>

namespace path_searching
{

void KinoAstar::setParam(ros::NodeHandle& nh)
{
  nh.param("kino_astar/allocated_node_num", allocated_node_num_, 100000);
  nh.param("kino_astar/collision_check_type", collision_check_type_, 1);
  nh.param("kino_astar/rou_time", rou_, 1.0);
  nh.param("kino_astar/lambda_heu", lambda_heu_, 2.0);
  nh.param("kino_astar/goal_tolerance", goal_tolerance_, 2.0);
  nh.param("kino_astar/time_step_size", step_size_, 0.1);
  nh.param("kino_astar/max_velocity", max_vel_, 5.0);
  nh.param("kino_astar/max_accelration", max_accel_, 7.0);
  nh.param("kino_astar/acc_resolution", acc_res_, 2.0);
  nh.param("kino_astar/sample_tau", sample_tau_, 0.5);
  nh.param("kino_se3/robot_r", robot_r_, 0.2);
  nh.param("kino_se3/robot_h", robot_h_, 0.1);

  local_cloud_sub_ = nh.subscribe<sensor_msgs::PointCloud2>("local_cloud", 10, &KinoAstar::localCloudCallback, this);
  path_node_pub_ = nh.advertise<visualization_msgs::Marker>("kino_astar_path_nodes", 1);
  elliposid_pub_ = nh.advertise<visualization_msgs::Marker>("elliposid", 300);

  path_node_marker_.header.frame_id = "world";
  path_node_marker_.type = visualization_msgs::Marker::SPHERE_LIST;
  path_node_marker_.action = visualization_msgs::Marker::ADD;
  path_node_marker_.pose.orientation.w = 1.0;
  path_node_marker_.scale.x = 0.3;
  path_node_marker_.scale.y = 0.3;
  path_node_marker_.scale.z = 0.3;
  path_node_marker_.color.a = 1.0;
  path_node_marker_.color.r = 1.0;
  path_node_marker_.color.g = 0.0;
  path_node_marker_.color.b = 0.0;
}

void KinoAstar::localCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
  // insert local cloud to kdtree_
  // std::cout << "local cloud callback" << std::endl;
  obs_.clear();
  PCLPointCloud latest_cloud;
  pcl::fromROSMsg(*msg, latest_cloud);
  for (size_t i = 0; i < latest_cloud.points.size(); ++i)
  {
    Eigen::Vector3d pt;
    pt(0) = latest_cloud.points[i].x;
    pt(1) = latest_cloud.points[i].y;
    pt(2) = latest_cloud.points[i].z;
    obs_.push_back(pt);
  }
  PCLPointCloud::Ptr cloud_ptr = boost::make_shared<PCLPointCloud>(toPCL(obs_));
  kdtree_.setInputCloud(cloud_ptr);
}

void KinoAstar::init()
{
  path_node_pool_.resize(allocated_node_num_);
  for (int i = 0; i < allocated_node_num_; i++)
  {
    path_node_pool_[i] = new KinoAstarNode();
  }

  use_node_num_ = 0;
  grid_map_->getRegion(origin_, map_size_);
  resolution_ = grid_map_->getResolution();
  this->inv_resolution_ = 1.0 / resolution_;
  this->tie_breaker_ = 1.0 + (3 / 1e4);

  std::cout << "origin: " << origin_.transpose() << std::endl;
  std::cout << "map_size: " << map_size_.transpose() << std::endl;
  std::cout << "allocated_node_num: " << allocated_node_num_ << std::endl;
}

void KinoAstar::setGridMap(GridMap::Ptr& grid_map)
{
  this->grid_map_ = grid_map;
}

int KinoAstar::search(Eigen::Vector3d start_pt, Eigen::Vector3d start_vel, Eigen::Vector3d end_pt,
                      Eigen::Vector3d end_vel, std::vector<Eigen::Vector3d>& path)
{
  ros::Time start_time = ros::Time::now();
  double inv_acc_res_ = 1.0 / acc_res_;
  double optimal_time = inf;
  KinoAstarNodePtr start_node = path_node_pool_[use_node_num_];
  start_node->position = start_pt;
  start_node->velocity = start_vel;
  start_node->index = posToIndex(start_pt);
  start_node->g_cost = 0.0;
  start_node->f_cost = lambda_heu_ * getHeuristicCost(start_pt, start_vel, end_pt, end_vel, optimal_time);
  use_node_num_++;

  open_list_.push(start_node);
  expanded_list_.insert(start_node->index, start_node);
  start_node->node_state = IN_OPEN_LIST_;

  std::vector<Eigen::Vector3d> path_nodes_list;

  while (!open_list_.empty())
  {
    KinoAstarNodePtr current_node = open_list_.top();
    open_list_.pop();
    close_list_.insert(current_node->index, current_node);
    current_node->node_state = IN_CLOSE_LIST_;
    current_node->duration = sample_tau_;
    // debug
    // std::cout << "current_node: " << current_node->position.transpose() << std::endl;

    // check if near goal
    if ((current_node->position - end_pt).norm() < goal_tolerance_)
    {
      double tmp_cost = lambda_heu_ * (current_node->position, current_node->velocity, end_pt, end_vel, optimal_time);
      bool shot_path_found =
          computeShotTraj(current_node->position, current_node->velocity, end_pt, end_vel, optimal_time);
      // std::cout << "optimal_time: " << optimal_time << std::endl;
      if (shot_path_found)
      {
        ros::Time end_time = ros::Time::now();
        std::cout << "kinodynamic path found, time cost: " << (end_time - start_time).toSec() << std::endl;
        std::cout << "use_node_num: " << use_node_num_ << std::endl;
        std::cout << "total_cost_J: " << current_node->g_cost + tmp_cost << std::endl;
        current_node->duration = optimal_time;
        std::vector<KinoAstarNodePtr> path_pool = retrievePath(current_node, path_nodes_list);
        path_nodes_list.push_back(end_pt);
        visPathNodes(path_nodes_list);
        switch (collision_check_type_)
        {
          case 1:  // grid map check
          {
            samplePath(path_pool, path);
            break;
          }
          case 2:  // local cloud check
          {
            sampleEllipsoid(path_pool, path, rot_list);
            visEllipsoid(path, rot_list);
            break;
          }
        }
        return REACH_END;
      }
      else if (current_node->parent != NULL)
      {
        // std::cout << "near end!" << std::endl;
      }
      else
      {
        std::cout << "no shot path found, end searching" << std::endl;
        return NO_PATH_FOUND;
      }
      // else continue;
    }

    // expand current node
    // (2r+1)^3 sample
    for (double ax = -max_accel_; ax <= max_accel_ + 1e-3; ax += inv_acc_res_ * max_accel_)
      for (double ay = -max_accel_; ay <= max_accel_ + 1e-3; ay += inv_acc_res_ * max_accel_)
        for (double az = -max_accel_; az <= max_accel_ + 1e-3; az += inv_acc_res_ * max_accel_)
        {
          Eigen::Vector3d ut;
          ut << ax, ay, az;
          Eigen::Matrix<double, 6, 1> x0;
          x0.head(3) = current_node->position;
          x0.tail(3) = current_node->velocity;
          Eigen::Matrix<double, 6, 1> xt;

          int segment_num = std::floor(sample_tau_ / step_size_);
          bool flag = false;
          bool collision_flag = false;
          for (int i = 0; i <= segment_num; i++)
          {
            double t = i * step_size_;
            StateTransit(x0, xt, ut, t);
            Eigen::Vector3d tmp_pos = xt.head(3);
            // check collision and if out of map
            if (grid_map_->isInMap(tmp_pos) == false)
            {
              flag = true;
              break;
            }
            // check collision
            switch (collision_check_type_)
            {
              case 1:  // grid map check
                if (grid_map_->getInflateOccupancy(tmp_pos) == 1)
                {
                  collision_flag = true;
                  break;
                }

              case 2:  // local cloud check
                if (isCollisionFree(tmp_pos, ut) == false)
                {
                  collision_flag = true;
                  break;
                }
            }
            if (collision_flag)
            {
              flag = true;
              break;
            }
            // check velocity limit
            if (xt.tail(3)(0) < -max_vel_ || xt.tail(3)(0) > max_vel_ || xt.tail(3)(1) < -max_vel_ ||
                xt.tail(3)(1) > max_vel_ || xt.tail(3)(2) < -max_vel_ || xt.tail(3)(2) > max_vel_)
            {
              flag = true;
              break;
            }
          }
          if (flag)
            continue;

          StateTransit(x0, xt, ut, sample_tau_);
          // std::cout << "xt: " << xt.head(3).transpose() << std::endl;
          // std::cout << "index: " << posToIndex(xt.head(3)).transpose() << std::endl;
          // check if in close_list_
          if (close_list_.find(posToIndex(xt.head(3))) != NULL)
          {
            continue;
          }
          // check if in expanded_list_
          else if (expanded_list_.find(posToIndex(xt.head(3))) == NULL)
          {
            KinoAstarNodePtr pro_node = path_node_pool_[use_node_num_];
            pro_node->position = xt.head(3);
            pro_node->velocity = xt.tail(3);
            pro_node->index = posToIndex(xt.head(3));
            pro_node->g_cost = current_node->g_cost + (ut.dot(ut) + rou_) * sample_tau_;
            pro_node->f_cost = pro_node->g_cost + lambda_heu_ * getHeuristicCost(pro_node->position, pro_node->velocity,
                                                                                 end_pt, end_vel, optimal_time);
            pro_node->parent = current_node;
            pro_node->input = ut;
            pro_node->duration = sample_tau_;
            pro_node->node_state = IN_OPEN_LIST_;
            use_node_num_++;
            open_list_.push(pro_node);
            expanded_list_.insert(pro_node->index, pro_node);

            // check if use_node_num reach the max_node_num
            if (use_node_num_ >= allocated_node_num_)
            {
              std::cout << "reach max node num, end searching" << std::endl;
              return NO_PATH_FOUND;
            }
          }
          else
          {
            // pruning, if in same grid, check if g_cost is smaller and update
            double tmp_g_cost = current_node->g_cost + (ut.dot(ut) + rou_) * sample_tau_;
            KinoAstarNodePtr old_node = expanded_list_.find(posToIndex(xt.head(3)));
            if (tmp_g_cost < old_node->g_cost)
            {
              old_node->position = xt.head(3);
              old_node->velocity = xt.tail(3);
              // old_node->index = posToIndex(xt.head(3));
              old_node->g_cost = tmp_g_cost;
              old_node->f_cost =
                  old_node->g_cost +
                  lambda_heu_ * getHeuristicCost(old_node->position, old_node->velocity, end_pt, end_vel, optimal_time);
              old_node->parent = current_node;
              old_node->input = ut;
            }
          }
        }
  }

  std::cout << "open_list is empty, end searching, no path found" << std::endl;
  return NO_PATH_FOUND;
}

void KinoAstar::reset()
{
  for (int i = 0; i < use_node_num_; i++)
  {
    KinoAstarNodePtr node = path_node_pool_[i];
    node->g_cost = inf;
    node->f_cost = inf;
    node->duration = inf;
    node->input = Eigen::Vector3d::Zero();
    node->parent = NULL;
    node->node_state = NOT_EXPANDED_;
  }

  std::priority_queue<KinoAstarNodePtr, std::vector<KinoAstarNodePtr>, KinoAstarNodeComparator> empty_queue;
  open_list_.swap(empty_queue);
  close_list_.clear();
  expanded_list_.clear();
  rot_list.clear();

  shot_coef_ = Eigen::Matrix<double, 3, 4>::Zero();
  vel_coef_ = Eigen::Matrix<double, 3, 4>::Zero();
  acc_coef_ = Eigen::Matrix<double, 3, 4>::Zero();

  path_node_pool_.clear();

  use_node_num_ = 0;
}

Eigen::Vector3i KinoAstar::posToIndex(Eigen::Vector3d pos)
{
  Eigen::Vector3i index;
  for (int i = 0; i < 3; i++)
  {
    index(i) = std::floor((pos(i) - origin_(i)) * inv_resolution_);
  }
  return index;
}

double KinoAstar::getHeuristicCost(Eigen::Vector3d x1, Eigen::Vector3d v1, Eigen::Vector3d x2, Eigen::Vector3d v2,
                                   double& optimal_time)
{
  Eigen::Vector3d dp = x2 - x1;
  double optimal_cost = inf;

  double a = -36 * dp.dot(dp);
  double b = 24 * dp.dot(v1 + v2);
  double c = -4 * (v1.dot(v1) + v1.dot(v2) + v2.dot(v2));
  double d = 0;
  double e = rou_;

  std::vector<double> dts = quartic(e, d, c, b, a);
  double T_bar = ((x1 - x2).lpNorm<Eigen::Infinity>() / max_vel_);
  for (int i = 0; i < dts.size(); i++)
  {
    double t = dts[i];
    double tmp_cost = a / (-3 * t * t * t) + b / (-2 * t * t) + c / (-1 * t) + e * t;
    if (tmp_cost < optimal_cost && t > T_bar && tmp_cost > 0)
    {
      optimal_cost = tmp_cost;
      optimal_time = t;
    }
  }
  return tie_breaker_ * optimal_cost;
}

std::vector<double> KinoAstar::cubic(double a, double b, double c, double d)
{
  std::vector<double> dts;

  double a2 = b / a;
  double a1 = c / a;
  double a0 = d / a;

  double Q = (3 * a1 - a2 * a2) / 9;
  double R = (9 * a1 * a2 - 27 * a0 - 2 * a2 * a2 * a2) / 54;
  double D = Q * Q * Q + R * R;
  if (D > 0)
  {
    double S = std::cbrt(R + sqrt(D));
    double T = std::cbrt(R - sqrt(D));
    dts.push_back(-a2 / 3 + (S + T));
    return dts;
  }
  else if (D == 0)
  {
    double S = std::cbrt(R);
    dts.push_back(-a2 / 3 + S + S);
    dts.push_back(-a2 / 3 - S);
    return dts;
  }
  else
  {
    double theta = acos(R / sqrt(-Q * Q * Q));
    dts.push_back(2 * sqrt(-Q) * cos(theta / 3) - a2 / 3);
    dts.push_back(2 * sqrt(-Q) * cos((theta + 2 * M_PI) / 3) - a2 / 3);
    dts.push_back(2 * sqrt(-Q) * cos((theta + 4 * M_PI) / 3) - a2 / 3);
    return dts;
  }
}

std::vector<double> KinoAstar::quartic(double a, double b, double c, double d, double e)
{
  std::vector<double> dts;

  double a3 = b / a;
  double a2 = c / a;
  double a1 = d / a;
  double a0 = e / a;

  std::vector<double> ys = cubic(1, -a2, a1 * a3 - 4 * a0, 4 * a2 * a0 - a1 * a1 - a3 * a3 * a0);
  double y1 = ys.front();
  double r = a3 * a3 / 4 - a2 + y1;
  if (r < 0)
    return dts;

  double R = sqrt(r);
  double D, E;
  if (R != 0)
  {
    D = sqrt(0.75 * a3 * a3 - R * R - 2 * a2 + 0.25 * (4 * a3 * a2 - 8 * a1 - a3 * a3 * a3) / R);
    E = sqrt(0.75 * a3 * a3 - R * R - 2 * a2 - 0.25 * (4 * a3 * a2 - 8 * a1 - a3 * a3 * a3) / R);
  }
  else
  {
    D = sqrt(0.75 * a3 * a3 - 2 * a2 + 2 * sqrt(y1 * y1 - 4 * a0));
    E = sqrt(0.75 * a3 * a3 - 2 * a2 - 2 * sqrt(y1 * y1 - 4 * a0));
  }

  if (!std::isnan(D))
  {
    dts.push_back(-a3 / 4 + R / 2 + D / 2);
    dts.push_back(-a3 / 4 + R / 2 - D / 2);
  }
  if (!std::isnan(E))
  {
    dts.push_back(-a3 / 4 - R / 2 + E / 2);
    dts.push_back(-a3 / 4 - R / 2 - E / 2);
  }

  return dts;
}

bool KinoAstar::computeShotTraj(Eigen::Vector3d x1, Eigen::Vector3d v1, Eigen::Vector3d x2, Eigen::Vector3d v2,
                                double optimal_time)
{
  double td = optimal_time;
  Eigen::Vector3d dp = x2 - x1;
  Eigen::Vector3d dv = v2 - v1;
  shot_coef_.col(0) = x1;
  shot_coef_.col(1) = v1;
  shot_coef_.col(2) = 0.5 * (6 / (td * td) * (dp - v1 * td) - 2 * dv / td);
  shot_coef_.col(3) = 1.0 / 6.0 * (-12 / (td * td * td) * (dp - v1 * td) + 6 * dv / (td * td));

  Eigen::Matrix<double, 4, 4> Transit_v;
  Transit_v << 0, 0, 0, 0, 1, 0, 0, 0, 0, 2, 0, 0, 0, 0, 3, 0;
  Eigen::Matrix<double, 4, 4> Transit_a;
  Transit_a << 0, 0, 0, 0, 1, 0, 0, 0, 0, 2, 0, 0, 0, 0, 0, 0;
  vel_coef_ = shot_coef_ * Transit_v;
  acc_coef_ = vel_coef_ * Transit_a;

  Eigen::Matrix<double, 4, 1> t_vector;

  int segment_num = std::floor(td / step_size_);
  double curr_t = 0.0;
  for (int j = 0; j <= segment_num; j++)
  {
    curr_t = j * step_size_;
    for (int i = 0; i < 4; i++)
    {
      t_vector(i) = pow(curr_t, i);
    }

    Eigen::Vector3d shot_pos = shot_coef_ * t_vector;
    Eigen::Vector3d shot_vel = vel_coef_ * t_vector;
    Eigen::Vector3d shot_acc = acc_coef_ * t_vector;
    // check collision
    if (grid_map_->getInflateOccupancy(shot_pos))
    {
      return false;
    }
    // only check collision, vel and acc limit by limit T_
    // // check velocity limit
    // if (shot_vel(0) > max_vel_ || shot_vel(0) < -max_vel_ ||
    //     shot_vel(1) > max_vel_ || shot_vel(1) < -max_vel_ ||
    //     shot_vel(2) > max_vel_ || shot_vel(2) < -max_vel_)
    // {
    //   return false;
    // }
    // // check acceleration limit
    // if (shot_acc(0) > max_accel_ || shot_acc(0) < -max_accel_ ||
    //     shot_acc(1) > max_accel_ || shot_acc(1) < -max_accel_ ||
    //     shot_acc(2) > max_accel_ || shot_acc(2) < -max_accel_)
    // {
    //   return false;
    // }
  }
  return true;
}

std::vector<KinoAstarNodePtr> KinoAstar::retrievePath(KinoAstarNodePtr end_node,
                                                      std::vector<Eigen::Vector3d>& path_nodes_list)
{
  KinoAstarNodePtr current_node = end_node;
  std::vector<KinoAstarNodePtr> path_nodes;

  while (current_node->parent != NULL)
  {
    path_nodes.push_back(current_node);
    path_nodes_list.push_back(current_node->position);
    current_node = current_node->parent;
  }
  path_nodes.push_back(current_node);
  path_nodes_list.push_back(current_node->position);
  std::reverse(path_nodes.begin(), path_nodes.end());
  std::reverse(path_nodes_list.begin(), path_nodes_list.end());
  return path_nodes;
}

void KinoAstar::samplePath(std::vector<KinoAstarNodePtr> path_pool, std::vector<Eigen::Vector3d>& path)
{
  if (path_pool.size() != 1)
  {
    for (int i = 0; i < path_pool.size() - 1; i++)
    {
      KinoAstarNodePtr curr_node = path_pool[i];
      KinoAstarNodePtr next_node = path_pool[i + 1];
      double curr_t = 0.0;
      Eigen::Matrix<double, 6, 1> x0, xt;
      x0 << curr_node->position, curr_node->velocity;

      int segment_num = std::floor(curr_node->duration / step_size_);
      for (int j = 0; j < segment_num; j++)
      {
        curr_t = j * step_size_;
        StateTransit(x0, xt, next_node->input, curr_t);
        path.push_back(xt.head(3));
      }
      StateTransit(x0, xt, next_node->input, curr_node->duration);
      if ((xt.head(3) - next_node->position).norm() > 1e-2)
      {
        std::cerr << "error in sample!" << std::endl;
      }
    }
    KinoAstarNodePtr last_node = path_pool.back();
    double td = last_node->duration;

    Eigen::Matrix<double, 4, 1> t_vector;

    int segment_num = std::floor(td / step_size_);
    double curr_t = 0.0;
    for (int j = 0; j <= segment_num; j++)
    {
      curr_t = j * step_size_;
      for (int i = 0; i < 4; i++)
      {
        t_vector(i) = pow(curr_t, i);
      }

      Eigen::Vector3d shot_pos = shot_coef_ * t_vector;
      path.push_back(shot_pos);
    }
  }
  else
  {
    KinoAstarNodePtr last_node = path_pool.back();
    double td = last_node->duration;

    Eigen::Matrix<double, 4, 1> t_vector;

    int segment_num = std::floor(td / step_size_);
    double curr_t = 0.0;
    for (int j = 0; j <= segment_num; j++)
    {
      curr_t = j * step_size_;
      for (int i = 0; i < 4; i++)
      {
        t_vector(i) = pow(curr_t, i);
      }

      Eigen::Vector3d shot_pos = shot_coef_ * t_vector;
      path.push_back(shot_pos);
    }
  }
}

void KinoAstar::sampleEllipsoid(std::vector<KinoAstarNodePtr> path_pool, std::vector<Eigen::Vector3d>& path,
                                std::vector<Eigen::Matrix3d>& rot_list)
{
  if (path_pool.size() != 1)
  {
    for (int i = 0; i < path_pool.size() - 1; i++)
    {
      KinoAstarNodePtr curr_node = path_pool[i];
      KinoAstarNodePtr next_node = path_pool[i + 1];
      double curr_t = 0.0;
      Eigen::Matrix<double, 6, 1> x0, xt;
      x0 << curr_node->position, curr_node->velocity;

      int segment_num = std::floor(curr_node->duration / step_size_);
      for (int j = 0; j < segment_num; j++)
      {
        curr_t = j * step_size_;
        StateTransit(x0, xt, next_node->input, curr_t);
        path.push_back(xt.head(3));

        Eigen::Vector3d b3 = (next_node->input + 9.81 * Eigen::Vector3d::UnitZ()).normalized();
        Eigen::Vector3d c1(cos(0), sin(0), 0);
        Eigen::Vector3d b2 = b3.cross(c1).normalized();
        Eigen::Vector3d b1 = b2.cross(b3).normalized();

        Eigen::Matrix3d Rot;
        Rot << b1, b2, b3;
        rot_list.push_back(Rot);
      }
    }

    KinoAstarNodePtr last_node = path_pool.back();
    double td = last_node->duration;

    Eigen::Matrix<double, 4, 1> t_vector;

    int segment_num = std::floor(td / step_size_);
    double curr_t = 0.0;
    for (int j = 0; j <= segment_num; j++)
    {
      curr_t = j * step_size_;
      for (int i = 0; i < 4; i++)
      {
        t_vector(i) = pow(curr_t, i);
      }

      Eigen::Vector3d shot_pos = shot_coef_ * t_vector;
      Eigen::Vector3d shot_acc = acc_coef_ * t_vector;
      path.push_back(shot_pos);

      Eigen::Vector3d b3 = (shot_acc + 9.81 * Eigen::Vector3d::UnitZ()).normalized();
      Eigen::Vector3d c1(cos(0), sin(0), 0);
      Eigen::Vector3d b2 = b3.cross(c1).normalized();
      Eigen::Vector3d b1 = b2.cross(b3).normalized();

      Eigen::Matrix3d Rot;
      Rot << b1, b2, b3;
      rot_list.push_back(Rot);
    }
  }
  else
  {
    KinoAstarNodePtr last_node = path_pool.back();
    double td = last_node->duration;

    Eigen::Matrix<double, 4, 1> t_vector;

    int segment_num = std::floor(td / step_size_);
    double curr_t = 0.0;
    for (int j = 0; j <= segment_num; j++)
    {
      curr_t = j * step_size_;
      for (int i = 0; i < 4; i++)
      {
        t_vector(i) = pow(curr_t, i);
      }

      Eigen::Vector3d shot_pos = shot_coef_ * t_vector;
      Eigen::Vector3d shot_acc = acc_coef_ * t_vector;
      path.push_back(shot_pos);
      Eigen::Vector3d b3 = (shot_acc + 9.81 * Eigen::Vector3d::UnitZ()).normalized();
      Eigen::Vector3d c1(cos(0), sin(0), 0);
      Eigen::Vector3d b2 = b3.cross(c1).normalized();
      Eigen::Vector3d b1 = b2.cross(b3).normalized();

      Eigen::Matrix3d Rot;
      Rot << b1, b2, b3;
      rot_list.push_back(Rot);
    }
  }
}

void KinoAstar::StateTransit(Eigen::Matrix<double, 6, 1>& x0, Eigen::Matrix<double, 6, 1>& xt, Eigen::Vector3d ut,
                             double t)
{
  Eigen::Matrix<double, 6, 6> e_At = Eigen::Matrix<double, 6, 6>::Identity();
  for (int i = 0; i < 3; i++)
  {
    e_At(i, 3 + i) = t;
  }
  // bug fixed: Integral should set zero
  Eigen::Matrix<double, 6, 3> Integral = Eigen::Matrix<double, 6, 3>::Zero();
  for (int i = 0; i < 6; i++)
  {
    if (i < 3)
      Integral(i, i) = 0.5 * t * t;
    else
      Integral(i, i - 3) = t;
  }

  xt = e_At * x0 + Integral * ut;
}

void KinoAstar::visPathNodes(std::vector<Eigen::Vector3d>& path_nodes_list)
{
  path_node_marker_.points.clear();
  for (int i = 0; i < path_nodes_list.size(); i++)
  {
    // publishes the path nodes
    geometry_msgs::Point pt;
    pt.x = path_nodes_list[i](0);
    pt.y = path_nodes_list[i](1);
    pt.z = path_nodes_list[i](2);
    path_node_marker_.points.push_back(pt);
  }
  path_node_pub_.publish(path_node_marker_);
}

void KinoAstar::visEllipsoid(std::vector<Eigen::Vector3d>& path_nodes_list, std::vector<Eigen::Matrix3d>& rot_list)
{
  for (int i = 0; i < path_nodes_list.size(); i++)
  {
    // publishes the ellipsoid
    visualization_msgs::Marker elliposid_marker_;
    elliposid_marker_.header.frame_id = "world";
    elliposid_marker_.header.stamp = ros::Time::now();
    elliposid_marker_.ns = "elliposid";
    elliposid_marker_.id = i;
    elliposid_marker_.type = visualization_msgs::Marker::SPHERE;
    elliposid_marker_.action = visualization_msgs::Marker::ADD;
    elliposid_marker_.color.a = 0.5;
    elliposid_marker_.color.r = 1.0;
    elliposid_marker_.color.g = 0.0;
    elliposid_marker_.color.b = 1.0;
    elliposid_marker_.scale.x = 0.4 * 2;
    elliposid_marker_.scale.y = 0.4 * 2;
    elliposid_marker_.scale.z = 0.1 * 2;

    elliposid_marker_.pose.position.x = path_nodes_list[i](0);
    elliposid_marker_.pose.position.y = path_nodes_list[i](1);
    elliposid_marker_.pose.position.z = path_nodes_list[i](2);

    Eigen::Quaterniond q(rot_list[i]);
    elliposid_marker_.pose.orientation.x = q.x();
    elliposid_marker_.pose.orientation.y = q.y();
    elliposid_marker_.pose.orientation.z = q.z();
    elliposid_marker_.pose.orientation.w = q.w();

    elliposid_pub_.publish(elliposid_marker_);
  }
}

bool KinoAstar::isCollisionFree(Eigen::Vector3d pt, Eigen::Vector3d acc)
{
  // check collision with local cloud
  Eigen::Vector3d b3 = (acc + 9.81 * Eigen::Vector3d::UnitZ()).normalized();
  Eigen::Vector3d c1(cos(0), sin(0), 0);
  Eigen::Vector3d b2 = b3.cross(c1).normalized();
  Eigen::Vector3d b1 = b2.cross(b3).normalized();

  Eigen::Matrix3d Rot;
  Rot << b1, b2, b3;

  Eigen::Matrix3d P = Eigen::Matrix3d::Zero();
  P(0, 0) = robot_r_;
  P(1, 1) = robot_r_;
  P(2, 2) = robot_h_;

  Eigen::Matrix3d E = Rot * P * Rot.transpose();

  pcl::PointXYZ searchPoint;
  std::vector<int> pointIdxRadiusSearch;
  std::vector<float> pointRadiusSquaredDistance;

  searchPoint.x = pt(0);
  searchPoint.y = pt(1);
  searchPoint.z = pt(2);

  float radius = robot_r_ + 1e-1;
  if (kdtree_.radiusSearch(searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0)
  {
    for (size_t i = 0; i < pointIdxRadiusSearch.size(); ++i)
    {
      Eigen::Vector3d tmp_pt = E.inverse() * (obs_[pointIdxRadiusSearch[i]] - pt);
      if (tmp_pt.norm() <= 1.0)
        return false;
    }
  }
  return true;
}

// Convert obstacle points into pcl point cloud
PCLPointCloud KinoAstar::toPCL(const std::vector<Eigen::Vector3d>& obs)
{
  PCLPointCloud cloud;
  cloud.width = obs.size();
  cloud.height = 1;
  cloud.points.resize(cloud.width * cloud.height);
  for (unsigned int i = 0; i < obs.size(); i++)
  {
    cloud.points[i].x = obs[i](0);
    cloud.points[i].y = obs[i](1);
    cloud.points[i].z = obs[i](2);
  }
  return cloud;
}

KinoAstar::~KinoAstar()
{
  for (int i = 0; i < allocated_node_num_; i++)
  {
    delete path_node_pool_[i];
  }
}

}  // namespace path_searching