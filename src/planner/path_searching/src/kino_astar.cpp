#include <path_searching/kino_astar.h>

namespace path_searching
{

void KinoAstar::setParam(ros::NodeHandle& nh)
{
  nh.param("kino_astar/allocated_node_num", allocated_node_num_, 100000);
  nh.param("kino_astar/rou_time", rou_, 1.0);
  nh.param("kino_astar/lambda_heu", lambda_heu_, 2.0);
  nh.param("kino_astar/goal_tolerance", goal_tolerance_, 2.0);
  nh.param("kino_astar/time_step_size", step_size_, 0.1);
  nh.param("kino_astar/max_velocity", max_vel_, 5.0);
  nh.param("kino_astar/max_accelration", max_accel_, 7.0);
  nh.param("kino_astar/acc_resolution", acc_res_, 2.0);
  nh.param("kino_astar/sample_tau", sample_tau_, 0.5);
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

int KinoAstar::search(Eigen::Vector3d start_pt, Eigen::Vector3d start_vel,
         Eigen::Vector3d end_pt, Eigen::Vector3d end_vel,
         std::vector<Eigen::Vector3d>& path)
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

  while (!open_list_.empty())
  {
    KinoAstarNodePtr current_node = open_list_.top();
    open_list_.pop();
    close_list_.insert(current_node->index, current_node);
    current_node->node_state = IN_CLOSE_LIST_;
    // debug
    // std::cout << "current_node: " << current_node->position.transpose() << std::endl;

    // check if near goal
    if ((current_node->position - end_pt).norm() < goal_tolerance_)
    {
      getHeuristicCost(current_node->position, current_node->velocity, end_pt, end_vel, optimal_time);
      bool shot_path_found = computeShotTraj(current_node->position, current_node->velocity, end_pt, end_vel, optimal_time);
      // std::cout << "optimal_time: " << optimal_time << std::endl;
      if (shot_path_found)
      {
        ros::Time end_time = ros::Time::now();
        std::cout << "kinodynamic path found, time cost: " << (end_time - start_time).toSec() << std::endl;
        current_node->duration = optimal_time;
        std::vector<KinoAstarNodePtr> path_pool = retrievePath(current_node);
        samplePath(path_pool, path);
        return REACH_END;
      }
      else continue;
    }

    // expand current node
    // (2r+1)^3 sample
    for (double ax = -max_accel_; ax <= max_accel_; ax += inv_acc_res_ * max_accel_)
      for (double ay = -max_accel_; ay <= max_accel_; ay += inv_acc_res_ * max_accel_)
        for (double az = -max_accel_; az <= max_accel_; az += inv_acc_res_ * max_accel_)
        {
          Eigen::Vector3d ut = Eigen::Vector3d(ax, ay, az);
          Eigen::Matrix<double, 6, 1> x0;
          x0.head(3) = current_node->position;
          x0.tail(3) = current_node->velocity;
          Eigen::Matrix<double, 6, 1> xt;
          current_node->duration = sample_tau_;
          

          int segment_num = std::floor(sample_tau_ / step_size_);
          bool flag = false;
          for (int i = 0; i <= segment_num; i++)
          {
            double t = i * step_size_;
            StateTransit(x0, xt, ut, t);
            // check collision and if out of map
            if (grid_map_->getInflateOccupancy(xt.head(3))) 
              {
                flag = true;
                break;
              }
            // check velocity limit
            if (xt.tail(3)(0) < -max_vel_ || xt.tail(3)(0) > max_vel_ 
                || xt.tail(3)(1) < -max_vel_ || xt.tail(3)(1) > max_vel_ 
                || xt.tail(3)(2) < -max_vel_ || xt.tail(3)(2) > max_vel_)
              {
                flag = true;
                break;
              }
          }
          if (flag) continue;

          StateTransit(x0, xt, ut, sample_tau_);
          // std::cout << "xt: " << xt.head(3).transpose() << std::endl;
          // std::cout << "index: " << posToIndex(xt.head(3)).transpose() << std::endl;
          // check if in close_list_
          if (close_list_.find(posToIndex(xt.head(3))) != NULL)
            continue;
          // check if in expanded_list_
          else if (expanded_list_.find(posToIndex(xt.head(3))) == NULL)
          {
            KinoAstarNodePtr pro_node = path_node_pool_[use_node_num_];
            pro_node->position = xt.head(3);
            // std::cout << "pro_node: " << pro_node->position.transpose() << std::endl;
            pro_node->velocity = xt.tail(3);
            pro_node->index = posToIndex(xt.head(3));
            pro_node->g_cost = current_node->g_cost + (ut.dot(ut) + rou_) * sample_tau_;
            pro_node->f_cost = pro_node->g_cost + lambda_heu_ * getHeuristicCost(pro_node->position, pro_node->velocity, end_pt, end_vel, optimal_time);
            pro_node->parent = current_node;
            pro_node->input = ut;
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
              old_node->index = posToIndex(xt.head(3));
              old_node->g_cost = tmp_g_cost;
              old_node->f_cost = old_node->g_cost + lambda_heu_ * getHeuristicCost(old_node->position, old_node->velocity, end_pt, end_vel, optimal_time);
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
    node->parent = nullptr;
    node->node_state = NOT_EXPANDED_;
  }

  std::priority_queue<KinoAstarNodePtr, std::vector<KinoAstarNodePtr>, KinoAstarNodeComparator> empty_queue;
  open_list_.swap(empty_queue);
  close_list_.clear();
  expanded_list_.clear();

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

double KinoAstar::getHeuristicCost(Eigen::Vector3d x1, Eigen::Vector3d v1,
                        Eigen::Vector3d x2, Eigen::Vector3d v2,
                        double &optimal_time)
{
  Eigen::Vector3d dp = x2 - x1;
  double optimal_cost = inf;

  double a = -36 * dp.dot(dp);
  double b = 24 * dp.dot(v1 + v2);
  double c = -4 * (v1.dot(v1) + v1.dot(v2) + v2.dot(v2));
  double d = 0;
  double e = rou_;

  std::vector<double> dts = quartic(e, d, c, b, a);
  for (int i=0; i < dts.size(); i++)
  {
    double t = dts[i];
    double tmp_cost = a / (-3 * t * t * t) + b / (-2 * t * t) + c / (-1 * t) + e * t;
    if (abs(tmp_cost) < optimal_cost && t > 0)
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

bool KinoAstar::computeShotTraj(Eigen::Vector3d x1, Eigen::Vector3d v1,
                            Eigen::Vector3d x2, Eigen::Vector3d v2,
                            double optimal_time)
{
  double td = optimal_time;
  Eigen::Vector3d dp = x2 - x1;
  Eigen::Vector3d dv = v2 - v1;
  shot_coef_.col(0) = x1;
  shot_coef_.col(1) = v1;
  shot_coef_.col(2) = 0.5 * (6 / (td * td) * (dp - v1 * td) - 2 * dv / td);
  shot_coef_.col(3) = 1.0 / 6.0 * (-12 / (td * td *td) * (dp - v1 * td) + 6 * dv / (td * td));

  Eigen::Matrix<double, 4, 4> Transit_v;
  Transit_v << 0, 0, 0, 0, 
             1, 0, 0, 0,
             0, 2, 0, 0, 
             0, 0, 3, 0;
  Eigen::Matrix<double, 4, 4> Transit_a;
  Transit_a << 0, 0, 0, 0, 
             1, 0, 0, 0,
             0, 2, 0, 0, 
             0, 0, 0, 0;
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
    // check velocity limit
    if (shot_vel(0) > max_vel_ || shot_vel(0) < -max_vel_ || 
        shot_vel(1) > max_vel_ || shot_vel(1) < -max_vel_ || 
        shot_vel(2) > max_vel_ || shot_vel(2) < -max_vel_)
    {
      return false;
    }
    // check acceleration limit
    if (shot_acc(0) > max_accel_ || shot_acc(0) < -max_accel_ || 
        shot_acc(1) > max_accel_ || shot_acc(1) < -max_accel_ || 
        shot_acc(2) > max_accel_ || shot_acc(2) < -max_accel_)
    {
      return false;
    }
  }
  return true;
}

std::vector<KinoAstarNodePtr> KinoAstar::retrievePath(KinoAstarNodePtr end_node)
{
  KinoAstarNodePtr current_node = end_node;
  std::vector<KinoAstarNodePtr> path_nodes;
  while (current_node->parent != NULL)
  {
    path_nodes.push_back(current_node);
    current_node = current_node->parent;
  }
  path_nodes.push_back(current_node);
  std::reverse(path_nodes.begin(), path_nodes.end());
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

void KinoAstar::StateTransit(Eigen::Matrix<double, 6, 1> &x0, Eigen::Matrix<double, 6, 1> &xt, Eigen::Vector3d ut, double t)
{
  xt = Eigen::Matrix<double, 6, 1>::Zero();
  Eigen::Matrix<double, 6, 6> e_At = Eigen::Matrix<double, 6, 6>::Identity();
  for (int i = 0; i < 3; i++)
  {
    e_At(i, 3 + i) = t;
  }

  Eigen::Matrix<double, 6, 3> Integral;
  for (int i = 0; i < 6; i++)
  {
    if (i < 3) Integral(i, i) = 0.5 * t * t;
    else Integral(i, i - 3) = t;
  }

  xt = e_At * x0 + Integral * ut;

}

KinoAstar::~KinoAstar()
{
  for (int i = 0; i < allocated_node_num_; i++)
  {
    delete path_node_pool_[i];
  }
}

} // namespace path_searching