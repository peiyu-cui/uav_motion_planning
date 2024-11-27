#include <path_searching/a_star.h>

namespace path_searching
{

void Astar::setParam(ros::NodeHandle& nh)
{
  nh.param("astar/resolution", resolution_, 0.1);
  nh.param("astar/lambda_heu", lambda_, 1.0);
  nh.param("astar/allocated_node_num", allocated_node_num_, 100000);
}

void Astar::init()
{
  this->inv_resolution_ = 1.0 / resolution_;
  this->tie_breaker_ = 1.0 + 1e-4;              // default value
  path_node_pool_.resize(allocated_node_num_);  // allocate memory for path_node_pool_
  for (int i = 0; i < allocated_node_num_; i++)
  {
    path_node_pool_[i] = new PathNode();
  }

  if (!grid_map_)
  {
    std::cerr << "Error: grid_map_ is not set." << std::endl;
    return;
  }

  use_node_num_ = 0;
  grid_map_->getRegion(origin_, map_size_);
  resolution_ = grid_map_->getResolution();

  std::cout << "origin: " << origin_.transpose() << std::endl;
  std::cout << "map_size: " << map_size_.transpose() << std::endl;
  std::cout << "resolution: " << resolution_ << std::endl;
  std::cout << "lambda_heu: " << lambda_ << std::endl;
  std::cout << "allocated_node_num: " << allocated_node_num_ << std::endl;
}

Astar::~Astar()
{
  for (int i = 0; i < allocated_node_num_; i++)  // free memory for path_node_pool_
  {
    delete path_node_pool_[i];
  }
}

int Astar::search(Eigen::Vector3d start_pt, Eigen::Vector3d end_pt, std::vector<Eigen::Vector3d>& path)
{
  // if end_pt is out of map, return NO_PATH_FOUND
  ros::Time start_time = ros::Time::now();
  if (grid_map_->isInMap(end_pt) == false)
  {
    std::cerr << "Error: end_pt is out of map." << std::endl;
    return NO_PATH_FOUND;
  }
  // push start_node into open_list
  PathNodePtr start_node = path_node_pool_[use_node_num_];  // bug fixed
  start_node->g_cost = 0.0;
  start_node->position = start_pt;
  start_node->parent = NULL;
  start_node->f_cost = lambda_ * getDiagonalHeu(start_pt, end_pt);
  start_node->node_state = IN_OPEN_LIST_;

  open_list_.push(start_node);
  expanded_nodes_.insert(start_node->position, start_node);
  use_node_num_ += 1;

  while (!open_list_.empty())
  {
    // pop node from open_list with lowest f_cost
    // and add it into close_list
    PathNodePtr current_node = open_list_.top();
    open_list_.pop();
    current_node->node_state = IN_CLOSE_LIST_;
    close_list_.insert(current_node->position, current_node);

    // if current_node is end_pt, retrieve path and return success
    if (abs(current_node->position(0) - end_pt(0)) < resolution_ &&
        abs(current_node->position(1) - end_pt(1)) < resolution_ &&
        abs(current_node->position(2) - end_pt(2)) < resolution_)
    {
      ros::Time end_time = ros::Time::now();
      std::cout << "reached end_pt" << std::endl;
      std::cout << "total_time: " << (end_time - start_time).toSec() << " seconds" << std::endl;
      PathNodePtr end_node = current_node;
      std::cout << "use_node_num: " << use_node_num_ << std::endl;
      std::cout << "total_cost:" << end_node->g_cost << std::endl;
      retrievePath(end_node, path);
      return REACH_END;
    }

    for (double x = -resolution_; x <= resolution_; x += resolution_)
      for (double y = -resolution_; y <= resolution_; y += resolution_)
        for (double z = -resolution_; z <= resolution_; z += resolution_)
        {
          Eigen::Vector3d neighbor_pos = current_node->position + Eigen::Vector3d(x, y, z);
          // std::cout << "neighbor_pos: " << neighbor_pos.transpose() << std::endl;

          // if neighbor is out of map, skip it
          if (grid_map_->isInMap(neighbor_pos) == false)
          {
            // std::cout << "out of map" << std::endl;
            continue;
          }

          // if neighbor is obstacle, skip it
          if (grid_map_->getInflateOccupancy(neighbor_pos) == true)
          {
            // std::cout << "known obstacle" << std::endl;
            continue;
          }

          // if neighbor is already in close_list, skip it
          if (close_list_.find(neighbor_pos) != NULL)
          {
            // std::cout << "already in close_list" << std::endl;
            continue;
          }

          double delta_pos = Eigen::Vector3d(x, y, z).norm();
          double tmp_g_cost = current_node->g_cost + delta_pos;
          PathNodePtr tmp_node = expanded_nodes_.find(neighbor_pos);

          if (tmp_node == NULL)
          {
            PathNodePtr neighbor_node = path_node_pool_[use_node_num_];
            use_node_num_ += 1;
            neighbor_node->g_cost = tmp_g_cost;
            neighbor_node->position = neighbor_pos;
            neighbor_node->parent = current_node;
            neighbor_node->f_cost = neighbor_node->g_cost + lambda_ * getDiagonalHeu(neighbor_pos, end_pt);
            neighbor_node->node_state = IN_OPEN_LIST_;
            open_list_.push(neighbor_node);
            expanded_nodes_.insert(neighbor_node->position, neighbor_node);
            if (use_node_num_ >= allocated_node_num_)
            {
              std::cerr << "Error: allocated_node_num is too small." << std::endl;
              return NO_PATH_FOUND;
            }
          }
          else if (tmp_g_cost < tmp_node->g_cost)
          {
            tmp_node->g_cost = tmp_g_cost;
            tmp_node->parent = current_node;
            tmp_node->f_cost = tmp_node->g_cost + lambda_ * getDiagonalHeu(tmp_node->position, end_pt);
          }
        }
  }

  std::cout << "no path found!" << std::endl;
  std::cout << "use_node_num: " << use_node_num_ << std::endl;
  return NO_PATH_FOUND;
}

double Astar::getEuclHeu(Eigen::Vector3d x1, Eigen::Vector3d x2)
{
  return tie_breaker_ * (x1 - x2).norm();
}

double Astar::getDiagonalHeu(Eigen::Vector3d x1, Eigen::Vector3d x2)
{
  double dx = std::abs(x1(0) - x2(0));
  double dy = std::abs(x1(1) - x2(1));
  double dz = std::abs(x1(2) - x2(2));
  double min_xyz = std::min({ dx, dy, dz });
  double h = dx + dy + dz + (std::sqrt(3) - 3) * min_xyz;
  return tie_breaker_ * h;
}

// Eigen::Vector3i Astar::posToIndex(Eigen::Vector3d pos) {
//   Eigen::Vector3i index;
//   index(0) =  std::floor((pos(0) - origin_(0)) * inv_resolution_); // fix bug
//   index(1) =  std::floor((pos(1) - origin_(1)) * inv_resolution_);
//   index(2) =  std::floor((pos(2) - origin_(2)) * inv_resolution_);
//   return index;
// }

void Astar::retrievePath(PathNodePtr end_node, std::vector<Eigen::Vector3d>& path)
{
  PathNodePtr current_node = end_node;
  while (current_node->parent != NULL)
  {
    path.push_back(current_node->position);
    current_node = current_node->parent;
  }
  path.push_back(current_node->position);
  std::reverse(path.begin(), path.end());
}

void Astar::setGridMap(GridMap::Ptr& grid_map)
{
  this->grid_map_ = grid_map;
}

void Astar::reset()
{
  this->expanded_nodes_.clear();
  this->close_list_.clear();
  this->path_.clear();

  std::priority_queue<PathNodePtr, std::vector<PathNodePtr>, NodeComparator> empty_queue;
  this->open_list_.swap(empty_queue);

  use_node_num_ = 0;
  for (int i = 0; i < allocated_node_num_; i++)
  {
    PathNodePtr node = path_node_pool_[i];
    node->parent = NULL;
    node->g_cost = inf;
    node->f_cost = inf;
    node->node_state = NOT_EXPANDED_;
  }
}

}  // namespace path_searching