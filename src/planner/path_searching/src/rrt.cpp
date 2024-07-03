#include <path_searching/rrt.h>

namespace path_searching
{
  void RRT::setParam(ros::NodeHandle& nh) {
    nh.param ("rrt/max_tree_node_num", max_tree_node_num_, 100000);
    nh.param ("rrt/step_length", step_length_, 0.5);
    nh.param ("rrt/max_allowed_time", max_allowed_time_, 5.0);
    nh.param ("rrt/goal_tolerance", tolerance_, 0.1);
    std::cout << "rrt/max_tree_node_num: " << max_tree_node_num_ << std::endl;
    std::cout << "rrt/step_length: " << step_length_ << std::endl;
  }

  void RRT::setGridMap(GridMap::Ptr& grid_map) {
    this->grid_map_ = grid_map;
  }

  void RRT::getWholeTree(std::vector<Eigen::Vector3d> &vertices, 
                         std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> &edges) {
    RRTPathNodePtr start_node = path_node_pool_[0];
    vertices.push_back(start_node->position);
    std::queue<RRTPathNodePtr> bfs_queue;
    bfs_queue.push(start_node);
    while (!bfs_queue.empty()) {
      RRTPathNodePtr curr_node = bfs_queue.front();
      bfs_queue.pop();
      if (curr_node->children.empty()) continue;
      for (RRTPathNodePtr child : curr_node->children) {
        vertices.push_back(child->position);
        edges.push_back(std::make_pair(curr_node->position, child->position));
        bfs_queue.push(child);
      }
    }
  }

  void RRT::init() {
    // set up kdtree
    kdtree_ = kd_create(3);
    use_node_num_ = 0;

    // pre-allocate memory for kdtree nodes
    path_node_pool_.resize(max_tree_node_num_);
    for (int i = 0; i < max_tree_node_num_; i++) {
      path_node_pool_[i] = new RRTPathNode();
    }

    // set up occupancy grid
    grid_map_->getRegion(origin_, map_size_);
    resolution_ = grid_map_->getResolution();

    std::cout << "origin: " << origin_.transpose() << std::endl;
    std::cout << "map_size: " << map_size_.transpose() << std::endl;
    std::cout << "resolution: " << resolution_ << std::endl;
  }

  void RRT::reset() {
    use_node_num_ = 0;
    kd_clear(kdtree_);

    for (int i = 0; i < max_tree_node_num_; i++) {
      RRTPathNodePtr node = path_node_pool_[i];
      node->children.clear();
      node->parent = NULL;
      node->g_cost = inf;
    }
  }

  Eigen::Vector3d RRT::getRandomNode() {
    std::random_device rd;
    std::mt19937_64 gen(rd());
    std::uniform_real_distribution<double> dis(0, 1);

    Eigen::Vector3d x_rand;
    x_rand[0] = dis(gen) * map_size_[0] + origin_[0];
    x_rand[1] = dis(gen) * map_size_[1] + origin_[1];
    x_rand[2] = dis(gen) * map_size_[2] + origin_[2];

    return x_rand;
  }

  Eigen::Vector3d RRT::Step(Eigen::Vector3d from, Eigen::Vector3d to, double step_length) {
    Eigen::Vector3d step_dir = to - from;
    if (step_dir.norm() <= step_length)
      return to;
    step_dir.normalize();
    Eigen::Vector3d x_new = from + step_length * step_dir;

    return x_new;
  }

  bool RRT::isCollisionFree(Eigen::Vector3d from, Eigen::Vector3d to, double map_resolution) {
    Eigen::Vector3d step_dir = to - from;
    step_dir.normalize();
    for (double t = 0; t < step_dir.norm(); t += map_resolution) {
      Eigen::Vector3d x_test = from + t * step_dir;
      if (grid_map_->getInflateOccupancy(x_test))
        return false;
    }
    return true;
  }

  void RRT::retrievePath(RRTPathNodePtr end_node, std::vector<Eigen::Vector3d>& path) {
    RRTPathNodePtr node = end_node;
    while (node->parent!= NULL) {
      path.push_back(node->position);
      node = node->parent;
    }
    path.push_back(node->position);
    std::reverse(path.begin(), path.end());
  }


  int RRT::search(Eigen::Vector3d start_pt, Eigen::Vector3d end_pt, std::vector<Eigen::Vector3d>& path) {
    // insert start_pt into kdtree
    ros::Time start_time = ros::Time::now();
    RRTPathNodePtr start_node = path_node_pool_[use_node_num_];
    start_node->position = start_pt;
    start_node->g_cost = 0;
    start_node->parent = NULL;
    use_node_num_++;
    kd_insert(kdtree_, start_node->position.data(), start_node);
    
    for (int i = 0; i < max_tree_node_num_; i++) {
      ros::Time current_time = ros::Time::now();
      if ((current_time - start_time).toSec() > max_allowed_time_) {
        std::cout << "reach max allowed time" << std::endl;
        return NO_PATH_FOUND;
      }
      Eigen::Vector3d x_rand = getRandomNode();
      kdres* nearest_tree_node = kd_nearest(kdtree_, x_rand.data());
      Eigen::Vector3d x_near;
      RRTPathNodePtr nearest_node = static_cast<RRTPathNode*>(kd_res_item(nearest_tree_node, x_near.data()));
      //deallocate nearest_tree_node
      kd_res_free(nearest_tree_node);
      nearest_node->position = x_near;
      Eigen::Vector3d x_new = Step(x_near, x_rand, step_length_);
      // check if x_near to x_new is collision free
      if (isCollisionFree(x_near, x_new, resolution_)) {
        RRTPathNodePtr x_new_node = path_node_pool_[use_node_num_];
        x_new_node->position = x_new;
        x_new_node->g_cost = nearest_node->g_cost + (x_new - x_near).norm();
        x_new_node->parent = nearest_node;
        nearest_node->children.push_back(x_new_node);
        kd_insert(kdtree_, x_new.data(), x_new_node);
        use_node_num_++;

        // check if reach end point
        if (abs(x_new[0] - end_pt[0]) < tolerance_ && abs(x_new[1] - end_pt[1]) < tolerance_ && abs(x_new[2] - end_pt[2]) < tolerance_) {
          std::cout << "reach end point" << std::endl;
          RRTPathNodePtr end_node = x_new_node;
          retrievePath(end_node, path);
          ros::Time end_time = ros::Time::now();
          double total_time = (end_time - start_time).toSec();
          double total_cost = end_node->g_cost;
          std::cout << "total cost: " << total_cost << std::endl;
          std::cout << "total time: " << total_time << std::endl;
          return REACH_END;
        }
        else continue;

      }
      else continue;
    }
    std::cout << "tree is full, no path found" << std::endl;
    return NO_PATH_FOUND;
  }

  RRT::~RRT() {
    // delete kdtree
    kd_free(kdtree_);

    // delete path node pool
    for (int i = 0; i < max_tree_node_num_; i++) {
      delete path_node_pool_[i];
    }
  }




} // namespace path_searching