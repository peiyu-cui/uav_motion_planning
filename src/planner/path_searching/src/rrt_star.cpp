#include <path_searching/rrt_star.h>

namespace path_searching
{

void RRTStar::setParam(ros::NodeHandle& nh)
{
  nh.param("rrt_star/max_tree_node_num", max_tree_node_num_, 100000);
  nh.param("rrt_star/step_length", step_length_, 0.5);
  nh.param("rrt_star/search_radius", search_radius_, 0.5);
  nh.param("rrt_star/collision_check_resolution", resolution_, 0.05);

  vis_path_pub_ = nh.advertise<visualization_msgs::Marker>("rrt_star/path", 1);
  vis_path_marker_.header.frame_id = "world";
  vis_path_marker_.header.stamp = ros::Time::now();
  vis_path_marker_.ns = "rrt_star";
  vis_path_marker_.type = visualization_msgs::Marker::LINE_STRIP;
  vis_path_marker_.action = visualization_msgs::Marker::ADD;
  vis_path_marker_.pose.orientation.w = 1.0;
  vis_path_marker_.scale.x = 0.2;
  vis_path_marker_.scale.y = 0.2;
  vis_path_marker_.scale.z = 0.2;
  vis_path_marker_.color.a = 1.0;
  vis_path_marker_.color.r = 1.0;
  vis_path_marker_.color.g = 0.0;
  vis_path_marker_.color.b = 0.0;

  vis_tree_pub_ = nh.advertise<visualization_msgs::Marker>("rrt_star/tree", 10);
  vis_tree_marker_.header.frame_id = "world";
  vis_tree_marker_.header.stamp = ros::Time::now();
  vis_tree_marker_.ns = "rrt_star";
  vis_tree_marker_.type = visualization_msgs::Marker::LINE_LIST;
  vis_tree_marker_.action = visualization_msgs::Marker::ADD;
  vis_tree_marker_.pose.orientation.w = 1.0;
  vis_tree_marker_.scale.x = 0.05;
  vis_tree_marker_.scale.y = 0.05;
  vis_tree_marker_.scale.z = 0.05;
  vis_tree_marker_.color.a = 1.0;
  vis_tree_marker_.color.r = 0.0;
  vis_tree_marker_.color.g = 1.0;
  vis_tree_marker_.color.b = 0.0;

}


void RRTStar::setGridMap(GridMap::Ptr& grid_map)
{
  this->grid_map_ = grid_map;
}

void RRTStar::init()
{
  // create kdtree
  kdtree_ = kd_create(3);

  path_node_pool_.resize(max_tree_node_num_);
  for (int i = 0; i < max_tree_node_num_; i++)
  {
    path_node_pool_[i] = new RRTStarNode();
  }

  use_node_num_ = 0;
  reach_goal_ = false;
  goal_node_ = NULL;

  grid_map_->getRegion(origin_, map_size_);

  std::cout << "origin: " << origin_.transpose() << std::endl;
  std::cout << "map_size: " << map_size_.transpose() << std::endl;
  std::cout << "resolution: " << resolution_ << std::endl;
}


void RRTStar::reset()
{
  kd_clear(kdtree_);
  use_node_num_ = 0;
  goal_node_ = NULL;
  reach_goal_ = false;

  for (int i = 0; i < max_tree_node_num_; i++)
  {
    RRTStarNodePtr tmp_node = path_node_pool_[i];
    tmp_node->parent = NULL;
    tmp_node->children.clear();
    tmp_node->g_cost = inf;
  }

}

Eigen::Vector3d RRTStar::getRandomNode()
{
  Eigen::Vector3d x_rand;
  std::random_device rd;
  std::mt19937_64 gen(rd());
  std::uniform_real_distribution<> dis(0.0, 1.0);

  x_rand[0] = dis(gen) * map_size_[0] + origin_[0];
  x_rand[1] = dis(gen) * map_size_[1] + origin_[1];
  x_rand[2] = dis(gen) * map_size_[2] + origin_[2];

  return x_rand;
}

Eigen::Vector3d RRTStar::Step(Eigen::Vector3d from, Eigen::Vector3d to, double step_length)
{
  Eigen::Vector3d step_dir = (to - from).normalized();
  Eigen::Vector3d step_end = from + step_dir * step_length;
  return step_end;
}

bool RRTStar::isCollisionFree(Eigen::Vector3d from, Eigen::Vector3d to, double map_resolution) {
  Eigen::Vector3d step_dir = to - from;
  step_dir.normalize();
  for (double t = 0; t < step_dir.norm(); t += map_resolution) {
    Eigen::Vector3d x_test = from + t * step_dir;
    if (grid_map_->getInflateOccupancy(x_test))
      return false;
  }
  return true;
}

bool RRTStar::ChooseParent(kdres* &x_new_neighbors, RRTStarNodePtr& new_node)
{
  double compare_cost = inf;
  while (!kd_res_end(x_new_neighbors))
   {
    RRTStarNodePtr x_new_neighbor = static_cast<RRTStarNodePtr>(kd_res_item_data(x_new_neighbors));
    double g_cost_new = x_new_neighbor->g_cost + (x_new_neighbor->position - new_node->position).norm();
    if (g_cost_new < compare_cost && isCollisionFree(x_new_neighbor->position, new_node->position, resolution_))
    {
      compare_cost = g_cost_new;
      new_node->parent = x_new_neighbor;
      // x_new_neighbor->children.push_back(new_node); // fatal error here when traversing the search tree, has been fixed
      new_node->g_cost = g_cost_new;
    }
    kd_res_next(x_new_neighbors);
   }

  if (compare_cost == inf)
  {
    kd_res_free(x_new_neighbors);
    return false;
  } 
  else
  {
    new_node->parent->children.push_back(new_node);
    kd_res_free(x_new_neighbors);
    return true;
  }

}

void RRTStar::ReWireTree(RRTStarNodePtr& new_node)
{
  kdres* x_new_neighbors = kd_nearest_range(kdtree_, new_node->position.data(), search_radius_);
  while (!kd_res_end(x_new_neighbors))
  {
    RRTStarNodePtr x_new_neighbor = static_cast<RRTStarNodePtr>(kd_res_item_data(x_new_neighbors));
    double g_cost_new = new_node->g_cost + (x_new_neighbor->position - new_node->position).norm();
    if (g_cost_new < x_new_neighbor->g_cost && 
        isCollisionFree(new_node->position, x_new_neighbor->position, resolution_))
    {
      std::vector<RRTStarNodePtr> p_children = x_new_neighbor->parent->children;
      auto it = std::remove(p_children.begin(), p_children.end(), x_new_neighbor);
      p_children.erase(it, p_children.end());
      x_new_neighbor->parent = new_node;
      x_new_neighbor->g_cost = g_cost_new;
      new_node->children.push_back(x_new_neighbor);
    }
    kd_res_next(x_new_neighbors);
  }
  kd_res_free(x_new_neighbors);
}

void RRTStar::retrievePath(RRTStarNodePtr end_node, std::vector<Eigen::Vector3d>& path)
{
  RRTStarNodePtr tmp_node = end_node;
  while (tmp_node->parent!= NULL)
  {
    path.push_back(tmp_node->position);
    tmp_node = tmp_node->parent;
  }
  path.push_back(tmp_node->position);
  std::reverse(path.begin(), path.end());
}

void RRTStar::visFeasiblePath(std::vector<Eigen::Vector3d> path)
{
  vis_path_marker_.points.clear();
  for (int i = 0; i < path.size(); i++)
  {
    geometry_msgs::Point pt;
    pt.x = path[i][0];
    pt.y = path[i][1];
    pt.z = path[i][2];
    vis_path_marker_.points.push_back(pt);
  }
  vis_path_pub_.publish(vis_path_marker_);
}

void RRTStar::getWholeTree(std::vector<Eigen::Vector3d> &vertices, 
                        std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> &edges) {
  RRTStarNodePtr start_node = path_node_pool_[0];
  vertices.push_back(start_node->position);
  // bfs to retrieve whole tree， use queue data structure
  std::queue<RRTStarNodePtr> bfs_queue;
  bfs_queue.push(start_node);
  while (!bfs_queue.empty()) {
    RRTStarNodePtr curr_node = bfs_queue.front();
    bfs_queue.pop();
    if (curr_node->children.empty()) continue;
    for (RRTStarNodePtr child : curr_node->children) {
      // check if my child's child is me
      // assert(std::find(child->children.begin(), child->children.end(), curr_node) == child->children.end());
      vertices.push_back(child->position);
      edges.push_back(std::make_pair(curr_node->position, child->position));
      bfs_queue.push(child);
    }
  }
}

void RRTStar::visWholeTree(std::vector<Eigen::Vector3d> vertices, 
                          std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> edges) {
  vis_tree_marker_.points.clear();
  for (int i = 0; i < edges.size(); i++) {
    geometry_msgs::Point pt1, pt2;
    pt1.x = edges[i].first[0];
    pt1.y = edges[i].first[1];
    pt1.z = edges[i].first[2];
    pt2.x = edges[i].second[0];
    pt2.y = edges[i].second[1];
    pt2.z = edges[i].second[2];
    vis_tree_marker_.points.push_back(pt1);
    vis_tree_marker_.points.push_back(pt2);
  }
  // for (int i = 0; i < vertices.size(); i++) {
  //   geometry_msgs::Point pt;
  //   pt.x = vertices[i][0];
  //   pt.y = vertices[i][1];
  //   pt.z = vertices[i][2];
  //   vis_vertice_marker_.points.push_back(pt);
  // }
  vis_tree_pub_.publish(vis_tree_marker_);
}

RRTStarNodePtr RRTStar::getMinCostNode(kdres* &x_end_neighbors)
{
  double min_cost = inf;
  RRTStarNodePtr min_cost_node;
  while (!kd_res_end(x_end_neighbors))
  {
    RRTStarNodePtr tmp_node = static_cast<RRTStarNodePtr>(kd_res_item_data(x_end_neighbors));
    if (tmp_node->g_cost + (tmp_node->position - goal_node_->position).norm() < min_cost && isCollisionFree(tmp_node->position, goal_node_->position, resolution_))
    {
      min_cost = tmp_node->g_cost + (tmp_node->position - goal_node_->position).norm();
      min_cost_node = tmp_node;
    }
    kd_res_next(x_end_neighbors);
  }
  kd_res_free(x_end_neighbors);
  return min_cost_node;
}

int RRTStar::search(Eigen::Vector3d start, Eigen::Vector3d end, std::vector<Eigen::Vector3d>& path)
{
  ros::Time start_time = ros::Time::now();
  RRTStarNodePtr start_node = path_node_pool_[use_node_num_];
  start_node->position = start;
  start_node->g_cost = 0.0;
  start_node->parent = NULL;
  use_node_num_ = use_node_num_ + 1;

  RRTStarNodePtr global_goal_node = path_node_pool_[use_node_num_];
  global_goal_node->position = end;
  use_node_num_++;

  std::vector<Eigen::Vector3d> vertices;
  std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> edges;

  // add start node to kdtree
  kd_insert(kdtree_, start_node->position.data(), start_node);

  double feasible_cost_ = inf;
  double tmp_cost = inf;
  // sample random nodes
  for (int i = 0; i < max_tree_node_num_; i++)
  {
    Eigen::Vector3d x_rand = getRandomNode();
    kdres* nearest_tree_node = kd_nearest(kdtree_, x_rand.data());
    RRTStarNodePtr nearest_node = static_cast<RRTStarNodePtr>(kd_res_item_data(nearest_tree_node));
    kd_res_free(nearest_tree_node);

    Eigen::Vector3d x_near = nearest_node->position;
    Eigen::Vector3d x_new = Step(x_near, x_rand, step_length_);

    if(grid_map_->getInflateOccupancy(x_new) != true)
    {
      kdres* x_new_neighbors = kd_nearest_range(kdtree_, x_new.data(), search_radius_);
      RRTStarNodePtr new_node = path_node_pool_[use_node_num_];
      new_node->position = x_new;
      use_node_num_++;
      if (ChooseParent(x_new_neighbors, new_node))
      {
        kd_insert(kdtree_, new_node->position.data(), new_node);
        // rewire the local rrt_star tree
        ReWireTree(new_node);
 
        if((new_node->position-end).norm() <= search_radius_ )
        {
          if(reach_goal_ == false)
          {
            if(isCollisionFree(new_node->position, end, resolution_))
            {
              ros::Time first_end_time = ros::Time::now();
              reach_goal_ = true;
              std::cout << "first reach goal!" << std::endl;
              std::cout << "first reach goal time: " << (first_end_time - start_time).toSec() << std::endl;

              new_node->children.push_back(global_goal_node);
              global_goal_node->parent = new_node;
              global_goal_node->g_cost = new_node->g_cost + (global_goal_node->position - new_node->position).norm();
              goal_node_ = global_goal_node;
              feasible_cost_ = global_goal_node->g_cost;
              retrievePath(global_goal_node, path);
              visFeasiblePath(path);
              path.clear();
              getWholeTree(vertices, edges);
              visWholeTree(vertices, edges);
              vertices.clear();
              edges.clear();
            }
            else continue;

          }

          // only when x_new is close to the goal, check if it has a better path to the goal
          // bool has_better_path = new_node->g_cost + (new_node->position - end).norm() < feasible_cost_;
          // if(isCollisionFree(new_node->position, end, resolution_) && has_better_path) 
          // {
          //   feasible_cost_ = new_node->g_cost + (new_node->position - end).norm();
          //   std::cout << "path cost: " << feasible_cost_ << std::endl;
          //   RRTStarNodePtr end_node = global_goal_node;
          //   end_node->g_cost = new_node->g_cost + (end_node->position - new_node->position).norm();
          //   end_node->parent = new_node;
          //   bool not_exist_child = std::find(new_node->children.begin(), new_node->children.end(), end_node) != new_node->children.end();
          //   if (not_exist_child)
          //   {
          //     new_node->children.push_back(end_node);
          //   }
            
          //   retrievePath(end_node, path);
          //   visFeasiblePath(path);
          //   // getWholeTree(vertices, edges);
          //   // visWholeTree(vertices, edges);
          //   path.clear();
          //   // vertices.clear();
          //   // edges.clear();
          // }

        }

        if (reach_goal_)
        {

          kdres* x_end_neighbors = kd_nearest_range(kdtree_, global_goal_node->position.data(), search_radius_);
          RRTStarNodePtr min_end_neibor = getMinCostNode(x_end_neighbors);
          tmp_cost = min_end_neibor->g_cost + (min_end_neibor->position - global_goal_node->position).norm();

          if (tmp_cost < feasible_cost_)
          {
            global_goal_node->parent = min_end_neibor;
            min_end_neibor->children.push_back(global_goal_node);
            global_goal_node->g_cost = tmp_cost;

            std::cout << "path cost: " << tmp_cost << std::endl;
            feasible_cost_ = tmp_cost;

            retrievePath(goal_node_, path);
            visFeasiblePath(path);
            path.clear();

          }

          // std::cout << "find a feasible path!" << std::endl;
        }

      }
      else continue;
    }
    else continue;
    
  }

  if (reach_goal_)
  {
    std::cout << "find a asymptotic Optimal path!" << std::endl;
    return REACH_END;
  }
  else
  {
    std::cout << "reach max tree node num, no path found!" << std::endl;
    return NO_PATH_FOUND;
  }

}
  

RRTStar::~RRTStar()
{
  // free memory of kdtree and path_node_pool_
  kd_free(kdtree_);

  for (int i = 0; i < max_tree_node_num_; i++)
  {
    delete path_node_pool_[i];
  }
}


} // namespace path_searching