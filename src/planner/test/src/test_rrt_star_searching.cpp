#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <plan_env/grid_map.h>
#include <path_searching/rrt_star.h>

path_searching::RRTStar::Ptr rrt_star_;

ros::Subscriber goal_sub;
ros::Subscriber odom_sub;

// ros::Publisher path_pub;
// ros::Publisher rrt_tree_vertices_pub;
// ros::Publisher rrt_tree_edges_pub;

nav_msgs::Odometry::ConstPtr odom_;
std::vector<Eigen::Vector3d> path;
// std::vector<Eigen::Vector3d> vertices;
// std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> edges;

void OdomCallback(const nav_msgs::Odometry::ConstPtr& odom)
{
  odom_ = odom;
}

void GoalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  Eigen::Vector3d end_pt(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
  Eigen::Vector3d start_pt(odom_->pose.pose.position.x, odom_->pose.pose.position.y, odom_->pose.pose.position.z);
  std::cout << "Start point: " << start_pt.transpose() << std::endl;
  std::cout << "End point: " << end_pt.transpose() << std::endl;
  int success = rrt_star_->search(start_pt, end_pt, path);

  if (success == 1)
  {
    std::cout << "Path found!" << std::endl;
    // rrt_star_->getWholeTree(vertices, edges);
    // rrt_star_->visWholeTree(vertices, edges);
  }
  else
  {
    std::cout << "Path not found!" << std::endl;
  }
  path.clear();
  // vertices.clear(); 
  // edges.clear();
  rrt_star_->reset();
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "test_rrt_star_searching");
  ros::NodeHandle nh("~");

  ros::Subscriber goal_sub = nh.subscribe<geometry_msgs::PoseStamped>("/goal", 10, &GoalCallback);
  ros::Subscriber odom_sub = nh.subscribe<nav_msgs::Odometry>("/visual_slam/odom", 10, &OdomCallback);

  // path_pub = nh.advertise<visualization_msgs::Marker>("path", 10);
  // rrt_tree_vertices_pub = nh.advertise<visualization_msgs::Marker>("rrt_star_tree_vertices", 10);
  // rrt_tree_edges_pub = nh.advertise<visualization_msgs::Marker>("rrt_star_tree_edges", 10);

  GridMap::Ptr grid_map = std::make_shared<GridMap>();
  grid_map->initMap(nh);
  
  rrt_star_ = std::make_shared<path_searching::RRTStar>();

  rrt_star_->setParam(nh);
  rrt_star_->setGridMap(grid_map);
  rrt_star_->init();

  ros::spin();
  return 0;
}