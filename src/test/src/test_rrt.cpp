#include <path_searching/rrt.h>
#include <plan_env/grid_map.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "test_rrt_node");
  ros::NodeHandle nh;

  GridMap::Ptr grid_map = std::make_shared<GridMap>();
  grid_map->initMap(nh);

  path_searching::RRT::Ptr rrt_ ;
  rrt_ = std::make_shared<path_searching::RRT>();

  rrt_->setParam(nh);
  cout << "RRT parameters set" << endl;
  rrt_->setGridMap(grid_map);
  cout << "Grid map set" << endl;
  rrt_->init();
  cout << "RRT initialized" << endl;
  Eigen::Vector3d start_pt(0, 0, 0.1);
  Eigen::Vector3d end_pt(10, 10, 1);
  std::vector<Eigen::Vector3d> path;
  rrt_->search(start_pt, end_pt, path);

  for (int i = 0; i < path.size(); i++) {
    std::cout << "Point " << i << ": " << path[i].transpose() << std::endl;
  };

  ros::spin();
  return 0;
  
}