#include <path_searching/rrt_star.h>
#include <plan_env/grid_map.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "test_rrt_star_node");
  ros::NodeHandle nh;

  GridMap::Ptr grid_map = std::make_shared<GridMap>();
  grid_map->initMap(nh);

  path_searching::RRTStar::Ptr rrt_star_;
  rrt_star_ = std::make_shared<path_searching::RRTStar>();

  rrt_star_->setParam(nh);
  cout << "RRTStar parameters set" << endl;
  rrt_star_->setGridMap(grid_map);
  cout << "Grid map set" << endl;
  rrt_star_->init();
  cout << "RRTStar initialized" << endl;
  Eigen::Vector3d start_pt(0, 0, 0.1);
  Eigen::Vector3d end_pt(10, 10, 1);
  std::vector<Eigen::Vector3d> path;
  rrt_star_->search(start_pt, end_pt, path);

  for (int i = 0; i < path.size(); i++) {
    std::cout << "Point " << i << ": " << path[i].transpose() << std::endl;
  };

  ros::spin();
  return 0;
  
}