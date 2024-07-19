#include <path_searching/kino_astar.h>
#include <plan_env/grid_map.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "test_kino_astar_node");
  ros::NodeHandle nh;

  GridMap::Ptr grid_map = std::make_shared<GridMap>();
  grid_map->initMap(nh);

  path_searching::KinoAstar::Ptr kino_astar_ ;
  kino_astar_ = std::make_shared<path_searching::KinoAstar>();

  kino_astar_->setParam(nh);
  cout << "RRT parameters set" << endl;
  kino_astar_->setGridMap(grid_map);
  cout << "Grid map set" << endl;
  kino_astar_->init();
  cout << "RRT initialized" << endl;
  Eigen::Vector3d start_pt(0, 0, 1.0);
  Eigen::Vector3d start_vel(1, 1, 0);
  Eigen::Vector3d end_pt(5, 5, 0.8);
  Eigen::Vector3d end_vel(0, 0, 0);
  std::vector<Eigen::Vector3d> path;
  kino_astar_->search(start_pt, start_vel, end_pt, end_vel, path);

  for (int i = 0; i < path.size(); i++) {
    std::cout << "Point " << i << ": " << path[i].transpose() << std::endl;
  };

  ros::spin();
  return 0;
  
}