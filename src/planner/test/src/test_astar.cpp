#include <path_searching/a_star.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "test_astar_node");
  ros::NodeHandle nh;

  path_searching::Astar::Ptr astar_;
  astar_ = std::make_shared<path_searching::Astar>();

  astar_->setParam(nh);
  cout << "Astar parameters set" << endl;
  astar_->init();
  cout << "Astar initialized" << endl;
  Eigen::Vector3d start_pt(0, 0, 0);
  Eigen::Vector3d end_pt(10, 10, 1);
  std::vector<Eigen::Vector3d> path;
  astar_->search(start_pt, end_pt, path);

  for (int i = 0; i < path.size(); i++) {
    std::cout << "Point " << i << ": " << path[i].transpose() << std::endl;
  };

  ros::spin();
  return 0;
  
}