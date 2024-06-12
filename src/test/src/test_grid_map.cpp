#include <ros/ros.h>
#include <plan_env/grid_map.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "test_grid_map");
  ros::NodeHandle nh("~");

  GridMap::Ptr grid_map = std::make_shared<GridMap>();
  grid_map->initMap(nh);

  ros::spin();
  return 0;
}