#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <Eigen/Dense>
#include <fstream>
#include <iostream>
#include <pcl/search/impl/kdtree.hpp>
#include <vector>

using namespace std;
using namespace Eigen;

ros::Publisher cloud_pub_;

sensor_msgs::PointCloud2 local_map_pcl, local_depth_pcl;

ros::Subscriber odom_sub_, global_map_sub_, local_map_sub_;

ros::Timer local_sensing_timer_;

bool has_global_map(false);
bool has_local_map(false);
bool has_odom_(false);

nav_msgs::Odometry odom_;

double sensing_horizon_, sensing_rate_, estimation_rate_;
double x_size_, y_size_, z_size_;
double x_l_, y_l_, z_l_;
double resolution_, inv_resolution_;
int GLX_SIZE_, GLY_SIZE_, GLZ_SIZE_;

ros::Time last_odom_stamp = ros::TIME_MAX;

pcl::PointCloud<pcl::PointXYZ> cloud_all_map_, local_map_;
pcl::VoxelGrid<pcl::PointXYZ> voxel_sampler_;
sensor_msgs::PointCloud2 local_map_pcd_;

pcl::search::KdTree<pcl::PointXYZ> kdtreeLocalMap_;
vector<int> pointIdxRadiusSearch_;
vector<float> pointRadiusSquaredDistance_;

inline Eigen::Vector3d gridIndex2coord(const Eigen::Vector3i &index)
{
  Eigen::Vector3d pt;
  pt(0) = ((double)index(0) + 0.5) * resolution_ + x_l_;
  pt(1) = ((double)index(1) + 0.5) * resolution_ + y_l_;
  pt(2) = ((double)index(2) + 0.5) * resolution_ + z_l_;
  return pt;
};

inline Eigen::Vector3i coord2gridIndex(const Eigen::Vector3d &pt)
{
  Eigen::Vector3i idx;
  idx(0) = std::min(std::max(int((pt(0) - x_l_) * inv_resolution_), 0), GLX_SIZE_ - 1);
  idx(1) = std::min(std::max(int((pt(1) - y_l_) * inv_resolution_), 0), GLY_SIZE_ - 1);
  idx(2) = std::min(std::max(int((pt(2) - z_l_) * inv_resolution_), 0), GLZ_SIZE_ - 1);
  return idx;
};

void rcvOdometryCallbck(const nav_msgs::Odometry &odom)
{
  has_odom_ = true;
  odom_ = odom;
}

void rcvGlobalPointCloudCallBack(const sensor_msgs::PointCloud2 &pointcloud_map)
{
  if (has_global_map)
    return;

  ROS_WARN("Global Pointcloud received..");

  pcl::PointCloud<pcl::PointXYZ> cloud_input;
  pcl::fromROSMsg(pointcloud_map, cloud_input);

  voxel_sampler_.setLeafSize(0.1f, 0.1f, 0.1f);
  voxel_sampler_.setInputCloud(cloud_input.makeShared());
  voxel_sampler_.filter(cloud_all_map_);

  kdtreeLocalMap_.setInputCloud(cloud_all_map_.makeShared());

  has_global_map = true;
}

void rcvLocalPointCloudCallBack(const sensor_msgs::PointCloud2 &pointcloud_map)
{
  // do nothing, fix later
}

void renderSensedPoints(const ros::TimerEvent &event)
{
  if (!has_global_map || !has_odom_)
    return;

  Eigen::Quaterniond q;
  q.x() = odom_.pose.pose.orientation.x;
  q.y() = odom_.pose.pose.orientation.y;
  q.z() = odom_.pose.pose.orientation.z;
  q.w() = odom_.pose.pose.orientation.w;

  Eigen::Matrix3d rot;
  rot = q;
  Eigen::Vector3d yaw_vec = rot.col(0);

  local_map_.points.clear();
  pcl::PointXYZ searchPoint(odom_.pose.pose.position.x, odom_.pose.pose.position.y, odom_.pose.pose.position.z);
  pointIdxRadiusSearch_.clear();
  pointRadiusSquaredDistance_.clear();

  pcl::PointXYZ pt;
  if (kdtreeLocalMap_.radiusSearch(searchPoint, sensing_horizon_, pointIdxRadiusSearch_, pointRadiusSquaredDistance_) > 0)
  {
    for (size_t i = 0; i < pointIdxRadiusSearch_.size(); i++)
    {
      pt = cloud_all_map_.points[pointIdxRadiusSearch_[i]];

      // if ((fabs(pt.z - odom_.pose.pose.position.z) / (pt.x - odom_.pose.pose.position.x)) >
      //     tan(M_PI / 12.0))
      //   continue;
      if ((fabs(pt.z - odom_.pose.pose.position.z) / sensing_horizon_) > tan(M_PI / 6.0))
        continue;

      Vector3d pt_vec(pt.x - odom_.pose.pose.position.x, pt.y - odom_.pose.pose.position.y, pt.z - odom_.pose.pose.position.z);

      if (pt_vec.normalized().dot(yaw_vec) < 0.5)
        continue;

      local_map_.points.push_back(pt);
    }
  }
  else
  {
    return;
  }

  local_map_.width = local_map_.points.size();
  local_map_.height = 1;
  local_map_.is_dense = true;

  pcl::toROSMsg(local_map_, local_map_pcd_);
  local_map_pcd_.header.frame_id = "world";

  cloud_pub_.publish(local_map_pcd_);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "pcl_render_node");
  ros::NodeHandle nh("~");

  nh.getParam("sensing_horizon", sensing_horizon_);
  nh.getParam("sensing_rate", sensing_rate_);
  nh.getParam("estimation_rate", estimation_rate_);

  nh.getParam("map/x_size", x_size_);
  nh.getParam("map/y_size", y_size_);
  nh.getParam("map/z_size", z_size_);

  // subscribe point cloud
  odom_sub_ = nh.subscribe("odom", 50, rcvOdometryCallbck);
  global_map_sub_ = nh.subscribe("global_map", 1, rcvGlobalPointCloudCallBack);
  local_map_sub_ = nh.subscribe("local_map", 1, rcvLocalPointCloudCallBack);

  // publisher depth image and color image
  cloud_pub_ = nh.advertise<sensor_msgs::PointCloud2>("cloud", 10);

  double sensing_duration = 1.0 / sensing_rate_ * 2.5;

  local_sensing_timer_ = nh.createTimer(ros::Duration(sensing_duration), renderSensedPoints);

  inv_resolution_ = 1.0 / resolution_;

  x_l_ = -x_size_ / 2.0;
  y_l_ = -y_size_ / 2.0;
  z_l_ = 0.0;

  GLX_SIZE_ = (int)(x_size_ * inv_resolution_);
  GLY_SIZE_ = (int)(y_size_ * inv_resolution_);
  GLZ_SIZE_ = (int)(z_size_ * inv_resolution_);

  ros::Rate rate(100);
  bool status = ros::ok();
  while (status)
  {
    ros::spinOnce();
    status = ros::ok();
    rate.sleep();
  }

  return 0;
}
