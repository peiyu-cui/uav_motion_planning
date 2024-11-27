#include <iostream>
#include <fstream>
#include <vector>
// include ros dep.
#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <image_transport/image_transport.h>
#include <dynamic_reconfigure/server.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Bool.h>

#include "tf/tf.h"
#include "tf/transform_datatypes.h"
#include <tf/transform_broadcaster.h>
// include pcl dep
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
// include opencv and eigen
#include <Eigen/Eigen>
#include "opencv2/highgui/highgui.hpp"
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>
#include <cv_bridge/cv_bridge.h>

// #include <cloud_banchmark/cloud_banchmarkConfig.h>
#include "depth_render.cuh"
#include "quadrotor_msgs/PositionCommand.h"
using namespace cv;
using namespace std;
using namespace Eigen;

int *depth_hostptr;
cv::Mat depth_mat;

// camera param
int width, height;
double fx, fy, cx, cy;

DepthRender depthrender;
ros::Publisher pub_depth;
ros::Publisher pub_color;
ros::Publisher pub_pose;
ros::Publisher pub_pcl_wolrd;

sensor_msgs::PointCloud2 local_map_pcl;
sensor_msgs::PointCloud2 local_depth_pcl;

ros::Subscriber odom_sub_;
ros::Subscriber global_map_sub_, local_map_sub_;

ros::Timer local_sensing_timer_, estimation_timer;

bool has_global_map(false);
bool has_local_map(false);
bool has_odom_(false);

Matrix4d cam02body;
Matrix4d cam2world;
Eigen::Quaterniond cam2world_quat;
nav_msgs::Odometry odom_;

double sensing_horizon_, sensing_rate_, estimation_rate_;
double x_size_, y_size_, z_size_;
double x_l_, y_l_, z_l_;
double resolution_, inv_resolution_;
int GLX_SIZE_, GLY_SIZE_, GLZ_SIZE_;

ros::Time last_odom_stamp = ros::TIME_MAX;
Eigen::Vector3d last_pose_world;

void render_currentpose();
void render_pcl_world();

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
  /*if(!has_global_map)
    return;*/
  has_odom_ = true;
  odom_ = odom;
  Matrix4d Pose_receive = Matrix4d::Identity();

  Eigen::Vector3d request_position;
  Eigen::Quaterniond request_pose;
  request_position.x() = odom.pose.pose.position.x;
  request_position.y() = odom.pose.pose.position.y;
  request_position.z() = odom.pose.pose.position.z;
  request_pose.x() = odom.pose.pose.orientation.x;
  request_pose.y() = odom.pose.pose.orientation.y;
  request_pose.z() = odom.pose.pose.orientation.z;
  request_pose.w() = odom.pose.pose.orientation.w;
  Pose_receive.block<3, 3>(0, 0) = request_pose.toRotationMatrix();
  Pose_receive(0, 3) = request_position(0);
  Pose_receive(1, 3) = request_position(1);
  Pose_receive(2, 3) = request_position(2);

  Matrix4d body_pose = Pose_receive;
  // convert to cam pose
  cam2world = body_pose * cam02body;
  cam2world_quat = cam2world.block<3, 3>(0, 0);

  last_odom_stamp = odom.header.stamp;

  last_pose_world(0) = odom.pose.pose.position.x;
  last_pose_world(1) = odom.pose.pose.position.y;
  last_pose_world(2) = odom.pose.pose.position.z;

  // publish tf
  /*static tf::TransformBroadcaster br;
  tf::Transform transform;
  transform.setOrigin( tf::Vector3(cam2world(0,3), cam2world(1,3), cam2world(2,3) ));
  transform.setRotation(tf::Quaternion(cam2world_quat.x(), cam2world_quat.y(), cam2world_quat.z(), cam2world_quat.w()));
  br.sendTransform(tf::StampedTransform(transform, last_odom_stamp, "world", "camera")); //publish transform from world frame to quadrotor frame.*/
}

void pubCameraPose(const ros::TimerEvent &event)
{
  // cout<<"pub cam pose"
  geometry_msgs::PoseStamped camera_pose;
  camera_pose.header = odom_.header;
  camera_pose.header.frame_id = "/map";
  camera_pose.pose.position.x = cam2world(0, 3);
  camera_pose.pose.position.y = cam2world(1, 3);
  camera_pose.pose.position.z = cam2world(2, 3);
  camera_pose.pose.orientation.w = cam2world_quat.w();
  camera_pose.pose.orientation.x = cam2world_quat.x();
  camera_pose.pose.orientation.y = cam2world_quat.y();
  camera_pose.pose.orientation.z = cam2world_quat.z();
  pub_pose.publish(camera_pose);
}

void renderSensedPoints(const ros::TimerEvent &event)
{
  // if(! has_global_map || ! has_odom_) return;
  if (!has_global_map && !has_local_map)
    return;

  if (!has_odom_)
    return;
  render_currentpose();
  render_pcl_world();
}

vector<float> cloud_data;
void rcvGlobalPointCloudCallBack(const sensor_msgs::PointCloud2 &pointcloud_map)
{
  if (has_global_map)
    return;

  ROS_WARN("Global Pointcloud received..");
  // load global map
  pcl::PointCloud<pcl::PointXYZ> cloudIn;
  pcl::PointXYZ pt_in;
  // transform map to point cloud format
  pcl::fromROSMsg(pointcloud_map, cloudIn);
  for (int i = 0; i < int(cloudIn.points.size()); i++)
  {
    pt_in = cloudIn.points[i];
    cloud_data.push_back(pt_in.x);
    cloud_data.push_back(pt_in.y);
    cloud_data.push_back(pt_in.z);
  }
  printf("global map has points: %d.\n", (int)cloud_data.size() / 3);
  // pass cloud_data to depth render
  depthrender.set_data(cloud_data);
  depth_hostptr = (int *)malloc(width * height * sizeof(int));

  has_global_map = true;
}

void rcvLocalPointCloudCallBack(const sensor_msgs::PointCloud2 &pointcloud_map)
{
  // ROS_WARN("Local Pointcloud received..");
  // load local map
  pcl::PointCloud<pcl::PointXYZ> cloudIn;
  pcl::PointXYZ pt_in;
  // transform map to point cloud format
  pcl::fromROSMsg(pointcloud_map, cloudIn);

  if (cloudIn.points.size() == 0)
    return;
  for (int i = 0; i < int(cloudIn.points.size()); i++)
  {
    pt_in = cloudIn.points[i];
    Eigen::Vector3d pose_pt(pt_in.x, pt_in.y, pt_in.z);
    // pose_pt = gridIndex2coord(coord2gridIndex(pose_pt));
    cloud_data.push_back(pose_pt(0));
    cloud_data.push_back(pose_pt(1));
    cloud_data.push_back(pose_pt(2));
  }
  // printf("local map has points: %d.\n", (int)cloud_data.size() / 3 );
  // pass cloud_data to depth render
  depthrender.set_data(cloud_data);
  depth_hostptr = (int *)malloc(width * height * sizeof(int));

  has_local_map = true;
}

void render_pcl_world()
{
  // for debug purpose
  pcl::PointCloud<pcl::PointXYZ> localMap;
  pcl::PointXYZ pt_in;

  Eigen::Vector4d pose_in_camera;
  Eigen::Vector4d pose_in_world;
  Eigen::Vector3d pose_pt;

  for (int u = 0; u < width; u++)
    for (int v = 0; v < height; v++)
    {
      float depth = depth_mat.at<float>(v, u);

      if (depth == 0.0)
        continue;

      pose_in_camera(0) = (u - cx) * depth / fx;
      pose_in_camera(1) = (v - cy) * depth / fy;
      pose_in_camera(2) = depth;
      pose_in_camera(3) = 1.0;

      pose_in_world = cam2world * pose_in_camera;

      if ((pose_in_world.segment(0, 3) - last_pose_world).norm() > sensing_horizon_)
        continue;

      pose_pt = pose_in_world.head(3);
      // pose_pt = gridIndex2coord(coord2gridIndex(pose_pt));
      pt_in.x = pose_pt(0);
      pt_in.y = pose_pt(1);
      pt_in.z = pose_pt(2);

      localMap.points.push_back(pt_in);
    }

  localMap.width = localMap.points.size();
  localMap.height = 1;
  localMap.is_dense = true;

  pcl::toROSMsg(localMap, local_map_pcl);
  local_map_pcl.header.frame_id = "/map";
  local_map_pcl.header.stamp = last_odom_stamp;

  pub_pcl_wolrd.publish(local_map_pcl);
}

void render_currentpose()
{
  double this_time = ros::Time::now().toSec();

  Matrix4d cam_pose = cam2world.inverse();

  double pose[4 * 4];

  for (int i = 0; i < 4; i++)
    for (int j = 0; j < 4; j++)
      pose[j + 4 * i] = cam_pose(i, j);

  depthrender.render_pose(pose, depth_hostptr);
  // depthrender.render_pose(cam_pose, depth_hostptr);

  depth_mat = cv::Mat::zeros(height, width, CV_32FC1);
  double min = 0.5;
  double max = 1.0f;
  for (int i = 0; i < height; i++)
    for (int j = 0; j < width; j++)
    {
      float depth = (float)depth_hostptr[i * width + j] / 1000.0f;
      depth = depth < 500.0f ? depth : 0;
      max = depth > max ? depth : max;
      depth_mat.at<float>(i, j) = depth;
    }
  // ROS_INFO("render cost %lf ms.", (ros::Time::now().toSec() - this_time) * 1000.0f);
  // printf("max_depth %lf.\n", max);

  cv_bridge::CvImage out_msg;
  out_msg.header.stamp = last_odom_stamp;
  out_msg.header.frame_id = "camera";
  out_msg.encoding = sensor_msgs::image_encodings::TYPE_32FC1;
  out_msg.image = depth_mat.clone();
  pub_depth.publish(out_msg.toImageMsg());

  cv::Mat adjMap;
  // depth_mat.convertTo(adjMap,CV_8UC1, 255 / (max-min), -min);
  depth_mat.convertTo(adjMap, CV_8UC1, 255 / 13.0, -min);
  cv::Mat falseColorsMap;
  cv::applyColorMap(adjMap, falseColorsMap, cv::COLORMAP_RAINBOW);
  cv_bridge::CvImage cv_image_colored;
  cv_image_colored.header.frame_id = "depthmap";
  cv_image_colored.header.stamp = last_odom_stamp;
  cv_image_colored.encoding = sensor_msgs::image_encodings::BGR8;
  cv_image_colored.image = falseColorsMap;
  pub_color.publish(cv_image_colored.toImageMsg());
  // cv::imshow("depth_image", adjMap);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "pcl_render");
  ros::NodeHandle nh("~");

  nh.getParam("cam_width", width);
  nh.getParam("cam_height", height);
  nh.getParam("cam_fx", fx);
  nh.getParam("cam_fy", fy);
  nh.getParam("cam_cx", cx);
  nh.getParam("cam_cy", cy);
  nh.getParam("sensing_horizon", sensing_horizon_);
  nh.getParam("sensing_rate", sensing_rate_);
  nh.getParam("estimation_rate", estimation_rate_);

  nh.getParam("map/x_size", x_size_);
  nh.getParam("map/y_size", y_size_);
  nh.getParam("map/z_size", z_size_);

  depthrender.set_para(fx, fy, cx, cy, width, height);

  // cam02body <<  0.0148655429818, -0.999880929698, 0.00414029679422, -0.0216401454975,
  //               0.999557249008, 0.0149672133247, 0.025715529948, -0.064676986768,
  //               -0.0257744366974, 0.00375618835797, 0.999660727178, 0.00981073058949,
  //               0.0, 0.0, 0.0, 1.0;

  cam02body << 0.0, 0.0, 1.0, 0.0,
      -1.0, 0.0, 0.0, 0.0,
      0.0, -1.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 1.0;

  // init cam2world transformation
  cam2world = Matrix4d::Identity();
  // subscribe point cloud
  global_map_sub_ = nh.subscribe("global_map", 1, rcvGlobalPointCloudCallBack);
  local_map_sub_ = nh.subscribe("local_map", 1, rcvLocalPointCloudCallBack);
  odom_sub_ = nh.subscribe("odom", 50, rcvOdometryCallbck);

  // publisher depth image and color image
  pub_depth = nh.advertise<sensor_msgs::Image>("depth", 1000);
  pub_color = nh.advertise<sensor_msgs::Image>("colordepth", 1000);
  pub_pose = nh.advertise<geometry_msgs::PoseStamped>("camera_pose", 1000);
  pub_pcl_wolrd = nh.advertise<sensor_msgs::PointCloud2>("rendered_pcl", 1);

  double sensing_duration = 1.0 / sensing_rate_;
  double estimate_duration = 1.0 / estimation_rate_;

  local_sensing_timer_ = nh.createTimer(ros::Duration(sensing_duration), renderSensedPoints);
  estimation_timer = nh.createTimer(ros::Duration(estimate_duration), pubCameraPose);
  // cv::namedWindow("depth_image",1);

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
}
