#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
// #include <pcl/search/kdtree.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <iostream>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3.h>
#include <math.h>
#include <nav_msgs/Odometry.h>
#include <ros/console.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <Eigen/Eigen>
#include <random>

using namespace std;

// pcl::search::KdTree<pcl::PointXYZ> kdtreeLocalMap;
pcl::KdTreeFLANN<pcl::PointXYZ> kdtreeLocalMap;

vector<int> pointIdxRadiusSearch;
vector<float> pointRadiusSquaredDistance;

random_device rd;
default_random_engine eng(rd());

uniform_real_distribution<double> rand_x_, rand_y_, rand_z_, rand_w_, rand_h_, rand_inf_, rand_radius1_, rand_radius2_, rand_theta_;

ros::Publisher local_map_pub_, global_map_pub_, click_map_pub_;
ros::Subscriber odom_sub_, click_sub_;

vector<double> state_;

int polar_num_, num_circles_, fix_map_type_;
double x_size_, y_size_, init_x_, init_y_;
double x_l_, x_h_, y_l_, y_h_, z_l_, z_h_, w_l_, w_h_, h_l_, h_h_, radius_l_, radius_h_, theta_, init_radius_;
double sensing_range_, resolution_, sense_rate_, min_dist_;
double wall_x_, wall_y_, wall_w_;

bool map_ok_ = false, has_odom_ = false;

sensor_msgs::PointCloud2 global_pcd_, local_pcd_;

pcl::PointCloud<pcl::PointXYZ> global_cloud_, clicked_cloud_;

bool checkClearance(double x, double y)
{
  Eigen::Vector2d pt(x, y);
  Eigen::Vector2d init_pt(init_x_, init_y_);
  return (pt - init_pt).norm() >= init_radius_;
}

void RandomMapGenerate()
{
  pcl::PointXYZ pt_random;

  rand_x_ = uniform_real_distribution<double>(x_l_, x_h_);
  rand_y_ = uniform_real_distribution<double>(y_l_, y_h_);
  rand_z_ = uniform_real_distribution<double>(z_l_, z_h_);
  rand_w_ = uniform_real_distribution<double>(w_l_, w_h_);
  rand_h_ = uniform_real_distribution<double>(h_l_, h_h_);
  rand_radius1_ = uniform_real_distribution<double>(radius_l_, radius_h_);
  rand_radius2_ = uniform_real_distribution<double>(radius_l_, radius_h_);
  rand_theta_ = uniform_real_distribution<double>(-theta_, theta_);

  // generate polar obs
  for (int i = 0; i < polar_num_; i++)
  {
    double x, y, w, h;
    x = rand_x_(eng);
    y = rand_y_(eng);
    w = rand_w_(eng);
    if (!checkClearance(x, y))
    {
      i--;
      continue;
    }

    x = floor(x / resolution_) * resolution_ + resolution_ / 2.0;
    y = floor(y / resolution_) * resolution_ + resolution_ / 2.0;

    int num_w = ceil(w / resolution_);

    for (int r = -num_w / 2.0; r < num_w / 2.0; r++)
    {
      for (int s = -num_w / 2.0; s < num_w / 2.0; s++)
      {
        h = rand_h_(eng);
        int num_h = ceil(h / resolution_);
        for (int t = -20; t < num_h; t++)
        {
          pt_random.x = x + (r + 0.5) * resolution_ + 1e-2;
          pt_random.y = y + (s + 0.5) * resolution_ + 1e-2;
          pt_random.z = (t + 0.5) * resolution_ + 1e-2;
          global_cloud_.points.push_back(pt_random);
        }
      }
    }
  }

  // generate circle obs
  for (int i = 0; i < num_circles_; i++)
  {
    double x, y, z, radius1, radius2, theta;
    x = rand_x_(eng);
    y = rand_y_(eng);
    z = rand_z_(eng);
    radius1 = rand_radius1_(eng);
    radius2 = rand_radius2_(eng);
    theta = rand_theta_(eng);

    if (!checkClearance(x, y))
    {
      i--;
      continue;
    }

    x = floor(x / resolution_) * resolution_ + resolution_ / 2.0;
    y = floor(y / resolution_) * resolution_ + resolution_ / 2.0;
    z = floor(z / resolution_) * resolution_ + resolution_ / 2.0;
    Eigen::Vector3d translate(x, y, z);

    Eigen::Matrix3d rotate;
    rotate << cos(theta), -sin(theta), 0.0, sin(theta), cos(theta), 0.0, 0, 0, 1;

    // draw a circle centered at (x,y,z)
    Eigen::Vector3d cpt;
    for (double angle = 0.0; angle < M_PI * 2; angle += resolution_ / 2)
    {
      cpt(0) = 0.0;
      cpt(1) = radius1 * cos(angle);
      cpt(2) = radius2 * sin(angle);

      // inflate
      Eigen::Vector3d cpt_if;
      for (int ifx = -0; ifx <= 0; ifx++)
      {
        for (int ify = -0; ify <= 0; ify++)
        {
          for (int ifz = -0; ifz <= 0; ifz++)
          {
            cpt_if = cpt + Eigen::Vector3d(ifx * resolution_, ify * resolution_, ifz * resolution_);
            cpt_if = rotate * cpt_if + translate;
            pt_random.x = cpt_if(0);
            pt_random.y = cpt_if(1);
            pt_random.z = cpt_if(2);
            global_cloud_.push_back(pt_random);
          }
        }
      }
    }
  }
}

void RandomMapGenerateCylinder()
{
  pcl::PointXYZ pt_random;

  vector<Eigen::Vector2d> obs_position;

  rand_x_ = uniform_real_distribution<double>(x_l_, x_h_);
  rand_y_ = uniform_real_distribution<double>(y_l_, y_h_);
  rand_z_ = uniform_real_distribution<double>(z_l_, z_h_);
  rand_w_ = uniform_real_distribution<double>(w_l_, w_h_);
  rand_h_ = uniform_real_distribution<double>(h_l_, h_h_);
  rand_inf_ = uniform_real_distribution<double>(0.5, 1.5);

  rand_radius1_ = uniform_real_distribution<double>(radius_l_, radius_h_);
  rand_radius2_ = uniform_real_distribution<double>(radius_l_, radius_h_);
  rand_theta_ = uniform_real_distribution<double>(-theta_, theta_);

  // generate polar obs
  for (int i = 0; i < polar_num_; i++)
  {
    double x, y, w, h, inf;
    x = rand_x_(eng);
    y = rand_y_(eng);
    w = rand_w_(eng);
    inf = rand_inf_(eng);
    if (!checkClearance(x, y))
    {
      i--;
      continue;
    }

    bool flag_continue = false;
    for (auto p : obs_position)
    {
      if ((Eigen::Vector2d(x, y) - p).norm() < min_dist_ /*metres*/)
      {
        i--;
        flag_continue = true;
        break;
      }
    }
    if (flag_continue)
      continue;

    obs_position.push_back(Eigen::Vector2d(x, y));

    x = floor(x / resolution_) * resolution_ + resolution_ / 2.0;
    y = floor(y / resolution_) * resolution_ + resolution_ / 2.0;

    int num_w = ceil((w * inf) / resolution_);
    double radius = (w * inf) / 2;

    for (int r = -num_w / 2.0; r < num_w / 2.0; r++)
    {
      for (int s = -num_w / 2.0; s < num_w / 2.0; s++)
      {
        h = rand_h_(eng);
        int num_h = ceil(h / resolution_);
        for (int t = -20; t < num_h; t++)
        {
          double temp_x = x + (r + 0.5) * resolution_ + 1e-2;
          double temp_y = y + (s + 0.5) * resolution_ + 1e-2;
          double temp_z = (t + 0.5) * resolution_ + 1e-2;
          if ((Eigen::Vector2d(temp_x, temp_y) - Eigen::Vector2d(x, y)).norm() <= radius)
          {
            pt_random.x = temp_x;
            pt_random.y = temp_y;
            pt_random.z = temp_z;
            global_cloud_.points.push_back(pt_random);
          }
        }
      }
    }
  }

  // generate circle obs
  for (int i = 0; i < num_circles_; ++i)
  {
    double x, y, z, radius1, radius2, theta;
    x = rand_x_(eng);
    y = rand_y_(eng);
    z = rand_z_(eng);
    radius1 = rand_radius1_(eng);
    radius2 = rand_radius2_(eng);
    theta = rand_theta_(eng);

    if (!checkClearance(x, y))
    {
      i--;
      continue;
    }

    x = floor(x / resolution_) * resolution_ + resolution_ / 2.0;
    y = floor(y / resolution_) * resolution_ + resolution_ / 2.0;
    z = floor(z / resolution_) * resolution_ + resolution_ / 2.0;
    Eigen::Vector3d translate(x, y, z);

    Eigen::Matrix3d rotate;
    rotate << cos(theta), -sin(theta), 0.0, sin(theta), cos(theta), 0.0, 0, 0, 1;

    // draw a circle centered at (x,y,z)
    Eigen::Vector3d cpt;
    for (double angle = 0.0; angle < M_PI * 2; angle += resolution_ / 2)
    {
      cpt(0) = 0.0;
      cpt(1) = radius1 * cos(angle);
      cpt(2) = radius2 * sin(angle);

      // inflate
      Eigen::Vector3d cpt_if;
      for (int ifx = -0; ifx <= 0; ++ifx)
      {
        for (int ify = -0; ify <= 0; ++ify)
        {
          for (int ifz = -0; ifz <= 0; ++ifz)
          {
            cpt_if = cpt + Eigen::Vector3d(ifx * resolution_, ify * resolution_, ifz * resolution_);
            cpt_if = rotate * cpt_if + translate;
            pt_random.x = cpt_if(0);
            pt_random.y = cpt_if(1);
            pt_random.z = cpt_if(2);
            global_cloud_.push_back(pt_random);
          }
        }
      }
    }
  }
}

void GenerateWall(double x_min, double x_max, double y_min, double y_max, double z_min, double z_max)
{
  int num_x, num_y, num_z;
  num_x = ceil((x_max - x_min) / resolution_);
  num_y = ceil((y_max - y_min) / resolution_);
  num_z = ceil((z_max - z_min) / resolution_);
  for (int i = 0; i < num_x; i++)
  {
    for (int j = 0; j < num_y; j++)
    {
      for (int k = 0; k < num_z; k++)
      {
        pcl::PointXYZ pt;
        pt.x = x_min + i * resolution_;
        pt.y = y_min + j * resolution_;
        pt.z = z_min + k * resolution_;
        global_cloud_.push_back(pt);
      }
    }
  }
}

void GenerateCircle(double x, double y, double z, double radius, double theta)
{
  x = floor(x / resolution_) * resolution_ + resolution_ / 2.0;
  y = floor(y / resolution_) * resolution_ + resolution_ / 2.0;
  z = floor(z / resolution_) * resolution_ + resolution_ / 2.0;

  Eigen::Vector3d translate(x, y, z);
  Eigen::Matrix3d rotate;
  rotate << cos(theta), -sin(theta), 0.0, sin(theta), cos(theta), 0.0, 0, 0, 1;

  for (double angle = 0.0; angle < 6.282; angle += resolution_ / 2)
  {
    Eigen::Vector3d cpt;
    cpt(0) = 0.0;
    cpt(1) = radius * cos(angle);
    cpt(2) = radius * sin(angle);

    cpt = rotate * cpt + translate;

    pcl::PointXYZ pt;
    pt.x = cpt(0);
    pt.y = cpt(1);
    pt.z = cpt(2);
    global_cloud_.push_back(pt);
  }
}

void MapGenerate()
{
  switch (fix_map_type_)
  {
    case 0: {
      RandomMapGenerate();
      break;
    }
    case 1: {
      RandomMapGenerateCylinder();
      break;
    }
    case 2: {
      GenerateWall(wall_x_ - 0.25, wall_x_ + 0.25, wall_y_ + wall_w_ / 2.0, wall_y_ + 20.0, -0.5, 4.0);
      GenerateWall(wall_x_ - 0.25, wall_x_ + 0.25, wall_y_ - 20.0, wall_y_ - wall_w_ / 2.0, -0.5, 4.0);
      break;
    }
    case 3: {
      GenerateCircle(3.0, 0.0, 3.0, 2.0, 0.0);
      break;
    }
    case 4: {
      double r1 = 0.45, r2 = 0.6, width_2 = 0.05;

      GenerateCircle(0.0, -0.6, 0.9, r1, M_PI_2);
      GenerateCircle(0.0, -0.6, 0.9, r2, M_PI_2);
      GenerateWall(0.0 - r1 - width_2, 0.0 + r1 + width_2, -0.6 - width_2, -0.6 + width_2, 0, 0.9 - r1);
      // GenerateWall(0.0 + r1 - width_2, 0.0 + r1 + width_2, -0.6 - width_2, -0.6 + width_2, 0, 0.9 + r1);
      // GenerateWall(0.0 - r1 - width_2, 0.0 - r1 + width_2, -0.6 - width_2, -0.6 + width_2, 0, 0.9 + r1);

      GenerateCircle(0.0, 2.4, 1.2, r1, M_PI_2);
      GenerateCircle(0.0, 2.4, 1.2, r2, M_PI_2);
      GenerateWall(0.0 - r1 - width_2, 0.0 + r1 + width_2, 2.4 - width_2, 2.4 + width_2, 0, 1.1 - r1);
      // GenerateWall(0.0 + r1 - width_2, 0.0 + r1 + width_2, 2.4 - width_2, 2.4 + width_2, 0, 1.1 + r1);
      // GenerateWall(0.0 - r1 - width_2, 0.0 - r1 + width_2, 2.4 - width_2, 2.4 + width_2, 0, 1.1 + r1);

      width_2 = 0.1;
      GenerateWall(0.6 - width_2, 0.6 + width_2, 0.6 - width_2, 0.6 + width_2, 0, 2.0);
      GenerateWall(-0.6 - width_2, -0.6 + width_2, 0.6 - width_2, 0.6 + width_2, 0, 2.0);
      GenerateWall(0.0 - width_2, 0.0 + width_2, 1.2 - width_2, 1.2 + width_2, 0, 2.0);
      break;
    }
  }

  global_cloud_.width = global_cloud_.points.size();
  global_cloud_.height = 1;
  global_cloud_.is_dense = true;

  ROS_WARN("Finished generate fixed map");
  // kdtreeLocalMap.setInputCloud(global_cloud_.makeShared());
  map_ok_ = true;
}

void pubSensedPoints()
{
  pcl::toROSMsg(global_cloud_, global_pcd_);
  global_pcd_.header.frame_id = "world";
  global_map_pub_.publish(global_pcd_);
  return;

  /* ---------- only publish points around current position ---------- */
  // if (!map_ok_ || !has_odom_)
  //   return;

  // pcl::PointCloud<pcl::PointXYZ> local_map;

  // pcl::PointXYZ searchPoint(state_[0], state_[1], state_[2]);
  // pointIdxRadiusSearch.clear();
  // pointRadiusSquaredDistance.clear();

  // pcl::PointXYZ pt;

  // if (isnan(searchPoint.x) || isnan(searchPoint.y) || isnan(searchPoint.z))
  //   return;

  // if (kdtreeLocalMap.radiusSearch(searchPoint, sensing_range_, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0)
  // {
  //   for (size_t i = 0; i < pointIdxRadiusSearch.size(); i++)
  //   {
  //     pt = global_cloud_.points[pointIdxRadiusSearch[i]];
  //     local_map.points.push_back(pt);
  //   }
  // }
  // else
  // {
  //   ROS_ERROR("[Map server] No obstacles .");
  //   return;
  // }

  // local_map.width = local_map.points.size();
  // local_map.height = 1;
  // local_map.is_dense = true;

  // pcl::toROSMsg(local_map, local_pcd_);
  // local_pcd_.header.frame_id = "world";
  // local_map_pub_.publish(local_pcd_);
}

void clickCallback(const geometry_msgs::PoseStamped& msg)
{
  double x = msg.pose.position.x;
  double y = msg.pose.position.y;
  double h = h_h_;
  double w = 0.15;
  // double w = rand_w_(eng);
  // double h;

  pcl::PointXYZ pt_random;

  x = floor(x / resolution_) * resolution_ + resolution_ / 2.0;
  y = floor(y / resolution_) * resolution_ + resolution_ / 2.0;

  int num_w = ceil(w / resolution_);

  for (int r = -num_w / 2.0; r < num_w / 2.0; r++)
  {
    for (int s = -num_w / 2.0; s < num_w / 2.0; s++)
    {
      // h = rand_h_(eng);
      int num_h = ceil(h / resolution_);
      for (int t = -20; t < num_h; t++)
      {
        pt_random.x = x + (r + 0.5) * resolution_ + 1e-2;
        pt_random.y = y + (s + 0.5) * resolution_ + 1e-2;
        pt_random.z = (t + 0.5) * resolution_ + 1e-2;
        clicked_cloud_.points.push_back(pt_random);
        global_cloud_.points.push_back(pt_random);
      }
    }
  }

  clicked_cloud_.width = clicked_cloud_.points.size();
  clicked_cloud_.height = 1;
  clicked_cloud_.is_dense = true;

  pcl::toROSMsg(clicked_cloud_, local_pcd_);
  local_pcd_.header.frame_id = "world";
  click_map_pub_.publish(local_pcd_);

  global_cloud_.width = global_cloud_.points.size();

  pubSensedPoints();
  return;
}

void odomCallbck(const nav_msgs::Odometry odom)
{
  if (odom.child_frame_id == "X" || odom.child_frame_id == "O")
    return;

  has_odom_ = true;
  state_ = { odom.pose.pose.position.x,
             odom.pose.pose.position.y,
             odom.pose.pose.position.z,
             odom.twist.twist.linear.x,
             odom.twist.twist.linear.y,
             odom.twist.twist.linear.z,
             0.0,
             0.0,
             0.0 };
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "random_forest");
  ros::NodeHandle nh("~");

  // local_map_pub_ = nh.advertise<sensor_msgs::PointCloud2>("local_cloud", 1);
  global_map_pub_ = nh.advertise<sensor_msgs::PointCloud2>("global_cloud", 1);
  click_map_pub_ = nh.advertise<sensor_msgs::PointCloud2>("dynamic_cloud", 1);

  // odom_sub_ = nh.subscribe("odom", 10, odomCallbck);
  click_sub_ = nh.subscribe("/move_base_simple/goal", 10, clickCallback);

  nh.param("init_state_x", init_x_, 0.0);
  nh.param("init_state_y", init_y_, 0.0);
  nh.param("init_radius", init_radius_, 1.0);

  nh.param("map/x_size", x_size_, 50.0);
  nh.param("map/y_size", y_size_, 50.0);
  nh.param("map/resolution", resolution_, 0.1);

  nh.param("map/fix_map_type", fix_map_type_, 0);

  nh.param("map/polar_num", polar_num_, 30);
  nh.param("ObstacleShape/lower_rad", w_l_, 0.3);
  nh.param("ObstacleShape/upper_rad", w_h_, 0.8);
  nh.param("ObstacleShape/lower_hei", h_l_, 3.0);
  nh.param("ObstacleShape/upper_hei", h_h_, 7.0);
  nh.param("min_distance", min_dist_, 1.0);

  nh.param("wall_x", wall_x_, 0.0);
  nh.param("wall_y", wall_y_, 0.0);
  nh.param("wall_w", wall_w_, 0.5);

  nh.param("map/circle_num", num_circles_, 30);
  nh.param("ObstacleShape/radius_l", radius_l_, 7.0);
  nh.param("ObstacleShape/radius_h", radius_h_, 7.0);
  nh.param("ObstacleShape/z_l", z_l_, 7.0);
  nh.param("ObstacleShape/z_h", z_h_, 7.0);
  nh.param("ObstacleShape/theta", theta_, 7.0);

  nh.param("sensing/radius", sensing_range_, 10.0);
  nh.param("sensing/rate", sense_rate_, 10.0);

  int seed;
  nh.param("map/seed", seed, 0);

  x_l_ = -x_size_ / 2.0;
  x_h_ = +x_size_ / 2.0;
  y_l_ = -y_size_ / 2.0;
  y_h_ = +y_size_ / 2.0;

  if (seed != 0)
    eng.seed(seed);

  MapGenerate();

  ros::Rate loop_rate(sense_rate_);
  while (ros::ok())
  {
    pubSensedPoints();
    ros::spinOnce();
    loop_rate.sleep();
  }
}
