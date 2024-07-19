#include <path_searching/kino_se3.h>


namespace path_searching {

void KinoSE3::setParam(ros::NodeHandle& nh)
{
    nh.param("kino_se3/robot_r", robot_r_, 0.2);
    nh.param("kino_se3/robot_h", robot_h_, 0.1);
    
    local_cloud_sub_ = nh.subscribe<sensor_msgs::PointCloud2>("local_cloud", 10, &KinoSE3::localCloudCallback, this);
}

void KinoSE3::localCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
    // insert local cloud to kdtree_
    obs_.clear();
    PCLPointCloud latest_cloud;
    pcl::fromROSMsg(*msg, latest_cloud);
    for (size_t i = 0; i < latest_cloud.points.size(); ++i)
    {
        Eigen::Vector3d pt;
        pt(0) = latest_cloud.points[i].x;
        pt(1) = latest_cloud.points[i].y;
        pt(2) = latest_cloud.points[i].z;
        obs_.push_back(pt);
    }
    PCLPointCloud::Ptr cloud_ptr = boost::make_shared<PCLPointCloud>(toPCL(obs_));
    kdtree_.setInputCloud(cloud_ptr);
}

bool KinoSE3::isCollisionFree(Eigen::Vector3d pt, Eigen::Vector3d acc)
{
    // check collision with local cloud
    Eigen::Vector3d b3 = (acc + 9.81 * Eigen::Vector3d::UnitZ()).normalized();
    Eigen::Vector3d c1 (cos(0), sin(0), 0);
    Eigen::Vector3d b2 = b3.cross(c1).normalized();
    Eigen::Vector3d b1 = b2.cross(b3);

    Eigen::Matrix3d Rot;
    Rot.col(0) = b1;
    Rot.col(1) = b2;
    Rot.col(2) = b3;

    Eigen::Matrix3d P = Eigen::Matrix3d::Zero();
    P(0, 0) = robot_r_;
    P(1, 1) = robot_r_;
    P(2, 2) = robot_h_;

    Eigen::Matrix3d E = Rot * P * Rot.transpose();

    pcl::PointXYZ searchPoint;
    std::vector<int> pointIdxRadiusSearch;
    std::vector<float> pointRadiusSquaredDistance;

    searchPoint.x = pt(0);
    searchPoint.y = pt(1);
    searchPoint.z = pt(2);

    float radius = robot_r_ * 2;
    if (kdtree_.radiusSearch(searchPoint, radius, pointIdxRadiusSearch,
        pointRadiusSquaredDistance) > 0)
    {
        for (size_t i = 0; i < pointIdxRadiusSearch.size(); ++i)
        {
            Eigen::Vector3d tmp_pt = E.inverse() * (obs_[pointIdxRadiusSearch[i]] - pt);
            if (tmp_pt.norm() <= 1.0) return false;
        }
    }
    return true;
}

  /// Convert obstacle points into pcl point cloud
PCLPointCloud KinoSE3::toPCL(const std::vector<Eigen::Vector3d> &obs) {
    PCLPointCloud cloud;
    cloud.width = obs.size();
    cloud.height = 1;
    cloud.points.resize(cloud.width * cloud.height);
    for (unsigned int i = 0; i < obs.size(); i++) {
      cloud.points[i].x = obs[i](0);
      cloud.points[i].y = obs[i](1);
      cloud.points[i].z = obs[i](2);
    }
    return cloud;
  }

} // namespace path_searching