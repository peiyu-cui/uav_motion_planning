#ifndef KINO_SE3_H_
#define KINO_SE3_H_

#include <ros/ros.h>
#include <Eigen/Eigen>
#include <pcl/kdtree/kdtree_flann.h>
#include <boost/make_shared.hpp>

#include <path_searching/kino_astar.h>

typedef pcl::PointXYZ PCLPoint;
typedef pcl::PointCloud<PCLPoint> PCLPointCloud;
typedef pcl::KdTreeFLANN<PCLPoint> KdTree;

namespace path_searching {

class KinoSE3 {
    private:
        /* main robot parameters */
        double robot_r_;
        double robot_h_;

        /* main map parameters */
        std::vector<Eigen::Vector3d> obs_;
        KdTree kdtree_;
        ros::Subscriber local_cloud_sub_;

        /* main cloud map functions */
        void localCloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg);

        /* main collision checking functions */
        bool isCollisionFree(Eigen::Vector3d pt, Eigen::Vector3d acc);
        PCLPointCloud toPCL(const std::vector<Eigen::Vector3d>& obs);

    public:
        KinoSE3() {};
        ~KinoSE3() {};

        void setParam(ros::NodeHandle& nh);
        
        typedef std::shared_ptr<KinoSE3> Ptr;

}; // class KinoSE3


} // namespace path_searching



#endif // KINO_SE3_H_