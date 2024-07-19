#ifndef KINO_ASTAR_H_
#define KINO_ASTAR_H_

#include <ros/ros.h>
#include <Eigen/Eigen>
#include <queue>
#include <unordered_map>
#include <pcl/kdtree/kdtree_flann.h>
#include <boost/make_shared.hpp>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <plan_env/grid_map.h>

namespace path_searching
{

typedef pcl::PointXYZ PCLPoint;
typedef pcl::PointCloud<PCLPoint> PCLPointCloud;
typedef pcl::KdTreeFLANN<PCLPoint> KdTree;

#define inf (1 << 30);
#define IN_OPEN_LIST_   'a';
#define IN_CLOSE_LIST_  'b';
#define NOT_EXPANDED_   'c';


class KinoAstarNode
{
  public:
    Eigen::Vector3d position;
    Eigen::Vector3d velocity;
    Eigen::Vector3d input;
    Eigen::Vector3i index; // index of the node in the occupancy grid, used for pruning
    double g_cost;
    double f_cost;
    double duration;
    KinoAstarNode* parent;
    char node_state;

    KinoAstarNode()
    {
      g_cost = inf;
      f_cost = inf;
      parent = NULL;
      duration = inf;
      input = Eigen::Vector3d::Zero();
      node_state = NOT_EXPANDED_;
    }
    ~KinoAstarNode() {};

}; // class KinoAstarNode
typedef KinoAstarNode* KinoAstarNodePtr;

class KinoAstarNodeComparator
{
  public:
    bool operator() (KinoAstarNodePtr node1, KinoAstarNodePtr node2)
    {
      return node1->f_cost > node2->f_cost;
    }
}; // class KinoAstarNodeComparator

template<typename T>
struct vector3i_hash: public std::unary_function<T, size_t>
{
    size_t operator()(const T& vector) const
    {
        size_t seed = 0;
        for (size_t i = 0; i < vector.size(); ++i)
        {
          auto elem = *(vector.data() + i);
          seed ^= std::hash<typename T::Scalar>()(elem) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
        }

        return seed;
    }
}; // struct vector3i_hash

class KinoAstarNodeHashTable
{
  private:
    std::unordered_map<Eigen::Vector3i, KinoAstarNodePtr, vector3i_hash<Eigen::Vector3i>> data_table_;
  
  public:

    void insert(Eigen::Vector3i index, KinoAstarNodePtr node)
    {
      data_table_.insert(std::make_pair(index, node));
    }

    KinoAstarNodePtr find(Eigen::Vector3i index)
    {
      auto iter = data_table_.find(index);
      if (iter != data_table_.end())
      {
        return iter->second;
      }
      else
      {
        return NULL;
      }
    }

    void erase(Eigen::Vector3i index)
    {
      data_table_.erase(index);
    }

    void clear ()
    {
      data_table_.clear();
    }

    KinoAstarNodeHashTable() {};
    ~KinoAstarNodeHashTable() {};

}; // class KinoAstarNodeHashTable
class KinoAstar 
{
  private:
    /* main data structure */
    std::priority_queue<KinoAstarNodePtr, std::vector<KinoAstarNodePtr>, KinoAstarNodeComparator> open_list_;
    KinoAstarNodeHashTable close_list_;
    KinoAstarNodeHashTable expanded_list_;
    std::vector<KinoAstarNodePtr> path_node_pool_;
    std::vector<Eigen::Matrix3d> rot_list;

    /* main search parameters */
    int allocated_node_num_;
    int use_node_num_;
    int collision_check_type_;
    double acc_res_;
    double rou_;
    double lambda_heu_;
    double tie_breaker_;
    double goal_tolerance_;
    double step_size_;
    double max_vel_;
    double max_accel_;
    double sample_tau_;

    Eigen::Matrix<double, 3, 4> shot_coef_;
    Eigen::Matrix<double, 3, 4> vel_coef_;
    Eigen::Matrix<double, 3, 4> acc_coef_;

    /* main robot parameters */
    double robot_r_;
    double robot_h_;

    /* main visulization parameters */
    ros::Publisher path_node_pub_;
    visualization_msgs::Marker path_node_marker_;

    ros::Publisher elliposid_pub_;

    enum {
      REACH_END = 1,
      NO_PATH_FOUND = 2
    };

    /* mian map parameters: grid map */
    Eigen::Vector3d origin_;
    Eigen::Vector3d map_size_;
    double resolution_;
    double inv_resolution_;
    GridMap::Ptr grid_map_;
    /* main map parameters: cloud map */
    std::vector<Eigen::Vector3d> obs_;
    KdTree kdtree_;
    ros::Subscriber local_cloud_sub_;

    /* main helper functions */
    Eigen::Vector3i posToIndex(Eigen::Vector3d pos);
    double getHeuristicCost(Eigen::Vector3d x1, Eigen::Vector3d v1,
                            Eigen::Vector3d x2, Eigen::Vector3d v2,
                            double &optimal_time);
    std::vector<double> cubic(double a, double b, double c, double d);
    std::vector<double> quartic(double a, double b, double c, double d, double e);
    bool computeShotTraj(Eigen::Vector3d x1, Eigen::Vector3d v1,
                            Eigen::Vector3d x2, Eigen::Vector3d v2,
                            double optimal_time);
    std::vector<KinoAstarNodePtr> retrievePath(KinoAstarNodePtr end_node, std::vector<Eigen::Vector3d>& path_nodes_list);
    void samplePath(std::vector<KinoAstarNodePtr> path_pool, std::vector<Eigen::Vector3d>& path);
    void sampleEllipsoid(std::vector<KinoAstarNodePtr> path_pool, std::vector<Eigen::Vector3d>& path,
                         std::vector<Eigen::Matrix3d> &rot_list);
    void StateTransit(Eigen::Matrix<double, 6, 1> &x0, Eigen::Matrix<double, 6, 1> &xt, Eigen::Vector3d ut, double dt);

    /* main se3 helper functions */
    void localCloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg);
    bool isCollisionFree(Eigen::Vector3d pt, Eigen::Vector3d acc);
    PCLPointCloud toPCL(const std::vector<Eigen::Vector3d>& obs);

  public:
    void setParam(ros::NodeHandle& nh);
    void init();
    void setGridMap(GridMap::Ptr& grid_map);
    // second-order interator model 
    // acceleration-controller motion planning
    int search(Eigen::Vector3d start_pt, Eigen::Vector3d start_vel,
         Eigen::Vector3d end_pt, Eigen::Vector3d end_vel,
         std::vector<Eigen::Vector3d>& path);
    void reset();
    void visPathNodes(std::vector<Eigen::Vector3d>& path_nodes_list);
    void visEllipsoid(std::vector<Eigen::Vector3d>& path_nodes_list, std::vector<Eigen::Matrix3d> &rot_list);

    KinoAstar() {};
    ~KinoAstar();

    typedef std::shared_ptr<KinoAstar> Ptr;

}; // class KinoAstar


} // namespace path_searching
#endif // KINO_ASTAR_H_