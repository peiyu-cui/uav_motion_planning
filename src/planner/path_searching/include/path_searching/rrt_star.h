#ifndef RRT_STAR_H_
#define RRT_STAR_H_

#include <ros/ros.h>
#include <Eigen/Eigen>
#include <random>
#include <unordered_set>
#include <visualization_msgs/Marker.h>

#include <kdtree/kdtree.h>
#include <plan_env/grid_map.h>

namespace path_searching {

#define inf (1 << 30)

class RRTStarNode {
  public:
    Eigen::Vector3d position;
    double g_cost;
    RRTStarNode* parent;
    std::vector<RRTStarNode*> children;

    RRTStarNode() {
      g_cost = inf;
      parent = NULL;
    }
    ~RRTStarNode() {};
  }; // class RRTStarNode
typedef RRTStarNode* RRTStarNodePtr;

class RRTStar {
  private:
    /* main data structure */
    kdtree* kdtree_;
    std::vector<RRTStarNodePtr> path_node_pool_;
    std::vector<Eigen::Vector3d> optimal_path_;

    /* main search parameters */
    int max_tree_node_num_;
    int use_node_num_;
    double step_length_;
    double search_radius_;
    double tolerance_;
    double max_tolerance_time_;
    bool reach_goal_;

    /* main map parameters */
    GridMap::Ptr grid_map_;
    Eigen::Vector3d origin_;
    Eigen::Vector3d map_size_;
    double resolution_;


    /* main ros publishers and subscribers */
    ros::Publisher vis_path_pub_;
    ros::Publisher vis_waypoints_pub_;
    ros::Publisher vis_tree_pub_;
    visualization_msgs::Marker vis_path_marker_;
    visualization_msgs::Marker vis_waypoints_marker_;
    visualization_msgs::Marker vis_tree_marker_;

    enum {
      REACH_END = 1,
      NO_PATH_FOUND = 2
    };

    /* main helper functions */
    Eigen::Vector3d getRandomNode();
    Eigen::Vector3d Step(Eigen::Vector3d from, Eigen::Vector3d to, double step_length);
    bool isCollisionFree(Eigen::Vector3d from, Eigen::Vector3d to, double map_resolution);
    RRTStarNodePtr ChooseParent(Eigen::Vector3d x_new);
    void ReWireTree(RRTStarNodePtr& new_node);
    void retrievePath(RRTStarNodePtr end_node, std::vector<Eigen::Vector3d>& path);
    void visFeasiblePath(std::vector<Eigen::Vector3d> path);


  public:
    /* main interface function */
    void setParam(ros::NodeHandle& nh);
    void setGridMap(GridMap::Ptr& grid_map);

    void getWholeTree(std::vector<Eigen::Vector3d> &vertices, 
    std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> &edges);
    void visWholeTree(std::vector<Eigen::Vector3d> vertices, 
        std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> edges);
    std::vector<Eigen::Vector3d> getOptimalPath();

    /* main init function and reset function */
    void init();
    void reset();

    /* main search function */
    int search(Eigen::Vector3d start, Eigen::Vector3d end, std::vector<Eigen::Vector3d>& path);

    RRTStar() {};
    ~RRTStar();

    typedef std::shared_ptr<RRTStar> Ptr;

}; // class RRTStar

} // namespace path_searching

#endif // RRT_STAR_H_