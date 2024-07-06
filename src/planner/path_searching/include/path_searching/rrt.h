#ifndef RRT_H_
#define RRT_H_

#include <Eigen/Eigen>
#include <ros/ros.h>
#include <random>
#include <queue>

#include <plan_env/grid_map.h>
#include <kdtree/kdtree.h>

namespace path_searching {

#define inf (1 << 30);

  class RRTPathNode {
    public:
      Eigen::Vector3d position;
      double g_cost;                      // for total_cost
      RRTPathNode* parent;                // for retrieve path
      std::vector<RRTPathNode*> children; // for get whole tree

      RRTPathNode() {
        g_cost = inf;
        parent = NULL;
      }

      ~RRTPathNode() {} 

  }; // class PathNode
  typedef RRTPathNode* RRTPathNodePtr;

  class RRT {
    private:
      /* main data structure */
      kdtree* kdtree_;
      std::vector<RRTPathNodePtr> path_node_pool_;

      /* main search parameters */
      int max_tree_node_num_;
      int use_node_num_;
      double step_length_;
      double max_allowed_time_;
      double tolerance_;
      double search_radius_;

      /* main map parameters */
      GridMap::Ptr grid_map_;
      Eigen::Vector3d origin_;
      Eigen::Vector3d map_size_;
      double resolution_;

      enum {
        REACH_END = 1,
        NO_PATH_FOUND = 2
      };

      /* main helper functions */
      Eigen::Vector3d getRandomNode();
      Eigen::Vector3d Step(Eigen::Vector3d from, Eigen::Vector3d to, double step_length);
      bool isCollisionFree(Eigen::Vector3d from, Eigen::Vector3d to, double map_resolution);
      void retrievePath(RRTPathNodePtr node, std::vector<Eigen::Vector3d>& path);

    public:
      /* main interface */
      void setParam(ros::NodeHandle& nh);
      void setGridMap(GridMap::Ptr& grid_map);
      void getWholeTree(std::vector<Eigen::Vector3d> &vertices, 
              std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> &edges);

      /* main init and reset function */
      void init();
      void reset();

      /* main search algorithm */
      int search(Eigen::Vector3d start_pt, Eigen::Vector3d end_pt, std::vector<Eigen::Vector3d>& path);
      
      RRT() {};
      ~RRT();

      typedef std::shared_ptr<RRT> Ptr;

  }; // class RRT

} // namespace path_searching


#endif // RRT_H_