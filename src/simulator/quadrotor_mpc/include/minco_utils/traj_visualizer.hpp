#ifndef TRAJ_VISUALIZER_HPP
#define TRAJ_VISUALIZER_HPP

#include <ros/ros.h>
#include <minco_utils/trajectory.hpp>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

namespace minco
{
class TrajVisualizer
{
private:
  /* data */
  ros::NodeHandle nh;
  ros::Publisher wayPointsPub;
  ros::Publisher refPointsPub;
  ros::Publisher trajectoryPub;

  visualization_msgs::Marker wayPointsMarker;
  visualization_msgs::Marker refPointsMarker;
  visualization_msgs::Marker trajMarker;

public:
  TrajVisualizer(const ros::NodeHandle& nh_, const std::string& odomFrameName)
  {
    wayPointsPub = nh.advertise<visualization_msgs::Marker>("/planner/waypoints", 10);
    refPointsPub = nh.advertise<visualization_msgs::Marker>("/planner/reference", 10);
    trajectoryPub = nh.advertise<visualization_msgs::Marker>("/planner/trajectory", 10);

    wayPointsMarker.id = 0;
    wayPointsMarker.type = visualization_msgs::Marker::SPHERE_LIST;
    wayPointsMarker.header.frame_id = odomFrameName;
    wayPointsMarker.pose.orientation.w = 1.00;
    wayPointsMarker.ns = "waypoints";
    wayPointsMarker.color.r = 1.00;
    wayPointsMarker.color.g = 0.00;
    wayPointsMarker.color.b = 0.00;
    wayPointsMarker.color.a = 1.00;
    wayPointsMarker.scale.x = 0.35;
    wayPointsMarker.scale.y = 0.35;
    wayPointsMarker.scale.z = 0.35;

    refPointsMarker = wayPointsMarker;
    refPointsMarker.ns = "reference";
    refPointsMarker.color.r = 0.50;
    refPointsMarker.color.g = 0.00;
    refPointsMarker.color.b = 1.00;
    refPointsMarker.color.a = 1.00;

    trajMarker.id = 0;
    trajMarker.type = visualization_msgs::Marker::LINE_LIST;
    trajMarker.header.frame_id = odomFrameName;
    trajMarker.pose.orientation.w = 1.00;
    trajMarker.ns = "trajectory";
    trajMarker.color.r = 0.00;
    trajMarker.color.g = 0.50;
    trajMarker.color.b = 1.00;
    trajMarker.color.a = 1.00;
    trajMarker.scale.x = 0.30;
  };
  ~TrajVisualizer() {};

  template <int D>
  inline void visualRef(const Trajectory<D>& traj, int predHorizon, float dt, float t0)
  {
    refPointsMarker.points.clear();
    refPointsMarker.header.stamp = ros::Time::now();
    double totalTime = traj.getTotalDuration();
    if (totalTime > 0.0)
    {
      for (int i = 0; i < predHorizon; ++i)
      {
        float t = t0 + i * dt;

        if (t > totalTime)
          t = totalTime;

        Eigen::Vector3d pts = traj.getPos(t);
        geometry_msgs::Point point;
        point.x = pts(0);
        point.y = pts(1);
        point.z = pts(2);

        refPointsMarker.points.emplace_back(point);
      }
      refPointsPub.publish(refPointsMarker);
    }
  };

  template <int D>
  inline void visualizeTraj(const Trajectory<D>& traj)
  {
    wayPointsMarker.points.clear();
    trajMarker.points.clear();
    wayPointsMarker.header.stamp = ros::Time::now();
    trajMarker.header.stamp = ros::Time::now();

    if (traj.getPieceNum() > 0)
    {
      Eigen::MatrixXd wps = traj.getPositions();
      for (int i = 0; i < wps.cols(); i++)
      {
        geometry_msgs::Point point;
        point.x = wps.col(i)(0);
        point.y = wps.col(i)(1);
        point.z = wps.col(i)(2);
        wayPointsMarker.points.emplace_back(point);
      }

      wayPointsPub.publish(wayPointsMarker);
    }

    if (traj.getPieceNum() > 0)
    {
      double T = 0.01;
      Eigen::Vector3d lastX = traj.getPos(0.0);
      for (double t = T; t < traj.getTotalDuration(); t += T)
      {
        geometry_msgs::Point point;
        Eigen::Vector3d X = traj.getPos(t);
        point.x = lastX(0);
        point.y = lastX(1);
        point.z = lastX(2);
        trajMarker.points.emplace_back(point);
        point.x = X(0);
        point.y = X(1);
        point.z = X(2);
        trajMarker.points.emplace_back(point);
        lastX = X;
      }
      trajectoryPub.publish(trajMarker);
    }
  };
};

}  // namespace minco

#endif