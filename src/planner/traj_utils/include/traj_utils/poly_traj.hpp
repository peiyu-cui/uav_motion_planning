#ifndef _POLY_TRAJ_HPP_
#define _POLY_TRAJ_HPP_

#include <Eigen/Eigen>
#include <vector>

using std::vector;

class PolyTraj
{
private:
  vector<double> times; // time of each segment

  // from c0 to cn
  vector<vector<double>> cxs; // coefficients of x of each segment
  vector<vector<double>> cys; // coefficients of y of each segment
  vector<vector<double>> czs; // coefficients of z of each segment

  int num_seg;
  double total_time;

  // evaluation
  vector<Eigen::Vector3d> traj_vec3d;
  double length;

public:
  PolyTraj();
  ~PolyTraj();

  void reset();
  void addSegment(vector<double> cx, vector<double> cy, vector<double> cz, double t);
  void init();

  Eigen::Vector3d evaluatePos(double t);
  Eigen::Vector3d evaluateVel(double t);
  Eigen::Vector3d evaluateAcc(double t);

  double getTotalTIme();
  vector<Eigen::Vector3d> getTraj();
  double getLength();
  double getMeanVel();
};

PolyTraj::PolyTraj()
{
}

PolyTraj::~PolyTraj()
{
}

void PolyTraj::reset()
{
  times.clear(), cxs.clear(), cys.clear(), czs.clear();
  num_seg = 0, total_time = 0.0;
  traj_vec3d.clear(), length = 0.0;
}

void PolyTraj::addSegment(vector<double> cx, vector<double> cy, vector<double> cz, double t)
{
  cxs.push_back(cx), cys.push_back(cy), czs.push_back(cz), times.push_back(t);
}

void PolyTraj::init()
{
  num_seg = times.size();
  total_time = 0.0;
  for (int i = 0; i < num_seg; i++)
  {
    total_time += times[i];
  }
}

Eigen::Vector3d PolyTraj::evaluatePos(double t)
{
  // determine segment index
  int idx = 0;
  while (t > times[idx] + 1e-4 && idx < num_seg)
  {
    t -= times[idx];
    idx++;
  }

  if (idx == num_seg)
  {
    idx--;
    t = times[idx];
  }

  // evaluation
  int order = cxs[idx].size();
  Eigen::VectorXd cx(order), cy(order), cz(order), tv(order);
  for (int i = 0; i < order; i++)
  {
    cx[i] = cxs[idx][i];
    cy[i] = cys[idx][i];
    cz[i] = czs[idx][i];
    tv[i] = (i == 0) ? 1.0 : tv[i - 1] * t;
  }

  Eigen::Vector3d pos;
  pos[0] = tv.dot(cx), pos[1] = tv.dot(cy), pos[2] = tv.dot(cz);
  return pos;
}

Eigen::Vector3d PolyTraj::evaluateVel(double t)
{
  // determine segment index
  int idx = 0;
  while (t > times[idx] + 1e-4 && idx < num_seg)
  {
    t -= times[idx];
    idx++;
  }

  if (idx == num_seg)
  {
    idx--;
    t = times[idx];
  }

  // evaluation
  int order = cxs[idx].size();
  Eigen::VectorXd vx(order - 1), vy(order - 1), vz(order - 1), tv(order - 1);
  for (int i = 0; i < order - 1; i++)
  {
    vx[i] = double(i + 1) * cxs[idx][i + 1];
    vy[i] = double(i + 1) * cys[idx][i + 1];
    vz[i] = double(i + 1) * czs[idx][i + 1];
    tv[i] = (i == 0) ? 1.0 : tv[i - 1] * t;
  }

  Eigen::Vector3d vel;
  vel[0] = tv.dot(vx), vel[1] = tv.dot(vy), vel[2] = tv.dot(vz);
  return vel;
}

Eigen::Vector3d PolyTraj::evaluateAcc(double t)
{
  // determine segment index
  int idx = 0;
  while (t > times[idx] + 1e-4 && idx < num_seg)
  {
    t -= times[idx];
    idx++;
  }

  if (idx == num_seg)
  {
    idx--;
    t = times[idx];
  }

  // evaluation
  int order = cxs[idx].size();
  Eigen::VectorXd ax(order - 2), ay(order - 2), az(order - 2), tv(order - 2);
  for (int i = 0; i < order - 2; i++)
  {
    ax[i] = double((i + 2) * (i + 1)) * cxs[idx][i + 2];
    ay[i] = double((i + 2) * (i + 1)) * cys[idx][i + 2];
    az[i] = double((i + 2) * (i + 1)) * czs[idx][i + 2];
    tv[i] = (i == 0) ? 1.0 : tv[i - 1] * t;
  }

  Eigen::Vector3d acc;
  acc[0] = tv.dot(ax), acc[1] = tv.dot(ay), acc[2] = tv.dot(az);
  return acc;
}

double PolyTraj::getTotalTIme()
{
  return total_time;
}

vector<Eigen::Vector3d> PolyTraj::getTraj()
{
  traj_vec3d.clear();
  double t = 0.0;

  while (t < total_time)
  {
    traj_vec3d.push_back(evaluatePos(t));
    t += 0.01;
  }

  return traj_vec3d;
}

double PolyTraj::getLength()
{
  length = 0.0;

  Eigen::Vector3d p_l = traj_vec3d[0], p_n;
  for (int i = 1; i < traj_vec3d.size(); i++)
  {
    p_n = traj_vec3d[i];
    length += (p_n - p_l).norm();
    p_l = p_n;
  }

  return length;
}

double PolyTraj::getMeanVel()
{
  return length / total_time;
}

#endif // _POLY_TRAJ_HPP_