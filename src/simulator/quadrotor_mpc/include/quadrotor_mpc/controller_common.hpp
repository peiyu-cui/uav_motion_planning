#ifndef CONTROLLER_COMMON
#define CONTROLLER_COMMON

#include <Eigen/Eigen>
#include <cstdlib>
#include <fstream>

static constexpr unsigned int kSingleStateDim = 4;
static constexpr unsigned int kStateDim = 12;
static constexpr unsigned int kControlDim = 4;

typedef Eigen::Matrix<double, kStateDim, 1> StateVector;
typedef Eigen::Matrix<double, kControlDim, 1> ControlVector;

static const Eigen::IOFormat CSVFormat(Eigen::StreamPrecision, Eigen::DontAlignCols, ", ", "\n");
static std::fstream logger(std::string(getenv("HOME")) + "/logger.csv", std::ofstream::out | std::ofstream::trunc);

#endif