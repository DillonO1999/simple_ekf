#ifndef PROPAGATE
#define PROPAGATE

#include <eigen3/Eigen/Dense>

void propagate(Eigen::Matrix<double,3,1>& position, Eigen::Matrix<double,3,1>& velocity, Eigen::Matrix<double,3,1>& attitude, Eigen::Matrix<double,4,1>& q, Eigen::Matrix<double,15,15>& covariance, const Eigen::Matrix<double,3,1>& accel, const Eigen::Matrix<double,3,1>& gyro, const Eigen::Matrix<double,15,15>& F, const Eigen::Matrix<double,15,12>& G, const Eigen::Matrix<double,12,12>& Q, const Eigen::Matrix<double,3,3>& A, const double& dt);

#endif