#ifndef HELPER_FUNCTIONS
#define HELPER_FUNCTIONS

#include <eigen3/Eigen/Dense>

// Rotation Matrix
Eigen::Matrix<double,2,2> C(double theta);

// Wrap radian angles to -pi < angle < pi
double wrapToPi(double angle);

// Create 3x3 skew matrix from 3x1 vector
Eigen::Matrix<double,3,3> skew(Eigen::Matrix<double,3,1> vec);

// Calculate norm/magnitude of quaternion
double normq(Eigen::Matrix<double,4,1> q);

// Convert quaternion to 3x3 rotation matrix
Eigen::Matrix<double,3,3> q2rot(Eigen::Matrix<double,4,1> q);

// Convert euler angles to quaternion
Eigen::Matrix<double,4,1> eul2q(Eigen::Matrix<double,3,1> eul);

// Convert quaternion to euler angles
Eigen::Matrix<double,3,1> q2eul(Eigen::Matrix<double,4,1> q);

// Multiply two quaternions
Eigen::Matrix<double,4,1> qmult(Eigen::Matrix<double,4,1> q1, Eigen::Matrix<double,4,1> q2);

#endif