#ifndef DYNAMICS
#define DYNAMICS

#include <vector>
#include <eigen3/Eigen/Dense>

Eigen::Matrix<double,15,15> generate_F(double phi, double h, double Vn, double Ve, double Vd, Eigen::Matrix<double,3,1> a_bar, Eigen::Matrix<double,3,1> g_bar, double yaw, Eigen::Matrix<double,3,3> A, double a_tau, double g_tau, double w_e);

Eigen::Matrix<double,15,12> generate_G(Eigen::Matrix<double,3,3> A);

Eigen::Matrix<double,12,12> generate_Q(Eigen::Matrix<double,3,1> a_sig, Eigen::Matrix<double,3,1> g_sig, double abss_sigma, double gbss_sigma, double a_tau, double g_tau, Eigen::Matrix<double,3,3> A);

#endif