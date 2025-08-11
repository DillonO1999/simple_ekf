#ifndef MY_HEADER_EARTH
#define MY_HEADER_EARTH

#include <vector>
#include <eigen3/Eigen/Dense>

double calc_gravity(Eigen::Matrix<double,3,1> p, double R_phi, double R_lam);

std::vector<double> calculate_radii(double pos0, double a, double e_sqr);

#endif