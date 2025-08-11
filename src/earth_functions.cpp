#include <iostream>
#include <cmath>
#include "earth_functions.hpp"

using namespace std;
using namespace Eigen;

double calc_gravity(Matrix<double,3,1> p, double R_phi, double R_lam) {
    double g;
    if (p(2) > 0) {
        g = (9.780318 * (1 + 5.3024e-3 * pow(sin(p(0)),2) - 5.9e-6 * pow((sin(2 * p(0))), 2))) / pow((1 + p(2) / sqrt(R_phi * R_lam)),2);
    } else {
        g = (9.780318 * (1 + 5.3024e-3 * pow(sin(p(0)),2) - 5.9e-6 * pow((sin(2 * p(0))), 2))) * (1 + p(2) / sqrt(R_phi * R_lam));
    }

    return g;
}

vector<double> calculate_radii(double pos0, double a, double e_sqr) {

    double R_lat = (a*(1-e_sqr))/pow((1-e_sqr*pow((sin(pos0)),2)), 1.5);
    double R_lon = a/pow((1-e_sqr*pow((sin(pos0)),2)), 0.5);

    return {R_lat, R_lon};
}