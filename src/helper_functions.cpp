#include "helper_functions.hpp"

using namespace std;
using namespace Eigen;

Matrix<double,2,2> C(double theta) {

    Matrix<double,2,2> C;
    C << cos(theta), -sin(theta),
         sin(theta), cos(theta);

    return C;
}