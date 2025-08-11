#include "propagate.hpp"
#include "earth_functions.hpp"
#include "helper_functions.hpp"
#include <iostream>

using namespace std;
using namespace Eigen;

void propagate(Matrix<double,3,1>& position, Matrix<double,3,1>& velocity, Matrix<double,3,1>& attitude, Matrix<double,4,1>& q, Matrix<double,15,15>& covariance, const Matrix<double,3,1>& accel, const Matrix<double,3,1>& gyro, const Matrix<double,15,15>& F, const Matrix<double,15,12>& G, const Matrix<double,12,12>& Q, const Matrix<double,3,3>& A, const double& dt) {
    // Useful constants
    const double Fe = 1 / 298.257223563; const double e_sqr = Fe*(2 - Fe);
    const double a = 6378137; const double e = sqrt(e_sqr); const double omega = 7.2921151467e-5;
    
    // propagation equations
    double R_phi = calculate_radii(position(0), a, e_sqr)[0];
    double R_lam = calculate_radii(position(0), a, e_sqr)[1];

    // gravity
    double g;
    g = calc_gravity(position, R_phi, R_lam); // Titterton
    Matrix<double,3,1> gravity;
    gravity << 0, 0, g;

    double phi_dot = (velocity(0)/(R_phi+position(2)));
    double lam_dot = (velocity(1)/((R_lam+position(2))*cos(position(0))));
    double h_dot = (-velocity(2));

    Matrix<double,3,1> Cor; Cor << cos(position(0)), 0, -sin(position(0));
    Matrix<double,3,1> Vels; Vels << velocity(1)/(R_lam+position(2)), -velocity(0)/(R_phi+position(2)), -velocity(1)*tan(position(0))/(R_lam+position(2));

    Matrix<double,3,1> pos_dot, vel_dot;
    pos_dot << phi_dot, lam_dot, h_dot;
    vel_dot = accel - skew(2*omega*Cor + Vels)*velocity + gravity;

    position = position + pos_dot*dt;
    
    velocity = velocity + vel_dot*dt;

    Matrix<double,4,3> q_mat; Matrix<double,3,1> q_vec;
    q_vec << q(0), q(1), q(2);
    q_mat << q(3)*Matrix<double,3,3>::Identity() + skew(q_vec),
            -q_vec.transpose();

    // Euler quaternion propagation
    Matrix<double,4,1> q_dot = 0.5*q_mat*gyro;
    q = q + q_dot*dt;
    q = q/normq(q);

    attitude = q2eul(q);

    Matrix<double,15,15> Phi = Matrix<double,15,15>::Identity() + F*dt;
    Matrix<double,15,15> Qd = (G*Q*G.transpose())*dt;
    covariance = Phi*covariance*Phi.transpose() + Qd;
}
