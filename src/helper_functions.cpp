#include "helper_functions.hpp"

using namespace std;
using namespace Eigen;

// Rotation Matrix
Matrix<double,2,2> C(double theta) {

    Matrix<double,2,2> C;
    C << cos(theta), -sin(theta),
         sin(theta), cos(theta);

    return C;
}

// Wrap radian angles to -pi < angle < pi
double wrapToPi(double angle) {
    if (angle > M_PI) {
        angle -= 2.0 * M_PI;
    } else if (angle < -M_PI) {
        angle += 2.0 * M_PI;
    }
    return angle;
}

// Create 3x3 skew matrix from 3x1 vector
Matrix<double,3,3> skew(Matrix<double,3,1> vec) {
    Matrix<double,3,3> skew_mat;
    skew_mat << 0, -vec(2), vec(1),
                vec(2), 0, -vec(0),
                -vec(1), vec(0), 0;
    return skew_mat;
}

// Calculate norm/magnitude of quaternion
double normq(Matrix<double,4,1> q) {
    double mag;
    mag = sqrt(pow(q(0),2) + pow(q(1),2) + pow(q(2),2) + pow(q(3),2));
    
    return mag;
}

// Convert quaternion to 3x3 rotation matrix
Matrix<double,3,3> q2rot(Matrix<double,4,1> q) {
    Matrix<double,3,1> q_vec;
    q_vec << q(0), q(1), q(2);

    Matrix<double,3,3> qrot;
    qrot = Matrix<double,3,3>::Identity() - 2*q(3)*skew(q_vec) + 2*skew(q_vec)*skew(q_vec);

    return qrot;         
}

// Convert euler angles to quaternion
Matrix<double,4,1> eul2q(Matrix<double,3,1> eul) {
    double R = eul(0); double P = eul(1); double Y = eul(2);

    Matrix<double,4,1> quat;
    quat << sin(R/2)*cos(P/2)*cos(Y/2) - cos(R/2)*sin(P/2)*sin(Y/2),
            cos(R/2)*sin(P/2)*cos(Y/2) + sin(R/2)*cos(P/2)*sin(Y/2),
            cos(R/2)*cos(P/2)*sin(Y/2) - sin(R/2)*sin(P/2)*cos(Y/2),
            cos(R/2)*cos(P/2)*cos(Y/2) + sin(R/2)*sin(P/2)*sin(Y/2);

    return quat;
}

// Convert quaternion to euler angles
Matrix<double,3,1> q2eul(Matrix<double,4,1> q) {
    double qw = q(3); double qx = q(0); double qy = q(1); double qz = q(2);

    Matrix<double,3,1> eul;
    eul << atan2(2*(qw*qx + qy*qz), 1 - 2*(pow(qx,2) + pow(qy,2))),
           asin(std::min(std::max(2 * (qw * qy - qx * qz), -1.0), 1.0)),
           atan2(2*(qw*qz + qx*qy), 1 - 2*(pow(qy,2) + pow(qz,2))); 
    
    return eul;
}

// Multiply two quaternions
Matrix<double,4,1> qmult(Matrix<double,4,1> q1, Matrix<double,4,1> q2) {
    double qs = q1(3);
    Matrix<double,3,1> qv; qv << q1(0), q1(1), q1(2);

    Matrix<double,3,3> q_mat = qs*Matrix<double,3,3>::Identity()-skew(qv);
    Matrix<double,4,4> q_mat1;
    for (int i = 0; i < 3; i++) {
        q_mat1(i,3) = qv(i);
        q_mat1(3,i) = -qv(i);
        for (int j = 0; j < 3; j++) {
            q_mat1(i,j) = q_mat(i,j);
        }
    }
    q_mat1(3,3) = qs;
                    
    Matrix<double,4,1> q_out = q_mat1*q2;

    // Matrix<double,3,1> qv1(3,1); qv1 << q1(0), q1(1), q1(2);
    // Matrix<double,3,1> qv2(3,1); qv2 << q2(0), q2(1), q2(2);

    // Matrix<double,3,1> q_vec = q1(3)*qv2 + q2(3)*qv1 + crossProduct(qv1, qv2);
    // double q_scalar = q1(3)*q2(3) - dotProduct(qv1, qv2);

    // Matrix<double,4,1> q_out;
    // q_out << q_vec, q_scalar;

    return q_out;
}
