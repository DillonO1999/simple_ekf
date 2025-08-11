#include <iostream>
#include <cmath>
#include "helper_functions.hpp"
#include "dynamics_functions.hpp"
#include "earth_functions.hpp"

using namespace std;
using namespace Eigen;

Matrix<double,15,15> generate_F(double phi, double h, double Vn, double Ve, double Vd, Matrix<double,3,1> a_bar, Matrix<double,3,1> g_bar, double yaw, Matrix<double,3,3> A, double a_tau, double g_tau, double w_e) {

        // Schwartz model
        double a1 = 9.7803267715;
        double a2 = 0.0052790414;
        double a3 = 0.0000232718;
        double a4 = -3.0876910891e-6;
        double a5 = 4.3977311e-9;
        double a6 = 7.211e-13;

        // Useful constants
        const double Fe = 1 / 298.257223563; const double e_sqr = Fe*(2 - Fe);
        const double a = 6378137; const double e = sqrt(e_sqr); const double omega = 7.2921151467e-5;

        // propagation equations
        double R_phi = calculate_radii(phi, a, e_sqr)[0];
        double R_lam = calculate_radii(phi, a, e_sqr)[1];

        double dgdh = a4 + 2*a6*h + a5*pow(sin(phi),2);
        double dgdphi = a1*(2*a2*cos(phi)*sin(phi) + 4*a3*cos(phi)*pow(sin(phi),3)) + 2*a5*h*cos(phi)*sin(phi);

        Matrix<double,3,3> dp_dp;
        dp_dp << 0, 0, -Vn/pow(R_phi,2),
                (Ve*tan(phi))/(R_lam*cos(phi)), 0, -Ve/(pow(R_lam,2)*cos(phi)),
                0, 0, 0;

        Matrix<double,3,3> dp_dv;
        dp_dv << 1/R_phi, 0, 0,
                0, 1/(R_lam*cos(phi)), 0,
                0, 0, -1;

        Matrix<double,3,3> dv_dp;
        dv_dp << -Ve*(2*w_e*cos(phi)+Ve/(R_lam*pow((cos(phi)),2))), 0, (1/pow(R_lam,2))*(pow(Ve,2)*tan(phi))-(Vn*Vd)/(pow(R_phi,2)),
                2*w_e*(Vn*cos(phi)-Vd*sin(phi))+(Vn*Ve)/(R_lam*pow(cos(phi),2)), 0, (-Ve/pow(R_lam,2))*(Vn*tan(phi)+Vd),
                2*w_e*Ve*sin(phi)+dgdphi, 0, (1/pow(R_phi,2))*(pow(Vn,2))+(pow(Ve,2))/pow(R_lam,2)+dgdh;

        Matrix<double,3,3> dv_dv;
        dv_dv << Vd/R_phi, -2*(w_e*sin(phi)+(Ve/R_lam)*tan(phi)), Vn/R_phi,
                2*w_e*sin(phi)+(Ve/R_lam)*tan(phi), (1/R_lam)*(Vn*tan(phi)+Vd), 2*w_e*cos(phi)+(Ve/R_lam),
                -2*Vn/R_phi, -2*(w_e*cos(phi)+Ve/R_lam), 0;
        
        Matrix<double,3,3> dv_datt = -A.transpose()*skew(a_bar);

        Matrix<double,3,3> dv_dba = -A.transpose();

        Matrix<double,3,3> datt_dp;
        datt_dp << -w_e*sin(phi), 0, Ve/pow(R_lam,2),
                  0, 0, -Vn/pow(R_phi,2),
                  -w_e*cos(phi)-Ve/(R_lam*pow(cos(phi),2)), 0, Ve*tan(phi)/pow(R_lam,2);
        datt_dp = -A*datt_dp;

        Matrix<double,3,3> datt_dv;
        datt_dv << 0, 1/R_lam, 0,
                  -1/R_phi, 0, 0,
                  0, -tan(phi)/R_lam, 0;
        datt_dv = -A*datt_dv;

        Matrix<double,3,3> datt_datt = -skew(g_bar);
        
        Matrix<double,3,3> datt_dbg = -Matrix<double,3,3>::Identity();

        Matrix<double,15,15> F = Matrix<double,15,15>::Zero();

        for (int j = 0; j < 3; j++) {
                for (int k = 0; k < 3; k++) {
                        F(j,k) = dp_dp(j,k);
                        F(j,k+3) = dp_dv(j,k);
                        // F(j,k+6) = block3(j,k);
                        F(j+3,k) = dv_dp(j,k);
                        F(j+3,k+3) = dv_dv(j,k);
                        F(j+3,k+6) = dv_datt(j,k);
                        F(j+3,k+9) = dv_dba(j,k);
                        F(j+6,k) = datt_dp(j,k);
                        F(j+6,k+3) = datt_dv(j,k);
                        F(j+6,k+6) = datt_datt(j,k);
                        F(j+6,k+12) = datt_dbg(j,k);
                }
                F(j+9,j+9) = -1.0/a_tau;
                F(j+12,j+12) = -1.0/g_tau;
        }

        return F;
}

Matrix<double,15,12> generate_G(Matrix<double,3,3> A) {
    Matrix<double,15,12> G;
    G << Matrix<double,3,3>::Zero(), Matrix<double,3,3>::Zero(), Matrix<double,3,3>::Zero(), Matrix<double,3,3>::Zero(),
         -A.transpose(), Matrix<double,3,3>::Zero(), Matrix<double,3,3>::Zero(), Matrix<double,3,3>::Zero(),
         Matrix<double,3,3>::Zero(), -Matrix<double,3,3>::Identity(), Matrix<double,3,3>::Zero(), Matrix<double,3,3>::Zero(),
         Matrix<double,3,3>::Zero(), Matrix<double,3,3>::Zero(), Matrix<double,3,3>::Identity(), Matrix<double,3,3>::Zero(),
         Matrix<double,3,3>::Zero(), Matrix<double,3,3>::Zero(), Matrix<double,3,3>::Zero(), Matrix<double,3,3>::Identity();         

    return G;
}

Matrix<double,12,12> generate_Q(Matrix<double,3,1> a_sig, Matrix<double,3,1> g_sig, double abss_sigma, double gbss_sigma, double a_tau, double g_tau, Matrix<double,3,3> A) {
    Matrix<double,12,12> Q = Matrix<double,12,12>::Zero();
    
    for (int i = 0; i < 3; i++) {
        Q(i,i) = pow(a_sig(i),2);
    }
    for (int i = 3; i < 6; i++) {
        Q(i,i) = pow(g_sig(i-3),2);
    }
    for (int i = 6; i < 9; i++) {
        Q(i,i) = pow(abss_sigma*sqrt(2/a_tau),2);
    }
    for (int i = 9; i < 12; i++) {
        Q(i,i) = pow(gbss_sigma*sqrt(2/g_tau),2);
    }

    return Q;
}