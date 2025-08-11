#include <iostream>
#include <lcm/lcm-cpp.hpp>
#include "yaml-cpp/yaml.h"
#include <thread>
#include <chrono>
#include <iomanip>
#include <deque>
#include "helper_functions.hpp"
#include "propagate.hpp"
#include "dynamics_functions.hpp"
#include "earth_functions.hpp"

#include "msgs/imu.hpp"
#include "msgs/positionvelocityattitude.hpp"
#include "msgs/geodeticposition3d.hpp"

using namespace std;
using namespace Eigen;

class ekf 
{
    public:
        ekf() : lcm_tcpq("tcpq://localhost:7700")
        {
            YAML::Node config = YAML::LoadFile("configs/config.yaml");

            pva_channel = config["pva_chan"].as<string>();
            gps_channel = config["gps_chan"].as<string>();
            imu_channel = config["imu_chan"].as<string>();

            YAML::Node imu2platform_vec = config["imu2platform"];
            YAML::Node accel_sigma_vec = config["accel_sigma"];
            YAML::Node gyro_sigma_vec = config["gyro_sigma"];
            for (int i = 0; i < 3; i++) {
                for (int j = 0; j < 3; j++) {
                    imu2platform(i,j) = imu2platform_vec[i*3 + j].as<double>();
                }
                accel_sigma(i) = accel_sigma_vec[i].as<double>(); 
                gyro_sigma(i) = gyro_sigma_vec[i].as<double>(); 
            }
            accel_tau = config["accel_tau"].as<double>();
            gyro_tau = config["gyro_tau"].as<double>();
            accel_bias_initial_sigma = config["accel_bias_initial_sigma"].as<double>();
            accel_bias_steady_state_sigma = config["accel_bias_steady_state_sigma"].as<double>();
            gyro_bias_initial_sigma = config["gyro_bias_initial_sigma"].as<double>();
            gyro_bias_steady_state_sigma = config["gyro_bias_steady_state_sigma"].as<double>();
        }

        string pva_channel;
        string gps_channel;
        string imu_channel;

        int pva_count = 1;
        void handlePVA(const lcm::ReceiveBuffer* rbuf, const std::string& chan, const msgs::positionvelocityattitude* msg)
        {
            // initialize filter
            int init = 1;
            if (pva_count == init) {
                // setting initial mekf values to the first pva values
                this->pva_timestamp_valid = msg->header.timestamp_valid.sec + (msg->header.timestamp_valid.nsec)*1e-9;
                this->position << msg->position.latitude, msg->position.longitude, msg->position.altitude; 
                this->velocity << msg->velocity[0], msg->velocity[1], msg->velocity[2];
                this->attitude << wrapToPi(msg->attitude[0]), wrapToPi(msg->attitude[1]), wrapToPi(msg->attitude[2]);
                this->q = eul2q(this->attitude);
                this->q = this->q/normq(this->q);
                this->accel_bias.setZero();
                this->gyro_bias.setZero();

                P.setZero();
                for (int i = 0; i < 9; i++) {
                    for (int j = 0; j < 9; j++) {
                        P(i,j) = msg->covariance[i][j];
                    }
                }
                double R_lat = calculate_radii(position(0), a, e_sqr)[0];
                double R_lon = calculate_radii(position(0), a, e_sqr)[1];
                double lat_factor = 1/(R_lat+position(2));
                double lon_factor = 1/((R_lon+position(2))*cos(position(0)));
                P(0,0) = P(0,0)*pow(lat_factor,2);
                P(1,1) = P(1,1)*pow(lon_factor,2);
                for (int i = 9; i < 12; i++) {
                    P(i,i) = pow(accel_bias_initial_sigma,2);
                    P(i+3,i+3) = pow(gyro_bias_initial_sigma,2);
                }

                // publishing initial mekf estimates
                mekf.header.timestamp_valid.sec = msg->header.timestamp_valid.sec;
                mekf.header.timestamp_valid.nsec = msg->header.timestamp_valid.nsec;
                mekf.position.latitude = position(0);
                mekf.position.longitude = position(1);
                mekf.position.altitude = position(2);
                for (int j = 0; j < 3; j++) {
                    mekf.velocity[j] = velocity(j);
                    mekf.attitude[j] = attitude(j);
                }
                for (int i = 0; i < 9; i++) {
                    for (int j = 0; j < 9; j++) {
                        mekf.covariance[i][j] = P(i,j);
                    }
                }

                lcm_tcpq.publish("mekf_solution", &mekf);
            }

            pva_count += 1;
        }

        void handleIMU(const lcm::ReceiveBuffer* rbuf, const std::string& chan, const msgs::imu* msg)
        {   
            imu_buffer.push_back(*msg);
            if (pva_timestamp_valid) {
                if (imu_buffer.size() > 1) {
                    this->imu_timestamp_valid = imu_buffer.back().header.timestamp_valid.sec + (imu_buffer.back().header.timestamp_valid.nsec)*1e-9;
                    this->imu_dt = (imu_buffer.back().header.timestamp_valid.sec + (imu_buffer.back().header.timestamp_valid.nsec)*1e-9) - (imu_buffer.front().header.timestamp_valid.sec + (imu_buffer.front().header.timestamp_valid.nsec)*1e-9);
                    for (int i = 0; i < 3; i++) {
                        this->accel(i) = (imu_buffer.front().delta_v[i])/imu_dt;
                        this->gyro(i) = (imu_buffer.front().delta_theta[i])/imu_dt;
                    }
                    this->accel = imu2platform*accel;
                    this->gyro = imu2platform*gyro;

                    // propagation equations
                    double R_phi = calculate_radii(position(0), a, e_sqr)[0];
                    double R_lam = calculate_radii(position(0), a, e_sqr)[1];

                    const double omega = 7.2921151467e-5;
                    Matrix<double,3,1> Cor; Cor << cos(position(0)), 0, -sin(position(0));
                    Matrix<double,3,1> Vels; Vels << velocity(1)/(R_lam+position(2)), -velocity(0)/(R_phi+position(2)), -velocity(1)*tan(position(0))/(R_lam+position(2));

                    Matrix<double,3,3> A = q2rot(q);
                    Matrix<double,3,1> a_bar = A.transpose()*(accel - accel_bias);
                    Matrix<double,3,1> g_bar =  (gyro - gyro_bias) - A*(omega*Cor + Vels);

                    Matrix<double,15,15> F = generate_F(position(0), position(2), velocity(0), velocity(1), velocity(2), a_bar, g_bar, attitude(2), A, accel_tau, gyro_tau, omega);
                    Matrix<double,15,12> G = generate_G(A);
                    Matrix<double,12,12> Q = generate_Q(accel_sigma, gyro_sigma, accel_bias_steady_state_sigma, gyro_bias_steady_state_sigma, accel_tau, gyro_tau, A);
                    
                    propagate(position, velocity, attitude, q, P, a_bar, g_bar, F, G, Q, A, imu_dt);

                    mekf.header.timestamp_valid.sec = imu_buffer.back().header.timestamp_valid.sec;
                    mekf.header.timestamp_valid.nsec = imu_buffer.back().header.timestamp_valid.nsec;
                    mekf.position.latitude = position(0);
                    mekf.position.longitude = position(1);
                    mekf.position.altitude = position(2);
                    for (int i = 0; i < 3; i++) {
                        mekf.velocity[i] = velocity(i);
                        mekf.attitude[i] = attitude(i);
                    }
                    for (int i = 0; i < 9; i++) {
                        for (int j = 0; j < 9; j++) {
                            mekf.covariance[i][j] = P(i,j);
                        }
                    }

                    lcm_tcpq.publish("mekf_solution", &mekf);
                    imu_buffer.erase(imu_buffer.begin());
                }
            }
        }

        void handleGPS(const lcm::ReceiveBuffer* rbuf, const std::string& chan, const msgs::geodeticposition3d* msg)
        {
            this->gps_timestamp_valid = msg->header.timestamp_valid.sec + (msg->header.timestamp_valid.nsec)*1e-9;
            this->gps_latitude = msg->position.latitude;
            this->gps_longitude = msg->position.longitude;
            this->gps_altitude = msg->position.altitude;
            for (int i = 0; i < 3; i++) {
                for (int j = 0; j < 3; j++) {
                    this->gps_covariance(i,j) = msg->covariance[i][j];
                }
            }
        }

        lcm::LCM lcm_tcpq;

    private:
        // config variables
        Matrix<double,3,3> imu2platform;
        Matrix<double,3,1> accel_sigma;
        Matrix<double,3,1> gyro_sigma;
        double accel_tau;
        double gyro_tau;
        double accel_bias_initial_sigma;
        double accel_bias_steady_state_sigma;
        double gyro_bias_initial_sigma;
        double gyro_bias_steady_state_sigma;

        // imu variables
        vector<msgs::imu> imu_buffer;
        double imu_dt;
        double imu_timestamp_valid;
        Matrix<double,3,1> accel;
        Matrix<double,3,1> gyro;

        // gps variables
        double gps_timestamp_valid;
        double gps_latitude;
        double gps_longitude;
        double gps_altitude;
        Eigen::Matrix<double,3,3> gps_covariance;

        // pva variables
        double pva_timestamp_valid = 0;
        Matrix<double,3,1> position; Matrix<double,3,1> velocity; Matrix<double,4,1> q; Matrix<double,3,1> attitude; 
        Matrix<double,3,1> accel_bias; Matrix<double,3,1> gyro_bias; 
        Matrix<double,15,15> P;

        // mekf variables
        msgs::positionvelocityattitude mekf;

        // Useful constants
        const double Fe = 1 / 298.257223563; const double e_sqr = Fe*(2 - Fe);
        const double a = 6378137; const double e = sqrt(e_sqr); const double omega = 7.2921151467e-5;

};

int main(int argc, char ** argv) {

    ekf obj;

    if(!obj.lcm_tcpq.good()) {
        return 1;
    }

    obj.lcm_tcpq.subscribe(obj.pva_channel, &ekf::handlePVA, &obj);
    obj.lcm_tcpq.subscribe(obj.imu_channel, &ekf::handleIMU, &obj);
    obj.lcm_tcpq.subscribe(obj.gps_channel, &ekf::handleGPS, &obj);


    while(0 == obj.lcm_tcpq.handle());

    return 0;
}