#include <iostream>
#include <lcm/lcm-cpp.hpp>
#include <thread>
#include <chrono>
#include <iomanip>
#include "helper_functions.hpp"

#include "msgs/imu.hpp"

using namespace std;
using namespace Eigen;

int main(int argc, char ** argv) {
    lcm::LCM lcm;

    if(!lcm.good()) {
        return 1;
    }

    msgs::imu imu_msg;

    for (int i = 0; i < 1000; i++) {
        auto now = std::chrono::system_clock::now();
        auto microseconds = std::chrono::duration_cast<std::chrono::microseconds>(now.time_since_epoch());
        double microseconds_double = static_cast<double>(microseconds.count());

        imu_msg.timestamp_valid = microseconds_double*1e-6;

        lcm.publish("imu", &imu_msg);
        cout << setprecision(15) << imu_msg.timestamp_valid << endl;

        // Sleep for 1 second
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    while(0 == lcm.handle());

    return 0;
}