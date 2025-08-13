# MPU9250 IMU Data Reader for Raspberry Pi 5
# This script reads accelerometer, gyroscope, magnetometer, and temperature data
# from an MPU9250 sensor via the I2C bus.

# IMPORTANT: Before running, you must enable I2C on your Raspberry Pi.
# You can do this using `sudo raspi-config` -> Interface Options -> I2C.
# You may also need to install the smbus2 library: `pip install smbus2`

import smbus2
import time
import struct
import os
import sys
import lcm
import time

# Get the current working directory (CWD)
current_path = os.getcwd() 
sys.path.insert(0, current_path + "/build/msgs")

from msgs import imu

# The Raspberry Pi 5 has one I2C bus, usually bus 1.
I2C_BUS = 1
bus = smbus2.SMBus(I2C_BUS)

# MPU9250 I2C address is 0x68
MPU9250_ADDRESS = 0x68

# AK8963 (Magnetometer) I2C address is 0x0C
AK8963_ADDRESS = 0x0C

# MPU9250 Register Addresses
# Configuration Registers
MPU_CONFIG = 0x1A
MPU_GYRO_CONFIG = 0x1B
MPU_ACCEL_CONFIG = 0x1C
MPU_INT_PIN_CFG = 0x37
MPU_PWR_MGMT_1 = 0x6B
MPU_WHO_AM_I = 0x75

# Data Registers
MPU_ACCEL_XOUT_H = 0x3B
MPU_GYRO_XOUT_H = 0x43
MPU_TEMP_OUT_H = 0x41

# AK8963 Register Addresses
AK_CNTL = 0x0A
AK_ASAX = 0x10 # Mag Sensitivity Adjustment X
AK_ST1 = 0x02 # Status 1 register
AK_XOUT_L = 0x03 # Magnetometer data registers
AK_YOUT_L = 0x05
AK_ZOUT_L = 0x07

# Sensitivity scale factors (adjust these if you change the config registers)
# The default values were for a full-scale range of +/-2g and +/-250dps.
# These new values are for a larger range of +/-8g and +/-2000dps.
# The magnetometer scale is not changed as it is fixed at +/-4912uT.
ACCEL_SCALE = 4096.0 # For +/-8g
GYRO_SCALE = 16.4    # For +/-2000 dps
MAG_SCALE = 4912.0   # For +/-4912 uT

#--------------------------------------------------------------------------------
# Helper Functions
#--------------------------------------------------------------------------------

def read_raw_data(address, reg):
    """
    Reads two bytes of data from a specified register address.
    Returns the combined signed 16-bit value.
    """
    high = bus.read_byte_data(address, reg)
    low = bus.read_byte_data(address, reg + 1)
    value = (high << 8) | low
    if value > 32767:
        value -= 65536
    return value

def read_mag_data(address, reg):
    """
    Reads two bytes of data from the magnetometer, which requires a specific
    read order. Returns the combined signed 16-bit value.
    """
    low = bus.read_byte_data(address, reg)
    high = bus.read_byte_data(address, reg + 1)
    value = (high << 8) | low
    if value > 32767:
        value -= 65536
    return value

#--------------------------------------------------------------------------------
# Initialization Functions
#--------------------------------------------------------------------------------

def init_mpu9250():
    """
    Initializes the MPU9250 and its internal magnetometer.
    """
    # print("Initializing MPU9250...")

    # Wake up the MPU9250
    bus.write_byte_data(MPU9250_ADDRESS, MPU_PWR_MGMT_1, 0x00)
    time.sleep(0.1)

    # Set up bypass mode for the magnetometer
    bus.write_byte_data(MPU9250_ADDRESS, MPU_INT_PIN_CFG, 0x22)
    time.sleep(0.1)

    # *** NEW: Configure accelerometer full-scale range to +/- 8g (0x10) ***
    # Other options: 0x00 (+/- 2g), 0x08 (+/- 4g), 0x18 (+/- 16g)
    bus.write_byte_data(MPU9250_ADDRESS, MPU_ACCEL_CONFIG, 0x10)
    
    # *** NEW: Configure gyroscope full-scale range to +/- 2000 dps (0x18) ***
    # Other options: 0x00 (+/- 250 dps), 0x08 (+/- 500 dps), 0x10 (+/- 1000 dps)
    bus.write_byte_data(MPU9250_ADDRESS, MPU_GYRO_CONFIG, 0x18)

    # Check WHO_AM_I register to confirm connection
    who_am_i = bus.read_byte_data(MPU9250_ADDRESS, MPU_WHO_AM_I)
    if who_am_i != 0x71:
        print(f"MPU9250 not found! WHO_AM_I register returned 0x{who_am_i:02x}, expected 0x71.")
        return False
    # print("MPU9250 connection successful.")
    return True

def init_ak8963():
    """
    Initializes the AK8963 magnetometer.
    """
    # print("Initializing AK8963 magnetometer...")
    
    # Power down the magnetometer
    bus.write_byte_data(AK8963_ADDRESS, AK_CNTL, 0x00)
    time.sleep(0.1)

    # Set continuous measurement mode 2 (100 Hz)
    bus.write_byte_data(AK8963_ADDRESS, AK_CNTL, 0x16)
    time.sleep(0.1)

#--------------------------------------------------------------------------------
# Main Loop
#--------------------------------------------------------------------------------

def main():
    if not init_mpu9250():
        return
    init_ak8963()
    
    # print("MPU9250 and AK8963 initialized. Reading data...")
    # print("-" * 40)

    lc = lcm.LCM("tcpq://localhost:7700")

    try:
        while True:
            # Read Accelerometer data
            accel_x = read_raw_data(MPU9250_ADDRESS, MPU_ACCEL_XOUT_H) / ACCEL_SCALE
            accel_y = read_raw_data(MPU9250_ADDRESS, MPU_ACCEL_XOUT_H + 2) / ACCEL_SCALE
            accel_z = read_raw_data(MPU9250_ADDRESS, MPU_ACCEL_XOUT_H + 4) / ACCEL_SCALE

            # Read Gyroscope data
            gyro_x = read_raw_data(MPU9250_ADDRESS, MPU_GYRO_XOUT_H) / GYRO_SCALE
            gyro_y = read_raw_data(MPU9250_ADDRESS, MPU_GYRO_XOUT_H + 2) / GYRO_SCALE
            gyro_z = read_raw_data(MPU9250_ADDRESS, MPU_GYRO_XOUT_H + 4) / GYRO_SCALE
            
            # Read Magnetometer data
            # Check for data ready bit
            status = bus.read_byte_data(AK8963_ADDRESS, AK_ST1)
            if status & 0x01:
                mag_x = read_mag_data(AK8963_ADDRESS, AK_XOUT_L) / MAG_SCALE
                mag_y = read_mag_data(AK8963_ADDRESS, AK_YOUT_L) / MAG_SCALE
                mag_z = read_mag_data(AK8963_ADDRESS, AK_ZOUT_L) / MAG_SCALE

                # Read ST2 register to reset data read signal
                bus.read_byte_data(AK8963_ADDRESS, 0x09)
            
            # Read Temperature data
            temp_raw = read_raw_data(MPU9250_ADDRESS, MPU_TEMP_OUT_H)
            temperature_c = (temp_raw / 333.87) + 21.0

            # Get the current Unix timestamp
            timestamp = time.time()
            parts = str(accel_x).split('.')
            decimal_part_str = parts[1]

            msg = imu()
            msg.header.timestamp_valid.sec = int(timestamp)
            msg.header.timestamp_valid.nsec = int(decimal_part_str[:9])
            msg.delta_v[0] = accel_x
            msg.delta_v[1] = accel_y
            msg.delta_v[2] = accel_z
            msg.delta_theta[0] = gyro_x
            msg.delta_theta[1] = gyro_y
            msg.delta_theta[2] = gyro_z

            lc.publish("aspn/vectorNav/rawimu", msg.encode())

            
            # print(f"Accel: X={accel_x:7.2f} g, Y={accel_y:7.2f} g, Z={accel_z:7.2f} g")
            # print(f"Gyro:  X={gyro_x:7.2f} dps, Y={gyro_y:7.2f} dps, Z={gyro_z:7.2f} dps")
            # print(f"Mag:   X={mag_x:7.2f} uT, Y={mag_y:7.2f} uT, Z={mag_z:7.2f} uT")
            # print(f"Temp:  {temperature_c:7.2f} C")
            # print("-" * 40)
            
            # time.sleep(0.005)

    except KeyboardInterrupt:
        print("Script terminated by user.")
    finally:
        bus.close()

if __name__ == "__main__":
    main()
