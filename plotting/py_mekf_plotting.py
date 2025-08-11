#!/usr/bin/env python3

import os
import sys
import lcm
import yaml
import numpy as np
from scipy.interpolate import interp1d
from scipy.spatial.transform import Rotation
from scipy.spatial.transform import Slerp
import signal
import matplotlib.pyplot as plt
import math

signal.signal(signal.SIGINT, signal.SIG_DFL)

# Get the path to the msgs directory
parent_dir = os.path.abspath(os.path.join(os.getcwd(), "../build/msgs/"))
# Add the parent directory to sys.path
sys.path.append(parent_dir)

import msgs.positionvelocityattitude as pva
import msgs.geodeticposition3d as gps
import msgs.imu as imu


def wrapToPi(angle):

  return (angle + np.pi) % (2 * np.pi) - np.pi

def r2d(angle):

    return (180/np.pi)*angle


def wrap_angle_to_180(angles):
    """
    Wraps an array of angles to the range [-180, 180) degrees.

    Args:
        angles (numpy.ndarray): An array of angles in degrees.

    Returns:
        numpy.ndarray: An array of angles wrapped to the range [-180, 180) degrees.
    """
    return (angles + 180) % 360 - 180

def interpolate_list_to_match_size(list1, timestamps1, timestamps2):
    if not list1 or not timestamps1 or not timestamps2:
        return []
    if len(list1) != len(timestamps1):
        raise ValueError("The lengths of list1 and timestamps1 must be equal.")

    # Convert lists to numpy arrays for interpolation
    x = np.array(timestamps1)
    y = np.array(list1)
    x_new = np.array(timestamps2)

    # Sort the timestamps to ensure correct interpolation
    sort_indices = np.argsort(x)
    x_sorted = x[sort_indices]
    y_sorted = y[sort_indices]

    try:
        # Create an interpolation function (linear interpolation)
        f = interp1d(x_sorted, y_sorted, kind='linear', fill_value="extrapolate")

        # Interpolate the values at the new timestamps
        interpolated_list = f(x_new).tolist()
        return interpolated_list
    except ValueError as e:
        print(f"Error during interpolation: {e}")
        return []


def interpolate_euler_angles(times, angles, new_times, rotation_sequence='XYZ', degrees=True):
    # Convert Euler angles to rotations
    rotations = Rotation.from_euler(rotation_sequence, angles, degrees=degrees)

    # Create a Slerp object
    slerp = Slerp(times, rotations)

    # Interpolate rotations
    interp_rots = slerp(new_times)

    # Convert interpolated rotations back to Euler angles
    interp_angles = interp_rots.as_euler(rotation_sequence, degrees=degrees)

    return interp_angles

def wgs84_radii(phi):
    a = 6378137; 
    F = 1 / 298.257223563
    R = a
    e = (F * (2 - F))**(1/2)
    R_phi = (a*(1-e**2))/(1-e**2*(np.sin(phi))**2)**(3/2)
    R_lam = a/(1-e**2*(np.sin(phi))**2)**(1/2)

    return R_phi,R_lam

# Read Yaml config
try:
    with open('../configs/config.yaml', 'r') as file:
        config = yaml.safe_load(file)

        pva_channel = config['pva_chan']
        gps_channel = config['gps_chan']

except FileNotFoundError:
    print("Error: The 'config.yaml' file was not found.")
except yaml.YAMLError as exc:
    print(f"Error parsing YAML file: {exc}")
    
# Setting up LCM to loop through the output log
in_file = f"../flightData/output_log.lcmlog"
in_log = lcm.EventLog(in_file, mode='r')

# Initializing list variables
ekf_time, ekf_time_pa, pva_time, gps_time, bias_time, delta_pose_time, delta_pose_time_pa, delta_pose_time_kf, altitude_time, residual_time_gps, residual_time_LA, residual_time_alt, rf_time = [[] for i in range(13)]
N_truth, E_truth, D_truth, lat_truth, lon_truth, alt_truth, Vn_truth, Ve_truth, Vd_truth, An_truth, Ae_truth, Ad_truth = [[] for i in range(12)]
N, E, D, lat, lon, alt, Vn, Ve, Vd, An, Ae, Ad, eul = [[] for i in range(13)]
N_pa, E_pa, D_pa, lat_pa, lon_pa, alt_pa, Vn_pa, Ve_pa, Vd_pa, An_pa, Ae_pa, Ad_pa = [[] for i in range(12)]
gps_lat, gps_lon, gps_alt = [[] for i in range(3)]
unc_N, unc_E, unc_D, unc_lat, unc_lon, unc_alt, unc_Vn, unc_Ve, unc_Vd, unc_An, unc_Ae, unc_Ad = [[] for i in range(12)]
nunc_N, nunc_E, nunc_D, nunc_lat, nunc_lon, nunc_alt, nunc_Vn, nunc_Ve, nunc_Vd, nunc_An, nunc_Ae, nunc_Ad = [[] for i in range(12)]
unc_N_pa, unc_E_pa, unc_D_pa, unc_lat_pa, unc_lon_pa, unc_alt_pa, unc_Vn_pa, unc_Ve_pa, unc_Vd_pa, unc_An_pa, unc_Ae_pa, unc_Ad_pa = [[] for i in range(12)]
nunc_N_pa, nunc_E_pa, nunc_D_pa, nunc_lat_pa, nunc_lon_pa, nunc_alt_pa, nunc_Vn_pa, nunc_Ve_pa, nunc_Vd_pa, nunc_An_pa, nunc_Ae_pa, nunc_Ad_pa = [[] for i in range(12)]
ax_bias, ay_bias, az_bias, gx_bias, gy_bias, gz_bias = [[] for i in range(6)]
unc_abx, unc_aby, unc_abz, unc_gbx, unc_gby, unc_gbz = [[] for i in range(6)]
nunc_abx, nunc_aby, nunc_abz, nunc_gbx, nunc_gby, nunc_gbz = [[] for i in range(6)]

# Useful constants
F = 1 / 298.257223563 
e_sqr = F*(2 - F)
a = 6378137 
e = np.sqrt(e_sqr) 
omega = 7.2921151467e-5

# Loop through output lcm log and throw data in lists
try:
    for event in in_log:
        if event.channel == pva_channel:
            msg = pva.decode(event.data)
            pva_time.append(msg.header.timestamp_valid.sec + (msg.header.timestamp_valid.nsec)*1e-9)
            lat_truth.append(msg.position.latitude)
            lon_truth.append(msg.position.longitude)
            alt_truth.append(msg.position.altitude)
            Vn_truth.append(msg.velocity[0])
            Ve_truth.append(msg.velocity[1])
            Vd_truth.append(msg.velocity[2])
            An_truth.append(r2d(wrapToPi(msg.attitude[0])))
            Ae_truth.append(r2d(wrapToPi(msg.attitude[1])))
            Ad_truth.append(r2d(wrapToPi(msg.attitude[2])))

            R_phi_truth = (a*(1-e**2))/((1-e**2*(np.sin(lat_truth[-1]))**2))**(3/2)
            R_lam_truth = a/((1-e**2*(np.sin(lat_truth[-1]))**2))**(1/2)

            N_truth.append(lat_truth[-1]*(R_phi_truth+alt_truth[-1]) - lat_truth[0]*(R_phi_truth+alt_truth[-1])) 
            E_truth.append(lon_truth[-1]*((R_lam_truth+alt_truth[-1])*np.cos(lat_truth[-1])) - lon_truth[0]*((R_lam_truth+alt_truth[-1])*np.cos(lat_truth[-1]))) 
            D_truth.append(-alt_truth[-1] - (-alt_truth[0]))
        elif event.channel == "mekf_solution":
            msg = pva.decode(event.data)
            time = msg.header.timestamp_valid.sec + (msg.header.timestamp_valid.nsec)*1e-9
            if not ekf_time or time>ekf_time[-1]:
                ekf_time.append(msg.header.timestamp_valid.sec + (msg.header.timestamp_valid.nsec)*1e-9)
                lat.append(msg.position.latitude)
                lon.append(msg.position.longitude)
                alt.append(msg.position.altitude)
                Vn.append(msg.velocity[0])
                Ve.append(msg.velocity[1])
                Vd.append(msg.velocity[2])
                An.append(r2d(msg.attitude[0]))
                Ae.append(r2d(msg.attitude[1]))
                Ad.append(r2d(msg.attitude[2]))
                eul.append([(msg.attitude[0]),(msg.attitude[1]),(msg.attitude[2])])
                unc_lat.append(3*np.sqrt(msg.covariance[0][0]))
                unc_lon.append(3*np.sqrt(msg.covariance[1][1]))
                unc_alt.append(3*np.sqrt(msg.covariance[2][2]))
                unc_Vn.append(3*np.sqrt(msg.covariance[3][3]))
                unc_Ve.append(3*np.sqrt(msg.covariance[4][4]))
                unc_Vd.append(3*np.sqrt(msg.covariance[5][5]))
                unc_An.append(r2d(3*np.sqrt(msg.covariance[6][6])))
                unc_Ae.append(r2d(3*np.sqrt(msg.covariance[7][7])))
                unc_Ad.append(r2d(3*np.sqrt(msg.covariance[8][8])))
                nunc_lat.append(-3*np.sqrt(msg.covariance[0][0]))
                nunc_lon.append(-3*np.sqrt(msg.covariance[1][1]))
                nunc_alt.append(-3*np.sqrt(msg.covariance[2][2]))
                nunc_Vn.append(-3*np.sqrt(msg.covariance[3][3]))
                nunc_Ve.append(-3*np.sqrt(msg.covariance[4][4]))
                nunc_Vd.append(-3*np.sqrt(msg.covariance[5][5]))
                nunc_An.append(r2d(-3*np.sqrt(msg.covariance[6][6])))
                nunc_Ae.append(r2d(-3*np.sqrt(msg.covariance[7][7])))
                nunc_Ad.append(r2d(-3*np.sqrt(msg.covariance[8][8])))

                R_phi = (a*(1-e**2))/((1-e**2*(np.sin(lat[-1]))**2))**(3/2)
                R_lam = a/((1-e**2*(np.sin(lat[-1]))**2))**(1/2)

                N.append(lat[-1]*(R_phi+alt[-1]) - lat[0]*(R_phi+alt[-1])) 
                E.append(lon[-1]*((R_lam+alt[-1])*np.cos(lat[-1])) - lon[0]*((R_lam+alt[-1])*np.cos(lat[-1]))) 
                D.append(-alt[-1] - (-alt[0]))

                unc_N.append(unc_lat[-1]*(R_phi+alt[-1])) 
                unc_E.append(unc_lon[-1]*((R_lam+alt[-1])*np.cos(lat[-1]))) 
                unc_D.append(unc_alt[-1]) 
                nunc_N.append(-1*unc_N[-1]) 
                nunc_E.append(-1*unc_E[-1]) 
                nunc_D.append(-1*unc_D[-1])
        elif event.channel == gps_channel:
            msg = gps.decode(event.data)
            gps_lat.append(msg.position.latitude)
            gps_lon.append(msg.position.longitude)
            gps_alt.append(msg.position.altitude)
        elif event.channel == "bias_solution":
            msg = bias.decode(event.data)
            bias_time.append(msg.timestamp_valid)
            ax_bias.append(msg.accel_bias[0])
            ay_bias.append(msg.accel_bias[1])
            az_bias.append(msg.accel_bias[2])
            gx_bias.append(msg.gyro_bias[0])
            gy_bias.append(msg.gyro_bias[1])
            gz_bias.append(msg.gyro_bias[2])
            unc_abx.append(3*np.sqrt(msg.covariance[0][0]) + ax_bias[-1])
            unc_aby.append(3*np.sqrt(msg.covariance[1][1]) + ay_bias[-1])
            unc_abz.append(3*np.sqrt(msg.covariance[2][2]) + az_bias[-1])
            unc_gbx.append(3*np.sqrt(msg.covariance[3][3]) + gx_bias[-1])
            unc_gby.append(3*np.sqrt(msg.covariance[4][4]) + gy_bias[-1])
            unc_gbz.append(3*np.sqrt(msg.covariance[5][5]) + gz_bias[-1])
            nunc_abx.append(-3*np.sqrt(msg.covariance[0][0]) + ax_bias[-1])
            nunc_aby.append(-3*np.sqrt(msg.covariance[1][1]) + ay_bias[-1])
            nunc_abz.append(-3*np.sqrt(msg.covariance[2][2]) + az_bias[-1])
            nunc_gbx.append(-3*np.sqrt(msg.covariance[3][3]) + gx_bias[-1])
            nunc_gby.append(-3*np.sqrt(msg.covariance[4][4]) + gy_bias[-1])
            nunc_gbz.append(-3*np.sqrt(msg.covariance[5][5]) + gz_bias[-1])
        else:
            continue

    in_log.close()
except StopIteration:
    pass

# cutoff pva after last ekf time
for i in range(len(pva_time)):
    if (pva_time[i]>ekf_time[-1]):
        pva_time = pva_time[:i]
        lat_truth = lat_truth[:i]
        lon_truth = lon_truth[:i]
        alt_truth = alt_truth[:i]
        Vn_truth = Vn_truth[:i]
        Ve_truth = Ve_truth[:i]
        Vd_truth = Vd_truth[:i]
        An_truth = An_truth[:i]
        Ae_truth = Ae_truth[:i]
        Ad_truth = Ad_truth[:i]
        N_truth = N_truth[:i] 
        E_truth = E_truth[:i] 
        D_truth = D_truth[:i]
        break


# size matching and interpolation
N_interp = interpolate_list_to_match_size(N, ekf_time, pva_time)
E_interp = interpolate_list_to_match_size(E, ekf_time, pva_time)
D_interp = interpolate_list_to_match_size(D, ekf_time, pva_time)
Vn_interp = interpolate_list_to_match_size(Vn, ekf_time, pva_time)
Ve_interp = interpolate_list_to_match_size(Ve, ekf_time, pva_time)
Vd_interp = interpolate_list_to_match_size(Vd, ekf_time, pva_time)
interp_angles = interpolate_euler_angles(ekf_time, np.array(eul), pva_time, rotation_sequence='ZYX', degrees=False)
An_interp = r2d(wrapToPi(interp_angles[:,0]))
Ae_interp = r2d(wrapToPi(interp_angles[:,1]))
Ad_interp = r2d(wrapToPi(interp_angles[:,2]))


# Calculate errors
N_err = np.array(N_interp) - np.array(N_truth)
E_err = np.array(E_interp) - np.array(E_truth)
D_err = np.array(D_interp) - np.array(D_truth)
Vn_err = np.array(Vn_interp) - np.array(Vn_truth)
Ve_err = np.array(Ve_interp) - np.array(Ve_truth)
Vd_err = np.array(Vd_interp) - np.array(Vd_truth)
An_err = wrap_angle_to_180(np.array(An_interp) - np.array(An_truth))
Ae_err = wrap_angle_to_180(np.array(Ae_interp) - np.array(Ae_truth))
Ad_err = wrap_angle_to_180(np.array(Ad_interp) - np.array(Ad_truth))

# Plotting
try:
    plt.figure('Trajectory')
    plt.plot(lon_truth,lat_truth)
    plt.plot(lon,lat)
    plt.scatter(gps_lon,gps_lat,marker="o",s=4,facecolors='none',edgecolors='green')
    plt.title("Trajectory")
    plt.xlabel("Lon (rad)")
    plt.ylabel("Lat (rad)")
    plt.legend(['Truth','MEFK','GPS'])

    plt.figure('Position LLA')
    plt.subplot(3,1,1)
    plt.plot(pva_time,lat_truth)
    plt.plot(ekf_time,lat)
    plt.plot(ekf_time_pa,lat_pa)
    plt.ylabel('Lat (rad)')
    plt.title("Position LLA")
    plt.legend(['Truth','MEKF','peek ahead'])
    plt.subplot(3,1,2)
    plt.plot(pva_time,lon_truth)
    plt.plot(ekf_time,lon)
    plt.plot(ekf_time_pa,lon_pa)
    plt.ylabel('Lon (rad)')
    plt.subplot(3,1,3)
    plt.plot(pva_time,alt_truth)
    plt.plot(ekf_time,alt)
    plt.plot(ekf_time_pa,alt_pa)
    plt.ylabel('Alt (m)')

    plt.figure("Velocity")
    plt.subplot(3,1,1)
    plt.plot(pva_time,Vn_truth)
    plt.plot(ekf_time,Vn)
    plt.title("Velocity")
    plt.ylabel('Vn (m/s)')
    plt.legend(['Truth','MEKF'])
    plt.subplot(3,1,2)
    plt.plot(pva_time,Ve_truth)
    plt.plot(ekf_time,Ve)
    plt.ylabel('Ve (m/s)')
    plt.subplot(3,1,3)
    plt.plot(pva_time,Vd_truth)
    plt.plot(ekf_time,Vd)
    plt.xlabel('Time (s)')
    plt.ylabel('Vd (m/s)')

    plt.figure("Attitude")
    plt.subplot(3,1,1)
    plt.plot(pva_time,An_truth)
    plt.plot(ekf_time,An)
    plt.title("Attitude")
    plt.ylabel('\u03B81 (deg)')
    plt.legend(['Truth','MEKF'])
    plt.subplot(3,1,2)
    plt.plot(pva_time,Ae_truth)
    plt.plot(ekf_time,Ae)
    plt.ylabel('\u03B82 (deg)')
    plt.subplot(3,1,3)
    plt.plot(pva_time,Ad_truth)
    plt.plot(ekf_time,Ad)
    plt.xlabel('Time (s)')
    plt.ylabel('\u03B83 (deg)')

    plt.figure("Position Error NED")
    plt.subplot(3,1,1)
    plt.plot(pva_time,N_err)
    plt.plot(ekf_time,unc_N,"black")
    plt.plot(ekf_time,nunc_N,"black")
    plt.title("Position Error")
    plt.ylabel("N (m)")
    plt.subplot(3,1,2)
    plt.plot(pva_time,E_err)
    plt.plot(ekf_time,unc_E,"black")
    plt.plot(ekf_time,nunc_E,"black")
    plt.ylabel("E (m)")
    plt.subplot(3,1,3)
    plt.plot(pva_time,D_err)
    plt.plot(ekf_time,unc_D,"black")
    plt.plot(ekf_time,nunc_D,"black")
    plt.xlabel("Time (s)")
    plt.ylabel("D (m)")

    plt.figure("Velocity Error")
    plt.subplot(3,1,1)
    plt.plot(pva_time,Vn_err)
    plt.plot(ekf_time,unc_Vn,"black")
    plt.plot(ekf_time,nunc_Vn,"black")
    plt.ylabel("Vn (m/s)")
    plt.title("Velocity Error")
    plt.subplot(3,1,2)
    plt.plot(pva_time,Ve_err)
    plt.plot(ekf_time,unc_Ve,"black")
    plt.plot(ekf_time,nunc_Ve,"black")
    plt.ylabel("Ve (m/s)")
    plt.subplot(3,1,3)
    plt.plot(pva_time,Vd_err)
    plt.plot(ekf_time,unc_Vd,"black")
    plt.plot(ekf_time,nunc_Vd,"black")
    plt.ylabel("Vd (m/s)")

    plt.figure("Attitude Error")
    plt.subplot(3,1,1)
    plt.plot(pva_time,An_err)
    plt.plot(ekf_time,unc_An,"black")
    plt.plot(ekf_time,nunc_An,"black")
    plt.ylabel('\u03B81 (deg)')
    plt.title("Attitude Error")
    plt.subplot(3,1,2)
    plt.plot(pva_time,Ae_err)
    plt.plot(ekf_time,unc_Ae,"black")
    plt.plot(ekf_time,nunc_Ae,"black")
    plt.ylabel('\u03B82 (deg)')
    plt.subplot(3,1,3)
    plt.plot(pva_time,Ad_err)
    plt.plot(ekf_time,unc_Ad,"black")
    plt.plot(ekf_time,nunc_Ad,"black")
    plt.xlabel('Time (s)')
    plt.ylabel('\u03B83 (deg)')

    plt.figure("Accel Bias")
    plt.subplot(3,1,1)
    plt.plot(bias_time,ax_bias)
    plt.plot(bias_time,unc_abx,"black")
    plt.plot(bias_time,nunc_abx,"black")
    plt.ylabel("X (m/s\u00B2)")
    plt.title("Accel Bias")
    plt.subplot(3,1,2)
    plt.plot(bias_time,ay_bias)
    plt.plot(bias_time,unc_aby,"black")
    plt.plot(bias_time,nunc_aby,"black")
    plt.ylabel("Y (m/s\u00B2)")
    plt.subplot(3,1,3)
    plt.plot(bias_time,az_bias)
    plt.plot(bias_time,unc_abz,"black")
    plt.plot(bias_time,nunc_abz,"black")
    plt.xlabel("Time (s)")
    plt.ylabel("Z (m/s\u00B2)")

    plt.figure("Gyro Bias")
    plt.subplot(3,1,1)
    plt.plot(bias_time,gx_bias)
    plt.plot(bias_time,unc_gbx,"black")
    plt.plot(bias_time,nunc_gbx,"black")
    plt.ylabel("X (rad/s)")
    plt.title("Gyro Bias")
    plt.subplot(3,1,2)
    plt.plot(bias_time,gy_bias)
    plt.plot(bias_time,unc_gby,"black")
    plt.plot(bias_time,nunc_gby,"black")
    plt.ylabel("Y (rad/s)")
    plt.subplot(3,1,3)
    plt.plot(bias_time,gz_bias)
    plt.plot(bias_time,unc_gbz,"black")
    plt.plot(bias_time,nunc_gbz,"black")
    plt.xlabel("Time (s)")
    plt.ylabel("Z (rad/s)")

    plt.show()
except KeyboardInterrupt:
    print('\n\nClosing plots and quitting program...\n')