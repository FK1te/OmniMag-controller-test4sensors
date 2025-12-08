# External Magnetic Field Measurement Comparison Test
# This script runs a magnetic field tracking test using both internal (robot-mounted) and external Hall sensors.
# It supports both open-loop and closed-loop controllers, evaluates angular errors, and logs performance data.

import os
import sys
import csv
import time
import struct
import serial
import numpy as np
import pandas as pd
import threading

sys.path.append('./')
sys.path.append('./scripts')

from scripts.communication import ArduinoMinimacsCommunication
from scripts.controller_magnet import MagnetControllerOpenLoop, MagnetControllerClosedLoop


# =================== TRAJECTORY DEFINITION ===================
# Time base
dt = 2.0  # Consider changing by 2 seconds - Time step for steady-state
total_trajectory_time = 240
speed = 1

t_vals = np.arange(0, total_trajectory_time, dt)

# Define trajectory
trajectory_no = 1
if trajectory_no == 1:
    x = np.sin(t_vals)
    y = np.cos(t_vals)
    z = np.zeros_like(t_vals)
elif trajectory_no == 2:
    x = np.zeros_like(t_vals)
    y = np.cos(t_vals)
    z = np.sin(t_vals)
elif trajectory_no == 3:
    x = np.ones_like(t_vals)
    y = np.zeros_like(t_vals)
    z = np.zeros_like(t_vals)
elif trajectory_no == 4:
    x = np.zeros_like(t_vals)
    y = np.ones_like(t_vals)
    z = np.zeros_like(t_vals)
elif trajectory_no == 5:
    x = np.zeros_like(t_vals)
    y = np.zeros_like(t_vals)
    z = np.ones_like(t_vals)
else:
    raise ValueError("Invalid trajectory number")

trajectory = np.stack((x, y, z), axis=1)

# =================== EXTERNAL SENSOR SETUP ===================
# Serial setup for External sensor - 2nd Arduino
COM_PORT = 'COM5'
BAUD_RATE = 1000000
TIMEOUT = 1
try:
    ser = serial.Serial(COM_PORT, BAUD_RATE, timeout=TIMEOUT)
    time.sleep(2)  # Wait for Arduino to initialize
    print(f"[✓] Connected to [External sensor] Arduino on {COM_PORT}")
except serial.SerialException as e:
    print(f"[!] Arduino connection error: {e}")

# Rotation and position constants
# For external sensor only - Correct rotation matrix
rotation_matrix = np.array([
    [0, 0, 1],
    [0, -1, 0],
    [1, 0, 0]
])
# r_vector = np.array([0, 0, -0.2]) # - TO BE VERIFIED 
r_vector = 1e-3 * np.array([86.6, -86.6, -86.6])


# =================== CONTROLLER SETUP ===================
# Controller setup
controller_name = f'{speed}x_open_loop'

gains = {
    'KP_x': 825, 'KI_x': 5775, 'KD_x': 77.8,
    'KP_y': 1278, 'KI_y': 7672, 'KD_y': 140,
    'KP_z': 577.5, 'KI_z': 4908.8, 'KD_z': 44.8,
    'Lambda': 0.0
}
sat_v = 0.9
sat_e = 0.03
feedforward_scale = 2000.0


# Call the class Communication to use its functions
comm = ArduinoMinimacsCommunication()
comm.change_status_enable_disable_current(True)

if "open_loop" in controller_name.lower():
    controller = MagnetControllerOpenLoop()
elif "closed_loop" in controller_name.lower():
    controller = MagnetControllerClosedLoop()
    controller.set_closed_loop_gains(gains=gains) # previously set_pid_gains but function doesn't exist
    controller.sat_v = sat_v
    controller.sat_e = sat_e
    controller.feedforward_scale = feedforward_scale
else:
    raise ValueError("Invalid controller name.")

# Helper functions
def read_sensor_data(ser):
    # ONLY FOR 1 sensor
    # (external measurement - catheter tip)
    ser.write(b'U')
    raw = ser.read(1)
    if not raw:
        return [None] * 3
    flags = raw[0]
    if flags:
        data = ser.read(12)
        if len(data) == 12:
            x, y, z = struct.unpack('fff', data)
            return [x * 1e-6, y * 1e-6, z * 1e-6] # converted to tesla (instead of micro-tesla)
    return [None, None, None]

def compute_target_m(m, r, catheter2tcp):
    mu0 = 4 * np.pi * 1e-7
    r_norm = np.linalg.norm(r)
    r_unit = r / r_norm
    A = (mu0 / (4 * np.pi)) * (1 / r_norm**3) * (3 * np.outer(r_unit, r_unit) - np.eye(3))
    return catheter2tcp @ (np.linalg.pinv(A) @ m)

def angular_error_deg(target, actual):
    target = np.array(target)
    actual = np.array(actual)
    if None in target or None in actual or np.linalg.norm(target) == 0 or np.linalg.norm(actual) == 0:
        return np.nan
    cos_angle = np.clip(np.dot(target, actual) / (np.linalg.norm(target) * np.linalg.norm(actual)), -1.0, 1.0)
    angle_rad = np.arccos(cos_angle)
    return np.degrees(angle_rad)

# Global shared variables
m_target = np.zeros(3)
m_target_catheter = np.zeros(3)
target_reset = True
error_log = []
log_data = []
index = 0
exit_flag = threading.Event()
t0 = time.time()

# Threads
def update_target():
    global m_target, m_target_catheter, target_reset, index
    while index < len(trajectory) and not exit_flag.is_set():
        t = time.time() - t0
        m_target_catheter = trajectory[index]
        m_target = compute_target_m(m_target_catheter, r=r_vector, catheter2tcp=rotation_matrix)
        target_reset = True
        index += 1
        time.sleep(dt)

def follow_trajectory():
    global m_target, m_target_catheter, target_reset
    while (time.time() - t0) < total_trajectory_time / speed and not exit_flag.is_set():
        if target_reset:
            controller.reset()
            target_reset = False

        # Get magnetic field - From communication class
        # Read the cluster of 4 hall sensors
        m_current = comm.get_magnetic_field()
        # print(m_current)
        u, _ = controller.compute_control_currents(m_current=m_current, m_target=m_target)
        comm.set_currents_in_coils(u)

        # Current catheter = External sensor (m_current_xyz)
        # Target           = From compute_target_m (See above)
        # What we should obtain at the catheter tip, from OmniMag4Sensors cluster
        m_current_catheter = read_sensor_data(ser)
        err = angular_error_deg(m_target_catheter, m_current_catheter)
        error_log.append(err)
        print("Angle error :", err)

        log_data.append({
            'time': time.time() - t0,
            'm_target_x': m_target[0], 'm_target_y': m_target[1], 'm_target_z': m_target[2],
            'm_current_x': m_current_catheter[0], 'm_current_y': m_current_catheter[1], 'm_current_z': m_current_catheter[2],
            'angular_error_deg': err
        })
        time.sleep(0.001)

# Main
def main():
    global log_data, error_log
    try:
        thread1 = threading.Thread(target=update_target)
        thread2 = threading.Thread(target=follow_trajectory)
        thread1.start()
        thread2.start()
        thread1.join()
        thread2.join()
    except KeyboardInterrupt:
        print("\n[✓] Interrupted by user.")
        exit_flag.set()
    finally:
        comm.shutdown()
        print("[✓] Arduino and Minimacs communications shutdown complete.")
        error_array = np.array([e for e in error_log if not np.isnan(e)])
        rmse = np.sqrt(np.mean(error_array**2)) if len(error_array) > 0 else np.nan
        mean_error = np.mean(error_array) if len(error_array) > 0 else np.nan
        print(f'{controller_name}_dt_{dt}')
        print(f"[✓] RMSE: {rmse:.4f} degrees.")
        print(f"[✓] Mean error: {mean_error:.4f} degrees.")

        df_log = pd.DataFrame(log_data)
        filename = f'{controller_name}__traj{trajectory_no}_dt_{dt}_avg_error_{mean_error:.2f}.csv'
        df_log.to_csv(os.path.join(os.getcwd(), filename), index=False)
        print(f"[✓] Data saved to {filename}")

if __name__ == "__main__":
    main()
