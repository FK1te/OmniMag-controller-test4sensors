import os
import sys
import csv
import time
import struct
import serial
import numpy as np
import pandas as pd
import threading

import rtde_control
import rtde_receive

sys.path.append('./')
sys.path.append('./scripts')

from scripts.communication import ArduinoMinimacsCommunication
from scripts.controller_magnet import MagnetControllerOpenLoop, MagnetControllerClosedLoop

# ======== CONFIGURATION ========
TRAJECTORY_NO = 1
CONTROLLER_NAME = 'ClosedLoop'
R_INIT = np.array([0, 0, -0.2])
B_TARGET_MAG = 0.01
B_MAGNET = 198.45

# ======== TRAJECTORY DEFINITION ========
dt = 0.1
total_trajectory_time = 53.0 
t_vals = np.arange(0, total_trajectory_time, dt)

if TRAJECTORY_NO == 1:
    x = np.sin(t_vals)
    y = np.cos(t_vals)
    z = np.zeros_like(t_vals)
elif TRAJECTORY_NO == 2:
    x = np.zeros_like(t_vals)
    y = np.cos(t_vals)
    z = np.sin(t_vals)
else:
    raise ValueError("Invalid trajectory number")

trajectory = np.stack((x, y, z), axis=1)

# ======== SENSOR INITIALIZATION ========
COM_PORT = 'COM7'
BAUD_RATE = 1000000
TIMEOUT = 1
ser = serial.Serial(COM_PORT, BAUD_RATE, timeout=TIMEOUT)
time.sleep(2)

def read_sensor_data():
    global ser
    ser.write(b'U')
    raw = ser.read(1)
    if not raw:
        return [None] * 3
    flags = raw[0]
    if flags:
        data = ser.read(12)
        if len(data) == 12:
            x, y, z = struct.unpack('fff', data)
            return [x * 1e-6, y * 1e-6, z * 1e-6]
    return [None, None, None]

# ======== CONTROLLER SETUP ========
gains = {
    'KP_x': 825, 'KI_x': 5775, 'KD_x': 77.8,
    'KP_y': 1278, 'KI_y': 7672, 'KD_y': 140,
    'KP_z': 577.5, 'KI_z': 4908.8, 'KD_z': 44.8,
    'Lambda': 0.0
}
sat_v = 0.9
sat_e = 0.03
feedforward_scale = 2000.0

if "openloop" in CONTROLLER_NAME.lower():
    controller = MagnetControllerOpenLoop()
elif "closedloop" in CONTROLLER_NAME.lower():
    controller = MagnetControllerClosedLoop()
    controller.set_pid_gains(gains=gains)
    controller.sat_v = sat_v
    controller.sat_e = sat_e
    controller.feedforward_scale = feedforward_scale
else:
    raise ValueError("Invalid controller name.")

comm = ArduinoMinimacsCommunication()
comm.change_status_enable_disable_current(True)

# ======== ROBOT SETUP ========
ip_address = "192.168.1.100"
rtde_c = rtde_control.RTDEControlInterface(ip_address)
rtde_r = rtde_receive.RTDEReceiveInterface(ip_address)
target_robot_pos = rtde_r.getActualTCPPose() 
time.sleep(1)

r_vector = R_INIT.copy()
EE_POSITION = R_INIT + np.array(target_robot_pos[0:3])

# ======== FRAME TRANSFORMATION ========
rotation_matrix = np.array([
    [0, 0, 1],
    [0, -1, 0],
    [1, 0, 0]
])

# ======== HELPER FUNCTIONS ========
def angular_error_deg(target, actual):
    target = np.array(target)
    actual = np.array(actual)
    if None in target or None in actual or np.linalg.norm(target) == 0 or np.linalg.norm(actual) == 0:
        return np.nan
    cos_angle = np.clip(np.dot(target, actual) / (np.linalg.norm(target) * np.linalg.norm(actual)), -1.0, 1.0)
    angle_rad = np.arccos(cos_angle)
    return np.degrees(angle_rad)

def compute_target_m(m_dir_catheter, r, catheter2tcp):
    mu0 = 4 * np.pi * 1e-7
    r = np.array(r)
    r_norm = np.linalg.norm(r)
    if r_norm == 0 or np.linalg.norm(m_dir_catheter) == 0:
        return np.zeros(3)

    r_unit = r / r_norm
    A = (mu0 / (4 * np.pi)) * (1 / r_norm**3) * (3 * np.outer(r_unit, r_unit) - np.eye(3))

    m_dir_catheter = np.array(m_dir_catheter)
    m_dir_catheter /= np.linalg.norm(m_dir_catheter)
    B_target_catheter = B_TARGET_MAG * m_dir_catheter

    B_target_tcp = catheter2tcp @ B_target_catheter

    try:
        m_tcp = np.linalg.pinv(A) @ B_target_tcp
    except np.linalg.LinAlgError:
        m_tcp = np.zeros(3)
    return m_tcp

# ======== THREAD FUNCTIONS ========
exit_flag = threading.Event()
target_reset = False
index = 0
log_data = []
error_log = []
t0 = time.time()

def update_target():
    global m_target, m_target_catheter, target_reset, index
    while index < len(trajectory) and not exit_flag.is_set():
        m_target_catheter = trajectory[index]
        m_target = compute_target_m(m_target_catheter, r=r_vector, catheter2tcp=rotation_matrix)
        target_reset = True
        index += 1
        time.sleep(dt)

def control_magnet():
    global m_target, m_target_catheter, target_reset
    while index < len(trajectory) and not exit_flag.is_set():
        if target_reset:
            controller.reset()
            target_reset = False
        m_current = comm.get_magnetic_field()
        u, _ = controller.compute_control_currents(m_current=m_current, m_target=m_target)
        comm.set_currents_in_coils(u)

        m_current_catheter = read_sensor_data()
        err = abs(np.linalg.norm(m_current_catheter) - B_TARGET_MAG)
        error_log.append(err)

        print(err)

        log_data.append({
            'time': time.time() - t0,
            'm_target_x': m_target[0], 'm_target_y': m_target[1], 'm_target_z': m_target[2],
            'm_current_x': m_current_catheter[0], 'm_current_y': m_current_catheter[1], 'm_current_z': m_current_catheter[2],
            'angular_error_deg': err
        })
        time.sleep(0.001)

def update_robot_pos():
    global  m_target, m_target_catheter, r_vector 
    current_robot_pose = rtde_r.getActualTCPPose() 
    r_vector = EE_POSITION - np.array(current_robot_pose[0:3])

    mu0 = 4 * np.pi * 1e-7
    r = r_vector.copy()
    r_norm = np.linalg.norm(r)
    r_unit = r / r_norm

    x = ((mu0 / (4 * np.pi)) * (3 * np.outer(r_unit, r_unit) - np.eye(3))) @ m_target
    x_norm = np.linalg.norm(x)
    
    target_r_norm = (x_norm / B_TARGET_MAG ) ** (1/3)

    delta_tcp = (target_r_norm - r_norm) * r_unit 

    current_robot_pose[0] += delta_tcp[0]
    current_robot_pose[1] += delta_tcp[1]
    current_robot_pose[2] += delta_tcp[2]

    target_robot_pos = [i for i in current_robot_pose] 

    print(delta_tcp)


def move_robot():
    """
    global target_robot_pos
    rtde_c.moveL(target_robot_pos, 0.05, 0.01, asynchronous=True)
    """
    pass

# ======== START THREADS ========
target_thread = threading.Thread(target=update_target)
control_thread = threading.Thread(target=control_magnet)
robot_update_thread = threading.Thread(target=update_robot_pos)

target_thread.start()
control_thread.start()
robot_update_thread.start()

# Wait for threads to finish
try:
    target_thread.join()
    control_thread.join()
    robot_update_thread.join()
except KeyboardInterrupt:
    exit_flag.set()

# ======== SAVE LOG ========
df_log = pd.DataFrame(log_data)
df_log.to_csv('magnet_control_log.csv', index=False)
