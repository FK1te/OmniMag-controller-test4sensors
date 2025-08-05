import os
import sys
import csv
import time
import serial
import struct
import numpy as np
from datetime import datetime

sys.path.append('./')
sys.path.append('./scripts')

from scripts.communication import ArduinoMinimacsCommunication
from scripts.controller_magnet import MagnetControllerPID
from scripts.test_params import ARDUINO_MINIMACS6_DEFAULT_PARAMS

S_dagger = ARDUINO_MINIMACS6_DEFAULT_PARAMS["S_dagger_matrix"]

time_per_input = 10.0

import numpy as np 
from itertools import product 
def rotation_matrix(roll, pitch, yaw): 
    # Convert degrees to radians 
    r, p, y = np.radians([roll, pitch, yaw]) 
    # Rotation matrices 
    Rx = np.array([ 
    [1, 0, 0], 
    [0, np.cos(r), -np.sin(r)], 
    [0, np.sin(r), np.cos(r)] 
    ]) 

    Ry = np.array([ 
    [np.cos(p), 0, np.sin(p)], 
    [0, 1, 0], 
    [-np.sin(p), 0, np.cos(p)] 
    ]) 

    Rz = np.array([ 
    [np.cos(y), -np.sin(y), 0], 
    [np.sin(y), np.cos(y), 0], 
    [0, 0, 1] 
    ]) 

    return Rz @ Ry @ Rx

def generate_vectors(step_deg=36): 
    angles = list(range(0, 360, step_deg)) 
    base_vector = np.array([1, 0, 0]) # could use other directions as needed 
    vectors = [] 

    for pitch, yaw in product(angles, repeat=2): 
        R = rotation_matrix(0.0, pitch, yaw) 
        v = R @ base_vector 
        v_unit = v / np.linalg.norm(v) 
        vectors.append(tuple(np.round(v_unit, 6))) 
    return np.array(vectors) 

inputs_list = generate_vectors()

controller = MagnetControllerPID()
gains = {
    'KP_x': 1235 / 1.2,
    'KI_x': 2965 / 1.2,
    'KD_x': 32.0,

    'KP_y': 1490.0 / 1.2,
    'KI_y': 1015.0 / 1.2,
    'KD_y': 75.0,

    'KP_z': 955.0 / 1.2,
    'KI_z': 2085 / 1.2,
    'KD_z': 35.0,

    'Lambda': 0.0
}

controller.set_pid_gains(gains=gains)
controller.sat_v = 5.0 
controller.sat_e = 0.857

COM_PORT = 'COM7'
BAUD_RATE = 1000000
TIMEOUT = 1

ser = serial.Serial(COM_PORT, BAUD_RATE, timeout=TIMEOUT)
time.sleep(2)

comm = ArduinoMinimacsCommunication()
comm.change_status_enable_disable_current(True)

# === CSV SETUP ===
filename = f"sensor_log_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
header = ["timestamp", "m_target_tcp_x", "m_target_tcp_y", "m_target_tcp_z"]
for i in range(1, 6):  # 5 sensors
    header += [f"MLX{i}_X", f"MLX{i}_Y", f"MLX{i}_Z"]
header += ["angular_error_degrees"]

csvfile = open(filename, "w", newline="")
writer = csv.writer(csvfile)
writer.writerow(header)

# === HELPER TO READ SENSOR DATA ===
def read_sensor_data(ser, expected_sensors):
    ser.write(b'U')
    raw = ser.read(1)  # Read flag byte
    if not raw:
        return [None] * (3 * expected_sensors)

    flags = raw[0]
    values = []

    for i in range(expected_sensors):
        if flags & (1 << i):
            data = ser.read(12)  # 3 floats per sensor
            if len(data) == 12:
                x, y, z = struct.unpack('fff', data)
                values.extend([x * 1e-6, y * 1e-6, z * 1e-6])
            else:
                values.extend([None, None, None])
        else:
            values.extend([None, None, None])
    return values

flag = False

try:
    while True:
        if flag:
            continue
        flag = True

        for i, m_target in enumerate(inputs_list):
            t0 = time.time()
            m_target_tcp = m_target.tolist()
            print(i+1)
            while time.time() - t0 < time_per_input:
                timestamp = datetime.now().isoformat()
                s_vec = comm.get_sensor_readings()
                m_current = S_dagger @ s_vec
                data1 = s_vec.tolist()
                data2 = read_sensor_data(ser, 3)
                u, data = controller.compute_control_currents(m_current=m_current, m_target=m_target)
                comm.set_currents_in_coils(u)       
                row = [timestamp] + m_target_tcp + data1 + data2 + [float(data["angular_error_degrees"])]
                writer.writerow(row)
                time.sleep(0.001)
                
        print("Data collection ended!")

except KeyboardInterrupt:
    print("Logging stopped by user.")

finally:
    csvfile.close()
    ser.close()
    comm.shutdown()
