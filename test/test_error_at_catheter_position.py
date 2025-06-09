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

[UR_x, UR_y, UR_z, UR_rx, UR_ry, UR_rz] = [ 0.7690617488094837, 
                                           0.2543022600564637, 
                                           0.7222612027051074, 
                                           -2.263276906897582,
                                            -0.12431620701839485, 
                                            -2.085370041783]
print(f"Robot TCP position: {UR_x}, {UR_y}, {UR_z}, {UR_rx}, {UR_ry}, {UR_rz}.")

time_per_input = 10.0
inputs_list = [
    np.array([0.8, -0.6, 0.0]), 
    np.array([0.0, 0.8, 0.6]),
    np.array([-0.6, 0.0, 0.8]),
    np.array([0.6, 0.8, 0.0]), 
    np.array([0.8, 0.0, 0.6]),
    np.array([0.0, 0.6, -0.8]),
]
controller = MagnetControllerPID()
gains = {
    'KP_x': 600.0,
    'KI_x': 890.0,
    'KD_x': 20.0,
    'KP_y': 600.0,
    'KI_y': 702.0,
    'KD_y': 16.0,
    'KP_z': 510.0,
    'KI_z': 382.0,
    'KD_z': 10.0,
    'Lambda': 1.0
}
controller.set_pid_gains(gains=gains)

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
header += ["angular_error_degrees", "tcp_x", "tcp_y", "tcp_z", "tcp_x", "tcp_y", "tcp_z"]

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
                values.extend([x, y, z])
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
            while time.time() - t0 < time_per_input:
                timestamp = datetime.now().isoformat()
                s_vec = comm.get_sensor_readings()
                m_current = S_dagger @ s_vec
                data1 = m_current.tolist()
                data2 = read_sensor_data(ser, 3)
                u, data = controller.compute_control_currents(m_current=m_current, m_target=m_target)
                comm.set_currents_in_coils(u)       
                row = [timestamp] + m_target_tcp + data1 + data2 + [float(data["angular_error_degrees"]), UR_x, UR_y, UR_z, UR_rx, UR_ry, UR_rz]
                writer.writerow(row)
                print("Logged:", row)
                time.sleep(0.001)
                
        print("Data collection ended!")

except KeyboardInterrupt:
    print("Logging stopped by user.")

finally:
    csvfile.close()
    ser.close()
    comm.shutdown()
