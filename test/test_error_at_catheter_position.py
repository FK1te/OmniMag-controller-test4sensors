import os
import sys
import time
import glob
import struct
import serial

import numpy as np
import pandas as pd
from scipy.optimize import minimize

sys.path.append('./')
sys.path.append('./scripts')

from scripts.communication import ArduinoMinimacsCommunication

comm = ArduinoMinimacsCommunication()
comm.change_status_enable_disable_current(True)

COM_PORT = 'COM7'
BAUD_RATE = 1000000
TIMEOUT = 1

ser = serial.Serial(COM_PORT, BAUD_RATE, timeout=TIMEOUT)
time.sleep(2)

data_folder = "error_study"

robot_tcp_pos = [0, 0, 0, 0, 0, 0]

p1_est = np.array([0, 0, 0])
p2_est = np.array([0, 0, 0])

p3_est = np.array([0, 0, 0])
p4_est = np.array([0, 0, 0])
p5_est = np.array([0, 0, 0])

p_catheter = np.array([0, 0, 0])

data_1 = []
data_2 = []

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

def compute_A(v):
    mu0 = 4 * np.pi * 1e-7
    v_norm = np.linalg.norm(v)
    v_hat = v/v_norm
    scalar_multp = mu0/(4 * np.pi) * (1/(v_norm)**3)
    A = scalar_multp * (3 * np.outer(v_hat, v_hat) - np.eye(3))
    return A

def compute_S_dagger(offset):
    R = np.array([[0, 0, -1], [0, 1, 0], [1, 0, 0]])
    R6 = np.block([[R, np.zeros((3, 3))], [np.zeros((3, 3)), R]])
    S = np.vstack((compute_A(p1_est+offset), compute_A(p2_est+offset)))
    return np.linalg.pinv(S) @ R6

def compute_S_dagger_catheter(offset):
    R = np.array([[0, 0, -1], [0, 1, 0], [1, 0, 0]])
    S = np.vstack((compute_A(p3_est+offset), compute_A(p4_est+offset), compute_A(p5_est+offset)))
    return np.linalg.pinv(S)

def get_m_target_in_tcp_frame(m_target_catheter):
    rx, ry, rz = robot_tcp_pos[3:]
    Rx = np.array([
        [1, 0, 0],
        [0, np.cos(rx), -np.sin(rx)],
        [0, np.sin(rx), np.cos(rx)]
    ])
    
    Ry = np.array([
        [np.cos(ry), 0, np.sin(ry)],
        [0, 1, 0],
        [-np.sin(ry), 0, np.cos(ry)]
    ])
    
    Rz = np.array([
        [np.cos(rz), -np.sin(rz), 0],
        [np.sin(rz), np.cos(rz), 0],
        [0, 0, 1]
    ])
    
    R = Rz @ Ry @ Rx
    return R @ m_target_catheter
    
def error_at_catheter_position(m_t, s3, s4, s5, S_dagger_catheter):
    m_s = S_dagger_catheter @ np.hstack((s3.flatten(), s4.flatten(), s5.flatten())).reshape(9, 1)
    m_s = m_s.flatten()
    m_s /= np.linalg.norm(m_s)
    cos_value = np.dot(m_t, m_s)
    sin_value = np.linalg.norm(np.cross(m_t, m_s))
    return np.rad2deg(np.arctan2(sin_value, cos_value))

# for all sensors
def collect_data():
    duration = 240  # seconds
    dt = 0.1        # time step
    time_vector = np.arange(0, duration, dt)

    positive_segment = 1000 * np.ones(duration/(2*dt))
    negative_segment = -1000 * np.ones(duration/(2*dt))
    positive_negative_step = np.concatenate((positive_segment, negative_segment))

    os.makedirs(data_folder, exist_ok=True)

    test_cases = [
        {
            'name': 'TC1_unit_step_u1',
            'u1': positive_negative_step,
            'u2': np.zeros_like(positive_negative_step),
            'u3': np.zeros_like(positive_negative_step)
        },
        {
            'name': 'TC2_unit_step_u2',
            'u1': np.zeros_like(positive_negative_step),
            'u2': positive_negative_step,
            'u3': np.zeros_like(positive_negative_step),
        },
        {
            'name': 'TC3_unit_step_u3',
            'u1': np.zeros_like(positive_negative_step),
            'u2': np.zeros_like(positive_negative_step),
            'u3': positive_negative_step
        }
    ]
    try:
        for case in test_cases:
            print(f"\n[•] Running {case['name']}...")
            log = []
            start_time = time.monotonic()

            for i, (u1, u2, u3) in enumerate(zip(case['u1'], case['u2'], case['u3'])):
                t_now = time.monotonic() - start_time
                u = np.array([u1, u2, u3], dtype=int).tolist()
                comm.set_currents_in_coils(u)
                s_vec = comm.get_sensor_readings()
                s_vec_2 = read_sensor_data(ser, 3)

                data_1.append(s_vec)
                data_2.append(s_vec_2)

                log.append({
                    't': round(t_now, 3),
                    'u1': u1,
                    'u2': u2,
                    'u3': u3,
                    'm11': s_vec[0],
                    'm12': s_vec[1],
                    'm13': s_vec[2],
                    'm21': s_vec[3],
                    'm22': s_vec[4],
                    'm23': s_vec[5],
                    'm31': s_vec_2[0],
                    'm32': s_vec_2[1],
                    'm33': s_vec_2[2],
                    'm41': s_vec_2[3],
                    'm42': s_vec_2[4],
                    'm43': s_vec_2[5],
                    'm51': s_vec_2[6],
                    'm52': s_vec_2[7],
                    'm53': s_vec_2[8]
                })

                time.sleep(dt)

            df = pd.DataFrame(log)
            filename = f"{data_folder}/{case['name']}.csv"
            df.to_csv(filename, index=False)
            print(f"[✓] Log saved to {filename}")

    except KeyboardInterrupt:
        print("\n[✓] Interrupted by user.")
    finally:
        comm.shutdown()
        print("[✓] Arduino and Minimacs communications shutdown complete.")


# optimize for sensor positions 
def optimize_for_offsets():
    data_1 = np.array(data_1)
    data_2 = np.array(data_2)
    print(data_1.shape)
    print(data_2.shape)
    target_magnitude = 198.45

    # === Objective Function ===
    def objective_1(offset):
        S_dagger = compute_S_dagger(offset)
        m_vectors = S_dagger @ data_1
        norms = np.linalg.norm(m_vectors, axis=0)
        return np.sqrt(np.mean((norms - target_magnitude) ** 2))
    
    def objective_2(offset):
        S_dagger = compute_S_dagger_catheter(offset)
        m_vectors = S_dagger @ data_2
        norms = np.linalg.norm(m_vectors, axis=0)
        return np.sqrt(np.mean((norms - target_magnitude) ** 2))
    
    # === Grid Search Parameters ===
    search_range = np.linspace(-10, 10, 9)  # 9 points per axis → 729 total
    bounds = [(-10, 10)] * 3

    best_result = None
    best_fun = np.inf

    print("Starting grid search with local optimization...")

    for x in search_range:
        for y in search_range:
            for z in search_range:
                start = np.array([x, y, z])
                res = minimize(
                    lambda offset: objective_1(offset),
                    start,
                    method='L-BFGS-B',
                    bounds=bounds
                )
                if res.success and res.fun < best_fun:
                    best_fun = res.fun
                    best_result = res

                print(f"Start {start} -> RMSE: {res.fun:.4f}, Success: {res.success}")

    if best_result:
        print("\nBest result found:")
        print(f"Optimal Offset (mm): {best_result.x}")
        print(f"Final RMSE: {best_result.fun:.4f}")
    else:
        print("No successful optimization run found.")

    offset_1 = best_result.x

    # === Grid Search Parameters ===
    search_range = np.linspace(-10, 10, 9)  # 9 points per axis → 729 total
    bounds = [(-10, 10)] * 3

    best_result = None
    best_fun = np.inf

    print("Starting grid search with local optimization...")

    for x in search_range:
        for y in search_range:
            for z in search_range:
                start = np.array([x, y, z])
                res = minimize(
                    lambda offset: objective_2(offset),
                    start,
                    method='L-BFGS-B',
                    bounds=bounds
                )
                if res.success and res.fun < best_fun:
                    best_fun = res.fun
                    best_result = res

                print(f"Start {start} -> RMSE: {res.fun:.4f}, Success: {res.success}")

    if best_result:
        print("\nBest result found:")
        print(f"Optimal Offset (mm): {best_result.x}")
        print(f"Final RMSE: {best_result.fun:.4f}")
    else:
        print("No successful optimization run found.")

    offset_2 = best_result.x
    return offset_1, offset_2

# run the test script with 
def run_test_cases_for_target_m():
    pass

def main():
    os.makedirs(data_folder, exist_ok=True)

    pass