import os
import sys
import csv
import time
import numpy as np

sys.path.append('./scripts')

from scripts.communication import ArduinoMinimacsCommunication
from scripts.controller_magnet import MagnetControllerStatic, MagnetControllerPID

controller_name = 'static'

seconds_per_target = 10.0 

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

def R_X(theta):
    return np.array([
        [1, 0, 0],
        [0, np.cos(theta), -np.sin(theta)],
        [0, np.sin(theta), np.cos(theta)]
    ])

def R_Y(theta):
    return np.array([
        [np.cos(theta), 0, np.sin(theta)],
        [0, 1, 0],
        [-np.sin(theta), 0, np.cos(theta)]
    ])

def R_Z(theta):
    return np.array([
        [np.cos(theta), -np.sin(theta), 0],
        [np.sin(theta), np.cos(theta), 0],
        [0, 0, 1]
    ])

rotation_map = {
    'x': R_X,
    'y': R_Y,
    'z': R_Z
}

test_cases = [
    {   
        'base_vec_name': 'X',
        'base_vec': np.array([1.0, 0.0, 0.0]), 
        'rotational_axis': ['Y', 'Z'],
        'angles_deg': [2.0, 5.0, 10.0, 30.0, 45.0, 90.0]
    },
    {
        'base_vec_name': 'Y',
        'base_vec': np.array([0.0, 1.0, 0.0]), 
        'rotational_axis': ['X', 'Z'],
        'angles_deg': [2.0, 5.0, 10.0, 30.0, 45.0, 90.0]
    },
    {
        'base_vec_name': 'Z',
        'base_vec': np.array([0.0, 0.0, 1.0]), 
        'rotational_axis': ['X', 'Y'],
        'angles_deg': [2.0, 5.0, 10.0, 30.0, 45.0, 90.0]
    }
]

def main():
    comm = ArduinoMinimacsCommunication()
    comm.change_status_enable_disable_current(True)

    if controller_name == "static":
        controller = MagnetControllerStatic()
    elif "pid" in controller_name.lower():
        global gains
        controller = MagnetControllerPID()
        controller.set_pid_gains(gains=gains)
    else:
        raise ValueError("Invalid controller name.")

    os.makedirs("rotation_logs", exist_ok=True)

    data_log_columns = [
            'time', 'm_x', 'm_y', 'm_z',
            'm_target_x', 'm_target_y', 'm_target_z',
            'angular_error_degrees', 'p_term_mag', 'i_term_mag', 'd_term_mag', 'input_torque_mag',
            'KP_x', 'KI_x', 'KD_x', 'KP_y', 'KI_y', 'KD_y', 'KP_z', 'KI_z', 'KD_z',
            'i_x', 'i_y', 'i_z', 'target_angle'
        ]

    try:
        while True:
            for i, case in enumerate(test_cases):
                base_vector = case['base_vec']
                base_vector_name = case['base_vec_name']
                rotation_axes = case['rotational_axis']
                angles_deg = case['angles_deg']
                angles_rad = np.deg2rad(angles_deg)

                for axis in rotation_axes:
                    rotate = rotation_map[axis.lower()]
                    filename = f"rotation_logs/{base_vector_name}_rot_{axis}deg.csv"
                    with open(filename, mode='w', newline='') as file:
                        writer = csv.writer(file)
                        writer.writerow(data_log_columns)
                        start_time = time.time()
                        for angle_deg, angle_rad in zip(angles_deg, angles_rad):
                            current_vector = base_vector.copy()
                            rotated_vector = rotate(angle_rad) @ current_vector
                        
                            t0 = time.time()
                            while time.time() - t0 < 10.0:
                                m_current = comm.get_magnetic_field()
                                u, data = controller.compute_control_currents(m_current=m_current, m_target=current_vector)
                                comm.set_currents_in_coils(u)
                                data['time'] = time.time() - start_time
                                data['target_angle'] = 0
                                writer.writerow(data)
                                time.sleep(0.1)

                            t0 = time.time()
                            while time.time() - t0 < 10.0:
                                m_current = comm.get_magnetic_field()
                                u, data = controller.compute_control_currents(m_current=m_current, m_target=rotated_vector)
                                comm.set_currents_in_coils(u)
                                data['time'] = time.time() - start_time
                                data['target_angle'] = angle_deg
                                writer.writerow(data)
                                time.sleep(0.1)

    except KeyboardInterrupt:
        print("\n[✓] Interrupted by user.")
    finally:
        comm.shutdown()
        print("[✓] Arduino and Minimacs communications shutdown complete.")
