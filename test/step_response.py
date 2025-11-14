"""
Collects step response data for magnetic field controllers
(open-loop or closed-loop) across varying target step angles.

Each test rotates a unit vector about different axes and logs:
- Sensor readings
- Target direction
- Angular error
- Control effort (P/I/D/torque)
- Controller parameters and target step angle

This data is saved in CSV format for later analysis.
"""

import os
import csv
import time
import numpy as np
import sys

sys.path.append('./')
sys.path.append('./scripts')

from scripts.communication import ArduinoMinimacsCommunication
from scripts.controller_magnet import MagnetControllerOpenLoop, MagnetControllerClosedLoop

# Choose controller type: "openloop" or "closedloop"
controller_name = 'openloop'

seconds_per_target = 5.0  # Duration to apply each target

# Closed-loop controller gain settings
gains = {
    'KP_x': 1300,
    'KI_x': 1300,
    'KD_x': 1300 * 0.05,
    'KP_y': 1250,
    'KI_y': 1250,
    'KD_y': 1250 * 0.05,
    'KP_z': 1000,
    'KI_z': 1000,
    'KD_z': 1000 * 0.05,
    'Lambda': 0.0
}

# Closed-loop controller additional params
sat_v = 2.5
sat_e = 0.025
feedforward_scale = 500.0

# Rotation matrices
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

# Test configuration: base vectors, axes, and step angles
test_cases = [
    {
        'base_vec_name': 'X',
        'base_vec': np.array([1.0, 0.0, 0.0]),
        'rotational_axis': ['Y', 'Z'],
        'angles_deg': [2.0, 5.0, 10.0, 30.0, 45.0, 90.0, 120.0, 150.0, 180.0]
    },
    {
        'base_vec_name': 'Y',
        'base_vec': np.array([0.0, 1.0, 0.0]),
        'rotational_axis': ['X', 'Z'],
        'angles_deg': [2.0, 5.0, 10.0, 30.0, 45.0, 90.0, 120.0, 150.0, 180.0]
    },
    {
        'base_vec_name': 'Z',
        'base_vec': np.array([0.0, 0.0, 1.0]),
        'rotational_axis': ['X', 'Y'],
        'angles_deg': [2.0, 5.0, 10.0, 30.0, 45.0, 90.0, 120.0, 150.0, 180.0]
    }
]

def main():
    comm = ArduinoMinimacsCommunication()
    comm.change_status_enable_disable_current(True)

    # Stabilize sensor readings
    for _ in range(5):
        print(comm.get_sensor_readings())

    # Initialize appropriate controller
    if "openloop" in controller_name.lower():
        controller = MagnetControllerOpenLoop()
    elif "closedloop" in controller_name.lower():
        controller = MagnetControllerClosedLoop()
        controller.set_closed_loop_gains(gains=gains)
        controller.sat_v = sat_v
        controller.sat_e = sat_e
        controller.feedforward_scale = feedforward_scale
    else:
        raise ValueError("Invalid controller name.")

    os.makedirs("rotation_logs", exist_ok=True)

    # Columns for CSV output
    data_log_columns = [
        'time', 'm_x', 'm_y', 'm_z',
        'm_target_x', 'm_target_y', 'm_target_z',
        'angular_error_degrees', 'p_term_mag', 'i_term_mag', 'd_term_mag', 'input_torque_mag',
        'KP_x', 'KI_x', 'KD_x', 'KP_y', 'KI_y', 'KD_y', 'KP_z', 'KI_z', 'KD_z',
        'i_x', 'i_y', 'i_z', 'target_angle'
    ]

    flag = False

    try:
        while True:
            if flag:
                continue
            flag = True

            for case in test_cases:
                base_vector = case['base_vec']
                base_name = case['base_vec_name']
                rotation_axes = case['rotational_axis']
                angles_deg = case['angles_deg']
                angles_rad = np.deg2rad(angles_deg)

                for axis in rotation_axes:
                    rotate = rotation_map[axis.lower()]
                    filename = f"rotation_logs/{controller_name}_controller_{base_name.lower()}_unit_vector_rot_around_{axis.lower()}_axis.csv"
                    
                    with open(filename, mode='w', newline='') as file:
                        writer = csv.writer(file)
                        writer.writerow(data_log_columns)
                        start_time = time.time()

                        for angle_deg, angle_rad in zip(angles_deg, angles_rad):
                            current_vector = base_vector.copy()
                            rotated_vector = rotate(angle_rad) @ current_vector

                            # Step 1: hold base vector
                            t0 = time.time()
                            controller.reset()
                            while time.time() - t0 < seconds_per_target:
                                m_current = comm.get_magnetic_field()
                                u, data = controller.compute_control_currents(m_current=m_current, m_target=current_vector)
                                comm.set_currents_in_coils(u)
                                data['time'] = time.time() - start_time
                                data['target_angle'] = 0
                                writer.writerow([data[k] for k in data_log_columns])
                                time.sleep(0.1)
                                print(data['angular_error_degrees'])
                                print(u)

                            # Step 2: step to rotated vector
                            t0 = time.time()
                            controller.reset()
                            while time.time() - t0 < seconds_per_target:
                                m_current = comm.get_magnetic_field()
                                u, data = controller.compute_control_currents(m_current=m_current, m_target=rotated_vector)
                                comm.set_currents_in_coils(u)
                                data['time'] = time.time() - start_time
                                data['target_angle'] = angle_deg
                                writer.writerow([data[k] for k in data_log_columns])
                                time.sleep(0.1)
                                print(data['angular_error_degrees'])
                                print(u)

            print("Tests completed.")
            comm.set_currents_in_coils([0, 0, 0])
            return 0

    except KeyboardInterrupt:
        print("\n[✓] Interrupted by user.")
    finally:
        comm.shutdown()
        print("[✓] Arduino and MiniMACS communications shutdown complete.")


if __name__ == "__main__":
    main()
