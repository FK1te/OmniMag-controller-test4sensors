
import os
import sys
import csv
import time
import numpy as np
import pandas as pd

import threading

sys.path.append('./')
sys.path.append('./scripts')

from scripts.communication import ArduinoMinimacsCommunication
from scripts.controller_magnet import MagnetControllerStatic, MagnetControllerPID

speed = 1
controller_name = f'{speed}x_static'

dt = 1.0

total_trajectory_time = 240

gains = {
    'KP_x': 825,
    'KI_x': 5775,
    'KD_x': 77.8,

    'KP_y': 1278,
    'KI_y': 7672,
    'KD_y': 140,

    'KP_z': 577.5,
    'KI_z': 4908.8,
    'KD_z': 44.8,

    'Lambda': 0.0
}

sat_v = 0.9
sat_e = 0.03
feedforward_scale = 2000.0

comm = ArduinoMinimacsCommunication()
comm.change_status_enable_disable_current(True)


if "static" in controller_name.lower():
    controller = MagnetControllerStatic()
elif "pid" in controller_name.lower():
    controller = MagnetControllerPID()
    controller.set_pid_gains(gains=gains)
    controller.sat_v = sat_v 
    controller.sat_e = sat_e
    controller.feedforward_scale = feedforward_scale
else:
    raise ValueError("Invalid controller name.")

df = pd.read_csv(os.path.join(os.getcwd(), 'reference_trajectory.csv'))

m_target = df.loc[0, ["m_target_x", "m_target_y", "m_target_z"]].to_numpy()

t0 = time.time()

error = []

t = time.time() - t0

target_reset = True

index = 0

log_data = [] 

def update_target():
    global m_target, t, t0, target_reset, index
    while t < total_trajectory_time/speed:
        t = time.time() - t0
        closest_idx = (df['time'] - speed * t).abs().idxmin()
        m_target = df.loc[closest_idx, ["m_target_x", "m_target_y", "m_target_z"]].to_numpy()
        # print("Updated Target")
        target_reset = True
        time.sleep(dt)

def follow_trajectory():
    global m_target, comm, controller, error, t, target_reset, log_data
    while t < total_trajectory_time/speed:
        if target_reset:
            controller.reset()
            target_reset = False
        m_current = comm.get_magnetic_field()
        u, data = controller.compute_control_currents(m_current=m_current, m_target=m_target)
        comm.set_currents_in_coils(u)
        error.append(data['angular_error_degrees'])
        # print(data['angular_error_degrees'])

        log_data.append({
            't' : t,
            'm_target_x': m_target[0], 
            'm_target_y': m_target[1], 
            'm_target_z': m_target[2], 
            'm_target_x': data['m_x'], 
            'm_target_y': data['m_y'], 
            'm_target_z': data['m_z'], 
            'error': data['angular_error_degrees']
        })
        time.sleep(0.001)

def main():
    global log_data, dt
    try:
        thread1 = threading.Thread(target=update_target)
        thread2 = threading.Thread(target=follow_trajectory)

        thread1.start()
        thread2.start()

    except KeyboardInterrupt:
        print("\n[✓] Interrupted by user.")
    finally:
        thread1.join()
        thread2.join()
        comm.shutdown()
        print("[✓] Arduino and Minimacs communications shutdown complete.")
        rmse = np.sqrt(((np.array(error)) ** 2).mean())
        mean_error = np.mean(np.array(error))
        print(f'{controller_name}_dt_{dt}')
        print(f"[✓] RMSE: {rmse} degrees.")
        print(f"[✓] Mean error: {mean_error} degrees.")
        df_log = pd.DataFrame(log_data)
        df_log.to_csv(os.path.join(os.getcwd(), f'{controller_name}_dt_{dt}_avg_error_{mean_error}.csv'))

if __name__ == "__main__":
    main()
