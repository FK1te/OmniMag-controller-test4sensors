
import os
import sys
import csv
import time
import numpy as np
import pandas as pd

sys.path.append('./')
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


def main():
    comm = ArduinoMinimacsCommunication()
    comm.change_status_enable_disable_current(True)

    for index in range(5):
        print(comm.get_sensor_readings())

    if controller_name == "static":
        controller = MagnetControllerStatic()
    elif "pid" in controller_name.lower():
        global gains
        controller = MagnetControllerPID()
        controller.set_pid_gains(gains=gains)
    else:
        raise ValueError("Invalid controller name.")
    
    flag = False

    df = pd.read_csv(os.path.join(os.getcwd(), 'reference_trajectory.csv'))

    t0 = time.time()

    m_target_prev = None

    error = []

    try:
        while True:
            t = time.time() - t0
            if t > 300.0:
                print("Tests ended")
                continue

            closest_idx = (df['col1'] - t).abs().idxmin()
            m_target = df.loc[closest_idx, ["m_target_x", "m_target_y", "m_target_z"]].to_numpy()
            if m_target_prev is None or not np.array_equal(m_target, m_target_prev):
                controller.reset()
                m_target_prev = m_target

            m_current = comm.get_magnetic_field()
            u, data = controller.compute_control_currents(m_current=m_current, m_target=m_target)
            comm.set_currents_in_coils(u)
            error.append(data['angular_error_degrees'])

    except KeyboardInterrupt:
        print("\n[✓] Interrupted by user.")
    finally:
        comm.shutdown()
        print("[✓] Arduino and Minimacs communications shutdown complete.")
        rmse = np.sqrt(((np.array(error)) ** 2).mean())
        print(f"[✓] RMSE: {rmse} degrees.")

if __name__ == "__main__":
    main()
