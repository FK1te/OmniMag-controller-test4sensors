import os
import sys
import time
import numpy as np
import pandas as pd

sys.path.append('./')
sys.path.append('./scripts')

from scripts.communication import ArduinoMinimacsCommunication

# --- Simulation settings ---
duration = 240  # seconds
dt = 0.1        # time step
time_vector = np.arange(0, duration, dt)

# --- Signal bounds ---
positive_segment = 1 * np.ones(int(duration/(2*dt)))
negative_segment = -1 * np.ones(int(duration/(2*dt)))
positive_negative_step = np.concatenate((positive_segment, negative_segment))


def main():
    comm = ArduinoMinimacsCommunication()
    comm.change_status_enable_disable_current(True)

    os.makedirs("rotation_logs", exist_ok=True)

    # --- Create test cases ---
    test_cases = [
        {
            'name': 'TC6_unit_step_u1',
            'u1': positive_negative_step,
            'u2': np.zeros_like(positive_negative_step),
            'u3': np.zeros_like(positive_negative_step)
        },
        {
            'name': 'TC7_unit_step_u2',
            'u1': np.zeros_like(positive_negative_step),
            'u2': positive_negative_step,
            'u3': np.zeros_like(positive_negative_step),
        },
        {
            'name': 'TC8_unit_step_u3',
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
                    'm23': s_vec[5]
                })

                time.sleep(0.01)

            df = pd.DataFrame(log)
            filename = f"parameter_estimation_logs/{case['name']}.csv"
            df.to_csv(filename, index=False)
            print(f"[✓] Log saved to {filename}")

    except KeyboardInterrupt:
        print("\n[✓] Interrupted by user.")
    finally:
        comm.shutdown()
        print("[✓] Arduino and Minimacs communications shutdown complete.")

if __name__ == "__main__":
    main()
