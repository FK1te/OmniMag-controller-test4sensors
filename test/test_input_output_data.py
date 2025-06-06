import os
import sys
import time
import numpy as np
import pandas as pd

sys.path.append('./')
sys.path.append('./scripts')

from scripts.communication import ArduinoMinimacsCommunication

# --- Simulation settings ---
duration = 200  # seconds
dt = 0.1        # time step
time_vector = np.arange(0, duration, dt)

# --- Signal bounds ---
U_MAX = 1000
U_MIN = -1000

def saturate(signal):
    return np.clip(signal, U_MIN, U_MAX)

# --- Signal generators ---
def step_signal(t, value=2000, start_time=10):
    return np.where(t >= start_time, value, 0)

def sine_signal(t, amplitude=1000, freq=0.1):
    return amplitude * np.sin(2 * np.pi * freq * t)

def ramp_signal(t, slope=20):
    return slope * t

def prbs_signal(t, amplitude=2000, switch_interval=10):
    np.random.seed(0)  # For reproducibility
    num_points = len(t)
    block_size = int(switch_interval / dt)
    num_blocks = int(np.ceil(num_points / block_size))
    values = amplitude * (2 * np.random.randint(0, 2, num_blocks) - 1)
    signal = np.repeat(values, block_size)[:num_points]
    return signal

def main():
    comm = ArduinoMinimacsCommunication()
    comm.change_status_enable_disable_current(True)

    os.makedirs("rotation_logs", exist_ok=True)

    # --- Create test cases ---
    test_cases = [
        {
            'name': 'TC1_sine_u1',
            'u1': sine_signal(time_vector, 1500, 0.01),
            'u2': np.zeros_like(time_vector),
            'u3': np.zeros_like(time_vector)
        },
        {
            'name': 'TC2_sine_u2',
            'u1': np.zeros_like(time_vector),
            'u2': sine_signal(time_vector, 1500, 0.01),
            'u3': np.zeros_like(time_vector)
        },
        {
            'name': 'TC3_sine_u3',
            'u1': np.zeros_like(time_vector),
            'u2': np.zeros_like(time_vector),
            'u3': sine_signal(time_vector, 1500, 0.01)
        },
        {
            'name': 'TC4_ramp_all',
            'u1': saturate(ramp_signal(time_vector, 30)),
            'u2': saturate(ramp_signal(time_vector, -20)),
            'u3': saturate(ramp_signal(time_vector, 10))
        },
        {
            'name': 'TC5_ramp_all',
            'u1': saturate(ramp_signal(time_vector, -20)),
            'u2': saturate(ramp_signal(time_vector, 10)),
            'u3': saturate(ramp_signal(time_vector, -10))
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

                time.sleep(dt)

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
