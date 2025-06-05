import os
import sys
import time
import numpy as np
import pandas as pd

sys.path.append('./')
sys.path.append('./scripts')

from scripts.communication import ArduinoMinimacsCommunication

# --- Simulation settings ---
duration = 20  # seconds
dt = 0.1        # time step
time_vector = np.arange(0, duration, dt)

# --- Signal bounds ---
U_MAX = 2000
U_MIN = -2000

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
    num_switches = int(len(t) / (switch_interval / dt))
    values = amplitude * (2 * np.random.randint(0, 2, num_switches) - 1)
    signal = np.repeat(values, int(switch_interval / dt))
    return signal[:len(t)]

def main():
    comm = ArduinoMinimacsCommunication()
    comm.change_status_enable_disable_current(True)

    os.makedirs("rotation_logs", exist_ok=True)

    # --- Create test cases ---
    test_cases = [
        {
            'name': 'TC1_sine_u1',
            'u1': sine_signal(time_vector, 1500, 0.05),
            'u2': np.zeros_like(time_vector),
            'u3': np.zeros_like(time_vector)
        },
        {
            'name': 'TC2_sine_u2',
            'u1': np.zeros_like(time_vector),
            'u2': sine_signal(time_vector, 1500, 0.05),
            'u3': np.zeros_like(time_vector)
        },
        {
            'name': 'TC3_sine_u3',
            'u1': np.zeros_like(time_vector),
            'u2': np.zeros_like(time_vector),
            'u3': sine_signal(time_vector, 1500, 0.05)
        },
        {
            'name': 'TC4_ramp_all',
            'u1': saturate(ramp_signal(time_vector, 30)),
            'u2': saturate(ramp_signal(time_vector, -20)),
            'u3': saturate(ramp_signal(time_vector, 10))
        },
        {
            'name': 'TC5_prbs_all',
            'u1': prbs_signal(time_vector, 1500, 5),
            'u2': prbs_signal(time_vector, 1500, 7),
            'u3': prbs_signal(time_vector, 1500, 9)
        }
    ]

    try:
        for case in test_cases:
            print(f"\n[•] Running {case['name']}...")
            log = []
            start_time = time.monotonic()

            for i, (u1, u2, u3) in enumerate(zip(case['u1'], case['u2'], case['u3'])):
                t_now = time.monotonic() - start_time
                u = [u1, u2, u3]
                comm.set_currents_in_coils(u)
                reading = comm.get_magnetic_field()

                log.append({
                    't': round(t_now, 3),
                    'u1': u1,
                    'u2': u2,
                    'u3': u3,
                    'reading_x': reading[0],
                    'reading_y': reading[1],
                    'reading_z': reading[2]
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
