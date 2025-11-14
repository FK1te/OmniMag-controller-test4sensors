"""
Performs grid search to tune closed-loop controller parameters:
- Integral term clamping threshold (sat_e)
- Derivative term saturation limit (sat_v)
- Feedforward scale

This script evaluates controller performance using multiple error metrics
(IAE, ISE, ITAE, ITSE) across a set of predefined unit vectors.

Important:
- Run only after manually tuning base gains
- Use to fine-tune saturation and feedforward parameters
"""

import time
import numpy as np
import pandas as pd
from sklearn.model_selection import ParameterGrid

import sys
sys.path.append('./')
sys.path.append('./scripts')

from scripts.communication import ArduinoMinimacsCommunication
from scripts.controller_magnet import MagnetControllerClosedLoop

# Initialize hardware interface
comm = ArduinoMinimacsCommunication()
comm.change_status_enable_disable_current(True)

# Base gain values — assumed to be manually tuned beforehand
base_gains = {
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

controller = MagnetControllerClosedLoop()
controller.set_closed_loop_gains(gains=base_gains)

# Allow system to stabilize
for _ in range(5):
    comm.get_sensor_readings()
    time.sleep(0.25)

def objective(sat_e, sat_v, ff_term):
    """
    Evaluate closed-loop performance with given saturation & feedforward parameters.
    """
    controller.sat_e = sat_e
    controller.sat_v = sat_v
    controller.feedforward_scale = ff_term

    iae = 0
    ise = 0
    itae = 0
    itse = 0

    vectors = [
        np.array([0.9961947, 0.0, -0.0871557]),
        np.array([0.9659258, 0.0, -0.2588190]),
        np.array([0.7071068, 0.0, -0.7071068]),
        np.array([0.0, 0.0, -1.0]),
        np.array([-1.0, 0.0, 0.0]),
        np.array([0.8660254, 0.0, 0.5]),
        np.array([-1.0, 0.0, 0.0]),
        np.array([-0.7071068, 0.0, -0.7071068]),
        np.array([-0.2588190, 0.0, -0.9659258]),
        np.array([-0.0871557, 0.0, -0.9961947]),
        np.array([0.0, 0.0, -1.0]),
        np.array([0.0, 0.0871557, -0.9961947]),
        np.array([0.0, 0.2588190, -0.9659258]),
        np.array([0.0, 0.7071068, -0.7071068]),
        np.array([0.0, 1.0, 0.0]),
        np.array([0.0, 0.0, 1.0]),
        np.array([0.0, -0.5, -0.8660254]),
        np.array([0.0, 0.0, 1.0]),
        np.array([0.0, 0.7071068, 0.7071068]),
        np.array([0.0, 0.9659258, 0.2588190]),
        np.array([0.0, 0.9961947, 0.0871557]),
        np.array([0.0, 1.0, 0.0]),
        np.array([-0.0871557, 0.9961947, 0.0]),
        np.array([-0.2588190, 0.9659258, 0.0]),
        np.array([-0.7071068, 0.7071068, 0.0]),
        np.array([-1.0, 0.0, 0.0]),
        np.array([0.0, -1.0, 0.0]),
        np.array([0.5, 0.8660254, 0.0]),
        np.array([0.0, -1.0, 0.0]),
        np.array([-0.7071068, -0.7071068, 0.0]),
        np.array([-0.9659258, -0.2588190, 0.0]),
        np.array([-0.9961947, -0.0871557, 0.0]),
        np.array([-1.0, 0.0, 0.0]),
    ]

    # Apply initial current to stabilize sensors
    comm.set_currents_in_coils(np.array([1, 0, 0]) * 2000)
    time.sleep(0.5)

    for vec in vectors:
        controller.reset()
        t_start = time.time()
        t_last = t_start
        while (time.time() - t_start) < 2.0:
            m_current = comm.get_magnetic_field()
            u, data = controller.compute_control_currents(m_current=m_current, m_target=vec)
            comm.set_currents_in_coils(u)

            err_deg = data['angular_error_degrees']
            now = time.time()
            dt = now - t_last
            elapsed = now - t_start

            # Error metrics
            err_rad = np.deg2rad(err_deg)
            iae += err_rad * dt
            ise += err_deg ** 2 * dt * 0.00030461768
            itae += err_rad * dt * elapsed
            itse += err_deg ** 2 * dt * 0.00030461768 * elapsed

            t_last = now

    return {
        'iae': iae,
        'ise': ise,
        'itae': itae,
        'itse': itse
    }

# Grid search parameter space
for ff in [[2000.0], [1000.0]]:
    comm.set_currents_in_coils([0, 0, 0])
    time.sleep(5)

    print(f"Feedforward: {ff[0]}")
    filename = f'25_07_2025_grid_search_fine_manually_tuned_gains_2_{ff[0]}.csv'
    print(filename)

    param_grid = {
        "sat_e": np.linspace(0.025, 0.075, 11).tolist(),
        "sat_v": np.linspace(0, 8, 11).tolist(),
        "ff_term": ff
    }

    results = []
    for idx, params in enumerate(ParameterGrid(param_grid), 1):
        res = objective(**params)
        print(f"{idx}. {params} => ITAE: {res['itae']:.4f}")
        res.update(**params)
        results.append(res)

    df = pd.DataFrame(results)
    df.to_csv(filename, index=False)

    comm.set_currents_in_coils([0, 0, 0])
    time.sleep(5)

comm.shutdown()
