import os
import sys
import csv
import time
import numpy as np

import pandas as pd

sys.path.append('./')
sys.path.append('./scripts')

from scripts.communication import ArduinoMinimacsCommunication
from scripts.controller_magnet import MagnetControllerClosedLoop

from sklearn.model_selection import ParameterGrid
    
comm = ArduinoMinimacsCommunication()
comm.change_status_enable_disable_current(True)

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

controller = MagnetControllerClosedLoop()
controller.set_pid_gains(gains=gains)

for index in range(5):
    comm.get_sensor_readings()
    time.sleep(0.25)
    

def objective(sat_e, sat_v, ff_term):
    global gains, controller, comm

    controller.sat_e = sat_e
    controller.sat_v = sat_v

    controller.feedforward_scale = ff_term

    iae = 0
    ise = 0
    itae = 0
    itse = 0
    iltae = 0
    iltse = 0

    vectors = [
        np.array([0.9961946980917455, 0.0, -0.08715574274765817]),
        np.array([0.9659258262890682, 0.0, -0.25881904510252074]),
        np.array([0.7071067811865476, 0.0, -0.7071067811865475]),
        np.array([1.2116883882008518e-16, 0.0, -1.0]),
        np.array([-1.0, 0.0, -1.8240117877745285e-16]),
        np.array([0.8660254037844386, 0.0, 0.5000000000000001]),
        np.array([-1.0, 0.0, -2.0717043678169387e-16]),
        np.array([-0.7071067811865475, 0.0, -0.7071067811865477]),
        np.array([-0.2588190451025207, 0.0, -0.9659258262890684]),
        np.array([-0.08715574274765807, 0.0, -0.9961946980917457]),
        np.array([1.0911896050930042e-16, 0.0, -1.0000000000000002]),
        np.array([1.0911896050930042e-16, 0.08715574274765818, -0.9961946980917458]),
        np.array([1.0911896050930042e-16, 0.2588190451025208, -0.9659258262890685]),
        np.array([1.0911896050930042e-16, 0.7071067811865477, -0.7071067811865478]),
        np.array([1.0911896050930042e-16, 1.0000000000000004, -1.143450299824811e-16]),
        np.array([1.0911896050930042e-16, 1.7557736993984878e-16, 1.0000000000000004]),
        np.array([1.0911896050930042e-16, -0.5000000000000003, -0.866025403784439]),
        np.array([1.0911896050930042e-16, 1.5909636962210479e-16, 1.0000000000000004]),
        np.array([1.0911896050930042e-16, 0.707106781186548, 0.7071067811865478]),
        np.array([1.0911896050930042e-16, 0.9659258262890689, 0.25881904510252085]),
        np.array([1.0911896050930042e-16, 0.996194698091746, 0.08715574274765818]),
        np.array([1.0911896050930042e-16, 1.0000000000000004, -2.551017016613283e-17]),
        np.array([-0.0871557427476581, 0.996194698091746, -2.551017016613283e-17]),
        np.array([-0.25881904510252074, 0.9659258262890688, -2.551017016613283e-17]),
        np.array([-0.7071067811865477, 0.707106781186548, -2.551017016613283e-17]),
        np.array([-1.0000000000000004, 1.8620419787673897e-16, -2.551017016613283e-17]),
        np.array([-2.4743653783410667e-16, -1.0000000000000004, -2.551017016613283e-17]),
        np.array([0.5000000000000003, 0.8660254037844389, -2.551017016613283e-17]),
        np.array([-2.3293325172627677e-16, -1.0000000000000004, -2.551017016613283e-17]),
        np.array([-0.707106781186548, -0.7071067811865478, -2.551017016613283e-17]),
        np.array([-0.9659258262890689, -0.25881904510252085, -2.551017016613283e-17]),
        np.array([-0.9961946980917461, -0.08715574274765815, -2.551017016613283e-17]),
        np.array([-1.0000000000000007, 6.455715859311652e-17, -2.551017016613283e-17]),
    ]

    vec0 = np.array([1, 0, 0])

    u0 = vec0 * 2000
    comm.set_currents_in_coils(u0)
    time.sleep(0.5)

    for vec in vectors:
        t0 = time.time()
        current_time = time.time() - t0
        controller.reset()
        t1 = time.time()
        while current_time < 2.0:
            m_current = comm.get_magnetic_field()
            u, data = controller.compute_control_currents(m_current=m_current, m_target=vec)
            comm.set_currents_in_coils(u)
            err = data['angular_error_degrees']
            current_time = time.time() - t0
            del_t = time.time() - t1
            iae += np.deg2rad(err) * del_t
            ise += err * err * del_t * 0.00030461768
            itae += np.deg2rad(err) * del_t * current_time 
            itse += err * err * del_t * 0.00030461768 * current_time 
            t1 = time.time()

    return {
        'iae' : iae,
        'ise' : ise,
        'itae' : itae,
        'itse' : itse,
        'iltae' : iltae,
        'iltse' : iltse
    }

"""
"sat_e": np.logspace(-3, 0.1962, 14).tolist(),
"sat_v": np.logspace(-2, 1.0, 14).tolist(),
"""

for ff in [[2000.0], [1000.0]]:
    comm.set_currents_in_coils([0, 0, 0])
    time.sleep(5)
    print(ff)
    print(f'25_07_2025_grid_search_fine_manually_tuned_gains_2_{ff[0]}.csv')
    param_grid = {
    "sat_e": np.linspace(0.025, 0.075, 11).tolist(), 
    "sat_v": np.linspace(0, 8, 11).tolist(), 
    "ff_term" : ff
    }

    results = []
    initial_time = time.time()
    iter = 1
    for params in ParameterGrid(param_grid):
        res = objective(**params)
        print(params)
        print(f'{iter}. iteration: {res["itae"]}')
        res.update(**params)
        results.append(res)
        iter += 1

    df_results = pd.DataFrame(results)
    # df_results.to_csv('23_07_2025_grid_search_finer_some_overshoot.csv', index=False)
    df_results.to_csv(f'25_07_2025_grid_search_fine_manually_tuned_gains_2_{ff[0]}.csv', index=False)
    comm.set_currents_in_coils([0, 0, 0])
    time.sleep(5)

comm.shutdown()
