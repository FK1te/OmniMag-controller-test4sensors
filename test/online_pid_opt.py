import os
import sys
import csv
import time
import numpy as np

import pandas as pd

sys.path.append('./')
sys.path.append('./scripts')

from scripts.communication import ArduinoMinimacsCommunication
from scripts.controller_magnet import MagnetControllerPID

import pickle
from hyperopt import fmin, tpe, rand, hp, Trials
    
comm = ArduinoMinimacsCommunication()
comm.change_status_enable_disable_current(True)

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

controller = MagnetControllerPID()
controller.set_pid_gains(gains=gains)

for index in range(5):
    comm.get_sensor_readings()
    time.sleep(0.25)

def objective(params):
    global gains, controller, comm, gains

    sat_e = params['sat_e']
    sat_v = params['sat_v']
    feedforward_scale = params['feedforward_scale']

    controller.feedforward_scale = feedforward_scale
    controller.sat_e = sat_e
    controller.sat_v = sat_v

    cost = 0

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
            current_time = time.time() - t0
            del_t = time.time() - t1
            cost += np.deg2rad(data['angular_error_degrees']) * del_t * current_time 
            t1 = time.time()

    return cost

space = {
    'sat_e' : hp.quniform('sat_e', 0.0, 0.05, 0.0005),
    'sat_v' : hp.quniform('sat_v', 0.0, 3.0, 0.03),
    'feedforward_scale': hp.quniform('feedforward_scale', 0.0, 3000.0, 30.0)
}

trials = Trials()

"""
trials.insert_trial_doc(
{
    'state': 2,
    'tid': 0,
    'exp_key': None,
    'owner': None,
    'book_time': None,
    'refresh_time': None,
    'spec': None,
    'result': {'loss': 9.366633, 'status': STATUS_OK},  
    'misc': {
        'tid': 0,
        'cmd': ('domain_attachment', 'try_everything'),
        'workdir': None,
        'idxs': {'sat_e': [0], 'sat_v': [0], 'i_scale': [0], 'feedforward_scale': [0]},
        'vals': {'sat_e': [0.0], 'sat_v': [6.0], 'i_scale': [0.1], 'feedforward_scale': [3000.0]},  # Example parameter values
        'err': None
    },
    'version': 0
}
)"""

best = fmin(
    fn=objective,
    space=space,
    algo=tpe.suggest,
    max_evals=64,
    trials=trials
)

log_file_name = os.path.join(os.getcwd(), 'controller_parameter_search_tpe.p')
pickle.dump(trials, open(log_file_name, "wb"))
print(f'Optimal gains and saturation limits : {best}')

comm.shutdown()

