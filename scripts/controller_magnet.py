"""
Python script that defines different magnet controllers
to implement and test multiple controllers for the magnet
"""
import numpy as np
from time import time
from abc import ABC, abstractmethod
from test_params import MAGNET_CONTROLLER_PARAMS

class MagnetController(ABC):
    def __init__(self):
        super().__init__()
        self.i_nominal = MAGNET_CONTROLLER_PARAMS["currents"]["I_nominal"]
        self.i_hold = MAGNET_CONTROLLER_PARAMS["currents"]["I_hold"]
        self.i_max = MAGNET_CONTROLLER_PARAMS["currents"]["I_max"]
        self.i_coefficients = MAGNET_CONTROLLER_PARAMS["currents"]["I_coefficients"]
        default_current_dirs = MAGNET_CONTROLLER_PARAMS["currents"]
        self.current_dirs_in_coils = np.array(
            [
                default_current_dirs["current_direction_in_coils"]["coil_x"],
                default_current_dirs["current_direction_in_coils"]["coil_y"],
                default_current_dirs["current_direction_in_coils"]["coil_z"],
            ]
        )
        self.m_hat = None
        self.m_mag = None
        self.m_target = None

    @abstractmethod
    def compute_control_currents(self, m_current, m_target):
        pass

    def get_target_magnet_direction(self):
        return self.m_target

    def set_target_magnet_direction(self, vec):
        m = vec.copy() 
        m_mag = np.linalg.norm(m)
        assert m_mag > 1e-8, "Target direction should be a non-zero vector!"

        self.m_target = m/m_mag

    def set_current_magnet_direction(self, vec):
        m = vec.copy() 
        m_mag = np.linalg.norm(m)
        assert m_mag > 1e-8, "Target direction should be a non-zero vector!"
        
        self.m_mag = m_mag
        self.m_hat = m/m_mag

    def get_error_vector(self):
        return np.cross(self.m_hat, self.m_target)

    def get_error_angles_in_degrees(self):
        cos_theta = np.dot(self.m_hat, self.m_target)
        err = self.get_error_vector()
        sin_theta = np.linalg.norm(err)
        # sgn = 1 if err[0] > 1 else -1
        return np.rad2deg(np.arctan2(sin_theta, cos_theta))
    
    def clip_input_currents(self, u):
        if np.any(np.abs(u) > 1e3 * self.i_max):
            output_currents_values_mA = u / np.linalg.norm(u) * 1e3 * self.i_max
        else:
            output_currents_values_mA = u.copy()
        output_currents_values_mA = np.round(np.diag(self.current_dirs_in_coils) @  output_currents_values_mA)
        return output_currents_values_mA        

class MagnetControllerStatic(MagnetController):
    def __init__(self):
        super().__init__()

    def compute_control_currents(self, m_current, m_target):
        self.set_current_magnet_direction(m_current)
        self.set_target_magnet_direction(m_target)
        u = 1e3 * self.i_nominal * self.m_target
        u = self.clip_input_currents(u)
        data = {
                'time': 0.0,
                'm_x': self.m_hat[0], 
                'm_y': self.m_hat[1], 
                'm_z': self.m_hat[2], 
                'm_target_x': self.m_target[0],
                'm_target_y': self.m_target[1],
                'm_target_z': self.m_target[2],
                'angular_error_degrees': self.get_error_angles_in_degrees(),
                'p_term_mag': 0, 
                'i_term_mag': 0, 
                'd_term_mag': 0, 
                'input_torque_mag': 0, 
                'KP_x': 0, 
                'KI_x': 0, 
                'KD_x': 0, 
                'KP_y': 0, 
                'KI_y': 0, 
                'KD_y': 0, 
                'KP_z': 0, 
                'KI_z': 0, 
                'KD_z': 0, 
                'i_x': u[0], 'i_y': u[1], 'i_z': u[2]
            }
        
        return u, data
    
    def reset(self):
        pass

class MagnetControllerPID(MagnetController):
    def __init__(self):
        super().__init__()
        pid_params = MAGNET_CONTROLLER_PARAMS["pid_params"]

        self.kp = pid_params["kp"] * np.eye(3)
        self.ki = pid_params["ki"] * np.eye(3)
        self.kd = pid_params["kd"] * np.eye(3)
        self.sat_e = pid_params["saturation_integral_term"]
        self.sat_v = pid_params["saturation_derivative_term"]
        self.lmbd = pid_params["lambda"]

        self.t0 = None
        self.e_prev = None
        self.q_e_prev = None
        self.t0_target = time()

        self.feedforward_scale = 0.0

    def set_pid_gains(self, gains):
        self.kp = np.diag([gains["KP_x"], gains["KP_y"], gains["KP_z"]])
        self.ki = np.diag([gains["KI_x"], gains["KI_y"], gains["KI_z"]])
        self.kd = np.diag([gains["KD_x"], gains["KD_y"], gains["KD_z"]])
        self.lmbd = gains["Lambda"]

    def saturation_v(self, z):
        z_norm = np.linalg.norm(z)
        return z if z_norm <= self.sat_v else (z / z_norm) * self.sat_v

    def update_integral(self, delta_e):
        candidate = self.q_e_prev + delta_e
        if np.linalg.norm(candidate) <= self.sat_e:
            return candidate
        else:
            return self.q_e_prev.copy()

    def compute_pid(self, m_current, m_target):
        proportional_term = np.zeros(3)
        integral_term = np.zeros(3)
        derivative_term = np.zeros(3)

        self.set_current_magnet_direction(m_current)
        self.set_target_magnet_direction(m_target)

        proportional_term = self.get_error_vector()

        if self.t0 is not None:
            delta_t = time() - self.t0
            integral_term = self.update_integral(delta_t * proportional_term)
            derivative_term = self.saturation_v((1/delta_t) * (proportional_term - self.e_prev))

        self.t0 = time()
        self.q_e_prev = integral_term
        self.e_prev = proportional_term
        return proportional_term, integral_term, derivative_term
    
    def compute_control_currents(self, m_current, m_target):
        p, i, d = self.compute_pid(m_current, m_target)
        err = self.get_error_angles_in_degrees()
        tau = self.kp @ p + self.ki @ i + self.kd @ d 
        u = np.cross(tau, self.m_hat) + self.lmbd * self.m_hat + (err*self.feedforward_scale/180.0) * self.m_target
        u = self.clip_input_currents(u)
    
        data = {
                'time': 0.0,
                'm_x': self.m_hat[0], 
                'm_y': self.m_hat[1], 
                'm_z': self.m_hat[2], 
                'm_target_x': self.m_target[0],
                'm_target_y': self.m_target[1],
                'm_target_z': self.m_target[2],
                'angular_error_degrees': err,
                'p_term_mag': np.linalg.norm(p),
                'i_term_mag': np.linalg.norm(i),
                'd_term_mag': np.linalg.norm(d),
                'input_torque_mag': np.linalg.norm(tau),
                'KP_x': self.kp[0, 0],
                'KI_x': self.ki[0, 0],
                'KD_x': self.kd[0, 0],
                'KP_y': self.kp[1, 1],
                'KI_y': self.ki[1, 1],
                'KD_y': self.kd[1, 1],
                'KP_z': self.kp[2, 2],
                'KI_z': self.ki[2, 2],
                'KD_z': self.kd[2, 2],
                'i_x': u[0], 'i_y': u[1], 'i_z': u[2]
            }

        return u, data
    
    def reset(self):
        self.t0 = None
        self.e_prev = np.zeros(3)
        self.q_e_prev = np.zeros(3)
        self.t0_target = time()
