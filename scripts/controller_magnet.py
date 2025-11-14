"""
Defines abstract magnet controller and two implementations:
- Open-loop controller: feeds fixed directional current
- Closed-loop controller: uses feedback (P/I/D terms) to steer magnetic direction
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

        dirs = MAGNET_CONTROLLER_PARAMS["currents"]["current_direction_in_coils"]
        self.current_dirs_in_coils = np.array([dirs["coil_x"], dirs["coil_y"], dirs["coil_z"]])

        self.m_hat = None          # unit vector of current magnet direction
        self.m_mag = None          # magnitude of current magnet vector
        self.m_target = None       # unit vector of desired magnet direction

    @abstractmethod
    def compute_control_currents(self, m_current, m_target):
        pass

    def get_target_magnet_direction(self):
        return self.m_target

    def set_target_magnet_direction(self, vec):
        m = vec.copy()
        m_mag = np.linalg.norm(m)
        assert m_mag > 1e-8, "Target direction should be a non-zero vector!"
        self.m_target = m / m_mag

    def set_current_magnet_direction(self, vec):
        m = vec.copy()
        m_mag = np.linalg.norm(m)
        assert m_mag > 1e-8, "Current direction should be a non-zero vector!"
        self.m_mag = m_mag
        self.m_hat = m / m_mag

    def get_error_vector(self):
        return np.cross(self.m_hat, self.m_target)

    def get_error_angles_in_degrees(self):
        cos_theta = np.dot(self.m_hat, self.m_target)
        err = self.get_error_vector()
        sin_theta = np.linalg.norm(err)
        return np.rad2deg(np.arctan2(sin_theta, cos_theta))

    def clip_input_currents(self, u):
        """
        Clips the currents if they exceed physical maximum and applies coil direction mapping.
        """
        if np.any(np.abs(u) > 1e3 * self.i_max):
            u = u / np.linalg.norm(u) * 1e3 * self.i_max
        return np.round(np.diag(self.current_dirs_in_coils) @ u)


class MagnetControllerOpenLoop(MagnetController):
    def __init__(self):
        super().__init__()

    def compute_control_currents(self, m_current, m_target):
        self.set_current_magnet_direction(m_current)
        self.set_target_magnet_direction(m_target)

        u = 1e3 * self.i_nominal * self.m_target
        u[0] *= self.i_coefficients[0]
        u[1] *= self.i_coefficients[1]
        u[2] *= self.i_coefficients[2]
        u = self.clip_input_currents(u)

        data = {
            'time': 0.0,
            'm_x': self.m_hat[0], 'm_y': self.m_hat[1], 'm_z': self.m_hat[2],
            'm_target_x': self.m_target[0], 'm_target_y': self.m_target[1], 'm_target_z': self.m_target[2],
            'angular_error_degrees': self.get_error_angles_in_degrees(),
            'p_term_mag': 0, 'i_term_mag': 0, 'd_term_mag': 0,
            'input_torque_mag': 0,
            'KP_x': 0, 'KI_x': 0, 'KD_x': 0,
            'KP_y': 0, 'KI_y': 0, 'KD_y': 0,
            'KP_z': 0, 'KI_z': 0, 'KD_z': 0,
            'i_x': u[0], 'i_y': u[1], 'i_z': u[2]
        }

        return u, data

    def reset(self):
        pass


class MagnetControllerClosedLoop(MagnetController):
    def __init__(self):
        super().__init__()
        ctrl_params = MAGNET_CONTROLLER_PARAMS["closed_loop_params"]

        self.kp = ctrl_params["kp"] * np.eye(3)
        self.ki = ctrl_params["ki"] * np.eye(3)
        self.kd = ctrl_params["kd"] * np.eye(3)
        self.sat_e = ctrl_params["saturation_integral_term"]
        self.sat_v = ctrl_params["saturation_derivative_term"]
        self.lmbd = ctrl_params["lambda"]

        self.t0 = None
        self.e_prev = None
        self.q_e_prev = None
        self.t0_target = time()

        self.feedforward_scale = 0.0

    def set_closed_loop_gains(self, gains):
        self.kp = np.diag([gains["KP_x"], gains["KP_y"], gains["KP_z"]])
        self.ki = np.diag([gains["KI_x"], gains["KI_y"], gains["KI_z"]])
        self.kd = np.diag([gains["KD_x"], gains["KD_y"], gains["KD_z"]])
        self.lmbd = gains["Lambda"]

    def saturation_v(self, z):
        z_norm = np.linalg.norm(z)
        return z if z_norm <= self.sat_v else (z / z_norm) * self.sat_v

    def update_integral(self, delta_e):
        candidate = self.q_e_prev + delta_e
        return candidate if np.linalg.norm(candidate) <= self.sat_e else self.q_e_prev.copy()

    def compute_feedback_terms(self, m_current, m_target):
        """
        Computes proportional, integral, and derivative terms of the closed-loop control.
        """
        self.set_current_magnet_direction(m_current)
        self.set_target_magnet_direction(m_target)

        p_term = self.get_error_vector()
        i_term = np.zeros(3)
        d_term = np.zeros(3)

        if self.t0 is not None:
            delta_t = time() - self.t0
            i_term = self.update_integral(delta_t * p_term)
            d_term = self.saturation_v((1 / delta_t) * (p_term - self.e_prev))

        self.t0 = time()
        self.q_e_prev = i_term
        self.e_prev = p_term

        return p_term, i_term, d_term

    def compute_control_currents(self, m_current, m_target):
        p, i, d = self.compute_feedback_terms(m_current, m_target)
        err_angle = self.get_error_angles_in_degrees()
        torque = self.kp @ p + self.ki @ i + self.kd @ d

        feedforward = np.array([
            self.m_target[0] * self.i_coefficients[0],
            self.m_target[1] * self.i_coefficients[1],
            self.m_target[2] * self.i_coefficients[2]
        ])

        u = np.cross(torque, self.m_hat) + self.lmbd * self.m_hat + (err_angle * self.feedforward_scale / 180.0) * feedforward
        u = self.clip_input_currents(u)

        data = {
            'time': 0.0,
            'm_x': self.m_hat[0], 'm_y': self.m_hat[1], 'm_z': self.m_hat[2],
            'm_target_x': self.m_target[0], 'm_target_y': self.m_target[1], 'm_target_z': self.m_target[2],
            'angular_error_degrees': err_angle,
            'p_term_mag': np.linalg.norm(p),
            'i_term_mag': np.linalg.norm(i),
            'd_term_mag': np.linalg.norm(d),
            'input_torque_mag': np.linalg.norm(torque),
            'KP_x': self.kp[0, 0], 'KI_x': self.ki[0, 0], 'KD_x': self.kd[0, 0],
            'KP_y': self.kp[1, 1], 'KI_y': self.ki[1, 1], 'KD_y': self.kd[1, 1],
            'KP_z': self.kp[2, 2], 'KI_z': self.ki[2, 2], 'KD_z': self.kd[2, 2],
            'i_x': u[0], 'i_y': u[1], 'i_z': u[2]
        }

        return u, data

    def reset(self):
        self.t0 = None
        self.e_prev = np.zeros(3)
        self.q_e_prev = np.zeros(3)
        self.t0_target = time()
