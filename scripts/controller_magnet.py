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
        self.__i_nominal = MAGNET_CONTROLLER_PARAMS["currents"]["I_nominal"]
        self.__i_hold = MAGNET_CONTROLLER_PARAMS["currents"]["I_hold"]
        self.__i_max = MAGNET_CONTROLLER_PARAMS["currents"]["I_max"]
        self.__i_coefficients = MAGNET_CONTROLLER_PARAMS["currents"]["I_coefficients"]
        default_current_dirs = MAGNET_CONTROLLER_PARAMS["currents"]
        self.__current_dirs_in_coils = np.array(
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
        sin_theta = np.linalg.norm(self.get_error_vector())
        return np.rad2deg(np.arctan2(sin_theta, cos_theta))
    
    def clip_input_currents(self, u):
        if np.any(np.abs(u) > 1e3 * self.__i_max):
            output_currents_values_mA = u / np.linalg.norm(u) * 1e3 * self.__i_max
        else:
            output_currents_values_mA = u.copy()
        output_currents_values_mA = np.round(np.diag(self.__current_dirs_in_coils) @  output_currents_values_mA)
        return output_currents_values_mA        

class MagnetControllerStatic(MagnetController):
    def __init__(self):
        super().__init__()

    def compute_control_currents(self, m_current, m_target):
        self.set_current_magnet_direction(m_current)
        self.set_target_magnet_direction(m_target)
        u = 1e3 * self.__i_nominal * self.m_target
        return self.clip_input_currents(u)

class MagnetControllerPID(MagnetController):
    def __init__(self):
        super().__init__()
        pid_params = MAGNET_CONTROLLER_PARAMS["pid_params"]

        self.kp = pid_params["kp"]
        self.ki = pid_params["ki"]
        self.kd = pid_params["kd"]
        self.sat_e = pid_params["saturation_integral_term"]
        self.sat_v = pid_params["saturation_derivative_term"]
        self.lmbd = pid_params["lambda"]

        self.t0 = None
        self.e_prev = None
        self.q_e_prev = None

    def set_pid_gains(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd

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
            integral_term = self.q_e_prev + delta_t * proportional_term
            derivative_term = (1/delta_t) * (proportional_term - self.e_prev)

        self.t0 = time()
        self.q_e_prev = integral_term
        self.e_prev = proportional_term
        return proportional_term, integral_term, derivative_term
    
    def compute_control_currents(self, m_current, m_target, debug = False):
        p, i, d = self.compute_pid(m_current, m_target)
        control_torque = self.kp * p + self.ki * i + self.kd * d
        print(control_torque)
        u = np.cross(control_torque, self.m_hat) + self.lmbd * self.m_hat
        u = self.clip_input_currents(u)
        print(u)
        return u, [self.kp * p , self.ki * i , self.kd * d , control_torque, self.get_error_angles_in_degrees()]
    
    def reset(self):
        self.t0 = None
        self.e_prev = None
        self.q_e_prev = None
        
