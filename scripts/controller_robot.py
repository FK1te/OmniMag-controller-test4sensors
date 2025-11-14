"""
Controls Universal Robot to position the end effector (TCP) for magnetic field interaction.

Features:
- Connects to Universal Robot via RTDE interface
- Retrieves and updates TCP position
- Moves robot toward target TCP position
- Computes transformation matrix from robot base to TCP frame
- Publishes target magnetic vector in TCP frame via background thread
"""

import time
import threading
import numpy as np
import rtde_control
import rtde_receive

from test_params import UNIVERSAL_ROBOT_PARAMS


class RobotController:
    def __init__(self):
        # Connection
        self._ip_address = UNIVERSAL_ROBOT_PARAMS["ip_address"]
        self._rtde_c = None
        self._rtde_r = None
        self._is_connected = False

        # State variables
        self._target_m_base_frame = None
        self._target_m_tcp_frame = None
        self._robot_tcp_position = None
        self._target_robot_tcp_position = None
        self._base_to_tcp_matrix = None

        # Threading
        self._thread_robot_motion = None
        self._stop_robot_motion = threading.Event()

        self._thread_m_tcp_computation = None
        self._stop_m_tcp_computation = threading.Event()

    def _connect_to_robot(self):
        try:
            self._rtde_c = rtde_control.RTDEControlInterface(self._ip_address)
            self._rtde_r = rtde_receive.RTDEReceiveInterface(self._ip_address)
            self._is_connected = True
        except Exception:
            self._is_connected = False
            raise ValueError(f"Cannot connect to Universal Robot at IP Address: {self._ip_address}")

    def is_connected(self):
        return self._rtde_r.isConnected() and self._rtde_c.isConnected()

    def start(self):
        self._connect_to_robot()
        self._get_robot_tcp_position()
        if self._thread_m_tcp_computation is None or not self._thread_m_tcp_computation.is_alive():
            self._stop_m_tcp_computation.clear()
            self._thread_m_tcp_computation = threading.Thread(
                target=self.publish_target_magnetic_field_in_tcp_frame
            )
            self._thread_m_tcp_computation.start()

    def shutdown(self):
        self._stop_m_tcp_computation.set()
        if self._thread_m_tcp_computation is not None:
            self._thread_m_tcp_computation.join()
            print("[Compute target m in tcp frame] Joined")

        self._stop_robot_motion.set()
        if self._thread_robot_motion is not None:
            self._thread_robot_motion.join()
            print("[Robot motion thread] Joined")

    def set_target_in_base_frame(self, vector: np.ndarray):
        assert vector.shape in [(3,), (3, 1)], "Target magnetic vector must be 3-dimensional"
        self._target_m_base_frame = np.vstack((np.expand_dims(np.squeeze(vector), 1), 1))

    def set_target_robot_tcp(self, target_tcp):
        # This function currently performs a simple offset move instead of using input
        tcp_pos = self._rtde_r.getActualTCPPose()
        tcp_pos[0] += 0.1  # Move X by +10cm
        self._target_robot_tcp_position = tcp_pos
        self._stop_robot_motion.clear()
        self._thread_robot_motion = threading.Thread(target=self.move_to_target)
        self._thread_robot_motion.start()

    def _get_robot_tcp_position(self):
        self._robot_tcp_position = self._rtde_r.getActualTCPPose()
        return self._robot_tcp_position

    def compute_base_to_tcp_transformation_matrix(self):
        if not self._is_connected:
            raise ConnectionError("Robot not connected.")
        [x, y, z, rx, ry, rz] = self._get_robot_tcp_position()

        # Convert axis-angle to rotation matrix (Rodrigues' formula)
        theta = np.linalg.norm([rx, ry, rz])
        if theta < 1e-6:
            R = np.eye(3)
        else:
            k = np.array([rx, ry, rz]) / theta
            K = np.array([
                [0, -k[2], k[1]],
                [k[2], 0, -k[0]],
                [-k[1], k[0], 0]
            ])
            R = np.eye(3) + np.sin(theta) * K + (1 - np.cos(theta)) * (K @ K)

        T = np.eye(4)
        T[:3, :3] = R
        T[:3, 3] = [x, y, z]
        self._base_to_tcp_matrix = T

    def get_target_in_tcp_frame(self):
        if self._base_to_tcp_matrix is not None and self._target_m_base_frame is not None:
            transformed = self._base_to_tcp_matrix @ self._target_m_base_frame
            return transformed[:3]

    def target_robot_tcp_position_is_reached(self):
        if self._target_robot_tcp_position is None:
            return True

        current_pos = self._rtde_r.getActualTCPPose()
        return np.linalg.norm(np.array(current_pos[:3]) - np.array(self._target_robot_tcp_position[:3])) < 0.05

    def move_to_target(self):
        print(f"[Thread started] Moving to robot position {self._target_robot_tcp_position[:3]}")
        self._rtde_c.moveL(self._target_robot_tcp_position, 0.1, 0.05)
        while not self._stop_robot_motion.is_set():
            if self.target_robot_tcp_position_is_reached():
                self._stop_robot_motion.set()
        print("Robot movement is complete.")

    def publish_target_magnetic_field_in_tcp_frame(self):
        print("[Thread started] Target magnetic field in tcp coordinate frame computation")
        while not self._stop_m_tcp_computation.is_set():
            self.compute_base_to_tcp_transformation_matrix()
            self._target_m_tcp_frame = self.get_target_in_tcp_frame()
            print(f"Target magnetic field vector in tcp frame: {self._target_m_tcp_frame}")
            time.sleep(0.1)
        print("[Compute target m in tcp frame] Thread exiting")


def main():
    robot_controller = RobotController()
    robot_controller.set_target_in_base_frame(np.array([0.6, 0.8, 0.0]))
    robot_controller.start()
    time.sleep(0.25)
    robot_controller.set_target_robot_tcp(None)
    time.sleep(10.0)
    robot_controller.shutdown()


if __name__ == "__main__":
    main()
