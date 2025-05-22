"""
Python script that manages communication with the universal robot. 
Functionalities:
    - Set/get end effector (tcp) position and orientation
    - Perform path planning and following for the robotic system
    - Compute the magnetic field vector in TCP coordinate frame during motion
"""
import time
import threading
import numpy as np
import rtde_control
import rtde_receive

from test_params import UNIVERSAL_ROBOT_PARAMS

class RobotController():
    def __init__(self):
        # For connection with the robot
        self.__ip_address = UNIVERSAL_ROBOT_PARAMS["ip_address"]
        self.__rtde_c = None
        self.__rtde_r = None
        self.__is_connected = False

        # Class vectors and matrices
        self.__target_m_base_frame = None
        self.__target_m_tcp_frame = None
        self.__robot_tcp_position = None
        self.__target_robot_tcp_position = None
        self.__base_to_tcp_transformation_matrix = None

        # Threading variables
        # Robot motion thread
        self.__thread_robot_motion = None
        self.__stop_robot_motion = threading.Event()

        # Target magnetic vector in tcp frame computation thread
        self.__thread_m_tcp_computation = None
        self.__stop_m_tcp_computation = threading.Event()

    def __connect_to_robot(self):
        try:
            self.__rtde_c = rtde_control.RTDEControlInterface(self.__ip_address)
            self.__rtde_r = rtde_receive.RTDEReceiveInterface(self.__ip_address)
            self.__is_connected = True
        except Exception as e:
            self.__is_connected = False
            raise ValueError(f"Cannot connect to Universal Robot at IP Adress: {self.__ip_address}")

    def is_connected(self):
        return self.__rtde_r.isConnected() and self.__rtde_c.isConnected()

    def start(self):
        self.__connect_to_robot()
        self.__get_robot_tcp_position()
        if self.__thread_m_tcp_computation is None or not self.__thread_m_tcp_computation.is_alive():
            self.__stop_m_tcp_computation.clear()
            self.__thread_m_tcp_computation = threading.Thread(target=self.publish_target_magnetic_field_in_tcp_frame)
            self.__thread_m_tcp_computation.start()

    def shutdown(self):
        self.__stop_m_tcp_computation.set()
        if self.__thread_m_tcp_computation is not None:
            self.__thread_m_tcp_computation.join()
            print("[Compute target m in tcp frame] Joined")

        self.__stop_robot_motion.set()
        if self.__thread_robot_motion is not None:
            self.__thread_robot_motion.join()
            print("[Robot motion thread] Joined")

    # serial functions
    def set_target_in_base_frame(self, vector : np.ndarray):
        assert (vector.shape == (3,) or vector.shape == (3, 1)), "Target magnetic vector must be 3 dimenstional"
        self.__target_m_base_frame = np.vstack((np.expand_dims(np.squeeze(vector), 1), 1))

    def set_target_robot_tcp(self, target_tcp):
        """
        assert (target_tcp.shape != (6,) and target_tcp.shape != (6, 1)), "Target magnetic vector must be 3 dimenstional"
        self.__target_robot_tcp_position = np.squeeze(target_tcp)
        """
        tcp_pos = self.__rtde_r.getActualTCPPose()
        tcp_pos[1] += 0.1
        self.__target_robot_tcp_position = tcp_pos
        self.__stop_robot_motion.clear()
        self.__thread_robot_motion = threading.Thread(target=self.move_to_target)
        self.__thread_robot_motion.start()

    def __get_robot_tcp_position(self):
        self.__robot_tcp_position = self.__rtde_r.getActualTCPPose()
        return self.__robot_tcp_position
    
    # functions to be used in the target publishing thread
    def compute_base_to_tcp_transformation_matrix(self):
        if not self.__is_connected:
            raise ConnectionError("Robot not connected.")
        [x, y, z, rx, ry, rz] = self.__get_robot_tcp_position()

        # Convert rotation vector (rx, ry, rz) to rotation matrix using Rodrigues' formula
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
        self.__base_to_tcp_transformation_matrix = T

    def get_target_in_tcp_frame(self):
        if self.__base_to_tcp_transformation_matrix is not None and self.__target_m_base_frame is not None:
            transformed_vector = self.__base_to_tcp_transformation_matrix @ self.__target_m_base_frame
            return transformed_vector[:3]

    # functions to be used in the robot motion thread
    def target_robot_tcp_position_is_reached(self):
        if self.__target_robot_tcp_position is None:
            return True
        
        tcp_pos = self.__rtde_r.getActualTCPPose()
        return True if np.linalg.norm(tcp_pos[:3]-self.__target_robot_tcp_position[:3]) < 0.05 else False

    # thread functions
    def move_to_target(self):
        print(f"[Thread started] Moving to robot position {self.__target_robot_tcp_position[:3]}")
        self.__rtde_c.moveL(self.__target_robot_tcp_position, 0.1, 0.05)
        while not self.__stop_robot_motion.is_set():
            if self.target_robot_tcp_position_is_reached():
                self.__stop_robot_motion.set()
                
        print("Robot movement is complete.")

    def publish_target_magnetic_field_in_tcp_frame(self):
        print("[Thread started] Target magnetic field in tcp coordinate frame computation")
        while not self.__stop_m_tcp_computation.is_set():
            self.compute_base_to_tcp_transformation_matrix()
            self.__target_m_tcp_frame = self.get_target_in_tcp_frame()
            print(f"Target magnetic field vector in tcp frame: {self.__target_m_tcp_frame}")
            time.sleep(0.5)
        print("[Compute target m in tcp frame] Thread exiting")

def main():
    robot_controller = RobotController()
    robot_controller.set_target_in_base_frame(np.array([0.6, 0.8, 0.0]))
    robot_controller.start()
    time.sleep(10.0)
    robot_controller.shutdown()

if __name__ == "__main__":
    main()