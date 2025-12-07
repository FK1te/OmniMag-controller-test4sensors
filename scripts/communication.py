"""
Handles communication with:
    - Arduino: for reading sensor values
    - MiniMACS6: for inputting current values into electromagnetic coils
"""

import time
import socket
import serial
import struct
import numpy as np

from test_params import ARDUINO_MINIMACS6_DEFAULT_PARAMS


class ArduinoMinimacsCommunication:
    def __init__(self):
        minimacs6_comm = ARDUINO_MINIMACS6_DEFAULT_PARAMS["minimacs6_comm"]
        self._ip_address = minimacs6_comm["ip_address"]
        self._port_number = minimacs6_comm["port"]
        self._socket_recv_size = minimacs6_comm["socket_recv_size"]

        board = ARDUINO_MINIMACS6_DEFAULT_PARAMS["board"]
        self._arduino_port = board["com"]
        self._arduino_baudrate = board["baudrate"]
        self._arduino_timeout = board["timeout"]

        self.S_dagger_matrix = ARDUINO_MINIMACS6_DEFAULT_PARAMS["S_dagger_matrix"]

        self._socket_minimac = None
        self._serial_arduino = None

        self._connect_to_arduino()
        self._connect_to_minimacs()

    def _connect_to_arduino(self):
        try:
            self._serial_arduino = serial.Serial(
                self._arduino_port,
                self._arduino_baudrate,
                timeout=self._arduino_timeout
            )
            time.sleep(2)  # Wait for Arduino to initialize
            print(f"[✓] Connected to [Hall Sensors Cluster] Arduino on {self._arduino_port}")
        except serial.SerialException as e:
            print(f"[!] Arduino connection error: {e}")

    def _connect_to_minimacs(self):
        try:
            self._socket_minimac = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self._socket_minimac.connect((self._ip_address, self._port_number))
            if not self.change_status_enable_disable_current(False):
                raise Exception("Failed to disable current on MiniMACS6")
            print(f"[✓] Connected to MiniMACS6!")
        except Exception as e:
            self._socket_minimac = None
            print(f"[!] Cannot connect to MiniMACS6: {e}")

    def _order_current_coil(self, coil: str, current: float):
        """
        Sends a current command to the specified coil.

        Args:
            coil (str): Coil identifier ("X", "Y", or "Z")
            current (float): Current value in mA
        """
        current = int(current)
        try:
            self._socket_minimac.sendall(f"{coil} {current}".encode())
            response = self._socket_minimac.recv(self._socket_recv_size).decode()
            if response != "OK":
                print(f"[!] Failed to set current {current} mA in coil {coil}")
        except Exception as e:
            print(f"[!] Communication error when sending current to {coil}: {e}")

    def is_connected(self) -> bool:
        return self._serial_arduino is not None and self._socket_minimac is not None

    def get_sensor_readings(self) -> np.ndarray:
        # TO BE UPDATED - Should be OK
        # We need to read 4 sensors
        # 12D sensor vector
        """
        Retrieves the 12D sensor vector from Arduino.

        Returns:
            np.ndarray: Sensor readings in microtesla (converted to Tesla)
        """
        s_vec = np.zeros(12)

        if not self._serial_arduino:
            print("[!] Serial connection not initialized.")
            return s_vec

        try:
            self._serial_arduino.write(b'U')
            flags = self._serial_arduino.read(1)

            if len(flags) < 1:
                return s_vec

            flag = flags[0]

            float_count = 0
            # if flag & 0x01:
            #     float_count += 3
            # if flag & 0x02:
            #     float_count += 3
            if flag & 0x03:
                float_count += 3
            if flag & 0x04:
                float_count += 3

            total_bytes = float_count * 4
            float_bytes = self._serial_arduino.read(total_bytes)

            if len(float_bytes) < total_bytes:
                return s_vec

            floats = struct.unpack('<' + 'f' * float_count, float_bytes)

            idx = 0
            # if flag & 0x01:
            #     s_vec[idx:idx + 3] = 1e-6 * np.array(floats[idx:idx + 3])
            #     idx += 3
            # if flag & 0x02:
            #     s_vec[idx:idx + 3] = 1e-6 * np.array(floats[idx:idx + 3])
            #     idx += 3
            if flag & 0x03:
                s_vec[idx:idx + 3] = 1e-6 * np.array(floats[idx:idx + 3])
                idx += 3
            if flag & 0x04:
                s_vec[idx:idx + 3] = 1e-6 * np.array(floats[idx:idx + 3])

        except Exception as e:
            print(f"[!] Exception while reading sensor data: {e}")
            return np.zeros(12)

        return s_vec

    def get_magnetic_field(self) -> np.ndarray:
        """
        Returns the magnetic field vector by projecting sensor values through the S† matrix.

        Returns:
            np.ndarray: Magnetic field vector
        """
        s_vec = self.get_sensor_readings()
        return self.S_dagger_matrix @ s_vec

    def set_currents_in_coils(self, coils_currents: list):
        """
        Sets currents in X, Y, Z coils.

        Args:
            coils_currents (list): List of current values [X, Y, Z] in mA
        """
        self._order_current_coil("X", coils_currents[0])
        self._order_current_coil("Y", coils_currents[1])
        self._order_current_coil("Z", coils_currents[2])

    def change_status_enable_disable_current(self, enable: bool) -> bool:
        """
        Enables or disables the coil current output.

        Args:
            enable (bool): True to enable, False to disable

        Returns:
            bool: True if command succeeded
        """
        try:
            msg = "E" if enable else "D"
            self._socket_minimac.sendall(msg.encode())
            response = self._socket_minimac.recv(self._socket_recv_size).decode()
            return response == "OK"
        except Exception as e:
            print(f"[!] Error changing coil status: {e}")
            return False

    def shutdown(self):
        """
        Gracefully shuts down the communication: sets currents to 0,
        disables current, and closes serial/socket connections.
        """
        self.set_currents_in_coils([0, 0, 0])
        try:
            if self._socket_minimac:
                self._socket_minimac.sendall("D".encode())
                _ = self._socket_minimac.recv(self._socket_recv_size).decode()
                self._socket_minimac.close()
        except Exception as e:
            print(f"[!] Error during shutdown of MiniMACS6: {e}")
        finally:
            if self._serial_arduino:
                self._serial_arduino.close()


def main():
    comm = ArduinoMinimacsCommunication()
    if not comm.is_connected():
        print("[!] Device not connected. Exiting.")
        return

    comm.change_status_enable_disable_current(enable=True)
    comm.set_currents_in_coils([1000, 0, 0])

    try:
        while True:
            magnetic_field = comm.get_magnetic_field()
            field_norm = np.linalg.norm(magnetic_field)
            if field_norm > 1e-8:
                normalized_field = magnetic_field / field_norm
                print(f"[✓] Sensor readings: {normalized_field}")
    except KeyboardInterrupt:
        print("\n[✓] Interrupted by user.")
    finally:
        comm.shutdown()


if __name__ == "__main__":
    main()
