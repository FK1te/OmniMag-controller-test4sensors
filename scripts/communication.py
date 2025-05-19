"""
Python sctipt that manages communications with:
    - Arduino: to reading sensor values
    - Minimacs: to input current values
"""
import json
import socket
import serial
import numpy as np

from test_params import ARDUINO_MINIMACS6_DEFAULT_PARAMS

class ArduinoMinimacsCommunication():
    def __init__(self):
        minimacs6_comm = ARDUINO_MINIMACS6_DEFAULT_PARAMS["minimacs6_comm"]
        self.__ip_address = minimacs6_comm["ip_address"]
        self.__port_number = minimacs6_comm["port"]
        self.__socket_recv_size = minimacs6_comm["socket_recv_size"]

        board = ARDUINO_MINIMACS6_DEFAULT_PARAMS["board"]
        self.__arduino_port = board["com"]
        self.__arduino_baudrate = board["baudrate"]
        self.__arduino_timeout = board["timeout"]

        self.S_dagger_matrix = ARDUINO_MINIMACS6_DEFAULT_PARAMS["S_dagger_matrix"]

        self.__socket_minimac = None
        self.__serial_arduino = None

        # Setup connections
        self.__connect_to_arduino()
        self.__connect_to_minimacs()

    def __connect_to_arduino(self):
        try:
            self.__serial_arduino = serial.Serial(
                self.__arduino_port,
                self.__arduino_baudrate,
                timeout=self.__arduino_timeout
            )
            print(f"[✓] Connected to Arduino on {self.__arduino_port}")
        except serial.SerialException as e:
            print(f"[!] Arduino connection error: {e}")

    def __connect_to_minimacs(self):
        try:
            self.__socket_minimac = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.__socket_minimac.connect((self.__ip_address, self.__port_number))
            if not self.change_status_enable_disable_current(False):
                raise Exception("Failed to disable current on MiniMACS6")
            else:
                print(f"[✓] Connected to MiniMACS6!")
        except Exception as e:
            self.__socket_minimac = None
            print(f"[!] Cannot connect to MiniMACS6: {e}")

    def __order_current_coil(self, coil, current):
        """
        Orders the current in the given coil.
        No verification of enabling or disabling of the current is done here.

        Args:
            coil (str): The coil to order the current in.
            current in mA (float): The current to order in the coil.

        Returns:
            None
        """
        current = int(current)
        self.__socket_minimac.sendall((coil + " " + str(current)).encode())
        minimac_answer = self.__socket_minimac.recv(self.__socket_recv_size).decode()

        # print("Ordering current in coil", coil, "with value", current, "[mA]")

        if minimac_answer != "OK":
            print(
                "Failed to order current in coil",
                coil,
                "with value",
                current,
                "[mA]",
            )

    def is_connected(self):
        return self.__serial_arduino is not None and self.__socket_minimac is not None

    def get_sensor_readings(self):
        if self.__serial_arduino:
            self.__serial_arduino.write("UP\n".encode())
            arduino_answer = self.__serial_arduino.readline().decode()
            try:
                digital_input_vec = json.loads(arduino_answer)
                s_vec = 1e-6 * np.array([       # The sensor measurements are converted from uT to T
                    [digital_input_vec["x1"]],
                    [digital_input_vec["y1"]],
                    [digital_input_vec["z1"]],
                    [digital_input_vec["x2"]],
                    [digital_input_vec["y2"]],
                    [digital_input_vec["z2"]],
                ])
                return np.squeeze(s_vec)
            except (json.JSONDecodeError, KeyError, ValueError):
                return np.zeros(6)
        else:
            return np.zeros(6)

    def get_magnet_direction(self):
        s_vec = self.get_sensor_readings()
        return self.S_dagger_matrix @ s_vec

    def set_currents_in_coils(self, coils_currents):
        self.__order_current_coil("X", coils_currents[0])
        self.__order_current_coil("Y", coils_currents[1])
        self.__order_current_coil("Z", coils_currents[2])

    def change_status_enable_disable_current(self, enable):
        try:
            msg = "E" if enable else "D"
            self.__socket_minimac.sendall(msg.encode())
            response = self.__socket_minimac.recv(self.__socket_recv_size).decode()
            return response == "OK"
        except Exception as e:
            print(f"[!] Error changing coil status: {e}")
            return False
        
    def shutdown(self):
        self.set_currents_in_coils([0, 0, 0])
        try:
            if self.__socket_minimac:
                self.__socket_minimac.sendall("D".encode())
                response = self.__socket_minimac.recv(self.__socket_recv_size).decode()
                self.__socket_minimac.close()
        except Exception as e:
            print(f"[!] Error during shutdown of minimac: {e}")
        finally:
            if self.__serial_arduino:
                self.__serial_arduino.close()

def main():
    comm = ArduinoMinimacsCommunication()
    if not comm.is_connected():
        print("[!] Device not connected. Exiting.")
        return
    
    comm.change_status_enable_disable_current(enable=True)
    comm.set_currents_in_coils([1000, 0, 0])
    try:
        while True:
            s_vec = comm.get_magnet_direction()
            s_vec_norm = np.linalg.norm(s_vec)
            if s_vec_norm > 1e-8:
                s_vec /= np.linalg.norm(s_vec)
                print(f"[✓] Sensor readings: {s_vec}")

    except KeyboardInterrupt:
        print("\n[✓] Interrupted by user.")
    finally:
        comm.shutdown()

if __name__ == "__main__":
    main()