import os
import sys
import csv
import time
import serial
import struct
import numpy as np
from datetime import datetime

import rtde_control
import rtde_receive

from communication import ArduinoMinimacsCommunication
from controller_magnet import MagnetControllerPID
from test_params import UNIVERSAL_ROBOT_PARAMS, ARDUINO_MINIMACS6_DEFAULT_PARAMS

S_dagger = ARDUINO_MINIMACS6_DEFAULT_PARAMS["S_dagger_matrix"]

ip_address = UNIVERSAL_ROBOT_PARAMS["ip_address"]
rtde_c = rtde_control.RTDEControlInterface(ip_address)
rtde_r = rtde_receive.RTDEReceiveInterface(ip_address)
robot_tcp_position = rtde_r.getActualTCPPose()
[UR_x, UR_y, UR_z, UR_rx, UR_ry, UR_rz] = robot_tcp_position
print(f"Robot TCP position: {UR_x}, {UR_y}, {UR_z}, {UR_rx}, {UR_ry}, {UR_rz}.")
rtde_c.stopScript()
rtde_r.disconnect()