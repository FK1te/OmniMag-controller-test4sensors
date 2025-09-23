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
from controller_magnet import d
from test_params import UNIVERSAL_ROBOT_PARAMS, ARDUINO_MINIMACS6_DEFAULT_PARAMS

S_dagger = ARDUINO_MINIMACS6_DEFAULT_PARAMS["S_dagger_matrix"]

ip_address = UNIVERSAL_ROBOT_PARAMS["ip_address"]
rtde_c = rtde_control.RTDEControlInterface(ip_address)
rtde_r = rtde_receive.RTDEReceiveInterface(ip_address)
robot_tcp_position = rtde_r.getActualTCPPose()
[UR_x, UR_y, UR_z, UR_rx, UR_ry, UR_rz] = robot_tcp_position
tcp_position = [
    UR_x,
    UR_y,
    UR_z,
    UR_rx,
    UR_ry,
    UR_rz,
]

tcp_position = [0.6773973514703341, 0.34778435810601754, 
                0.6808982730077721, 2.2159557980970006, 0.006550863337124177, 2.2214592596556475]

t = np.arange(0, 10, 0.1)
x = 0.1 * np.sin(t)
y = 0.1 * np.cos(t)
"""
new_tcp_position = [pos for pos in tcp_position]
new_tcp_position[1] += 0.1
rtde_c.moveL(tcp_position, 0.05, 0.01, asynchronous=False)
time.sleep(5)
for k in range(10):
    new_tcp_position = [pos for pos in tcp_position]
    new_tcp_position[0] += x[k]
    new_tcp_position[1] += y[k]
    rtde_c.moveL(new_tcp_position, 0.05, 0.01, asynchronous=False)
"""
sleep = [np.pi, -np.pi / 2, 0, -np.pi, -np.pi / 2, -np.pi / 2]
ready = [np.pi, -np.pi / 2, np.pi / 2, -np.pi, -np.pi / 2, -np.pi / 2]
# rtde_c.moveJ(ready, 0.05, 0.01, asynchronous=False)
rtde_c.moveJ(sleep, 0.05, 0.01, asynchronous=False)
# rtde_c.moveL(tcp_position, 0.05, 0.01, asynchronous=False)
# robot_tcp_position = rtde_r.getActualTCPPose()
[UR_x, UR_y, UR_z, UR_rx, UR_ry, UR_rz] = robot_tcp_position
print(f"Robot TCP position: {UR_x}, {UR_y}, {UR_z}, {UR_rx}, {UR_ry}, {UR_rz}.")
rtde_c.stopScript()
rtde_r.disconnect()