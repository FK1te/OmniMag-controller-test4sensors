"""
Utility script for querying and manipulating the Universal Robot TCP pose.

Features:
- Connects to a Universal Robot (UR) via RTDE
- Retrieves and prints the current TCP position and orientation
- Provides optional example movements (commented for safety)
- Intended to be run from a Linux shell (not Windows)
"""

import time
import numpy as np
import rtde_control
import rtde_receive

from test_params import UNIVERSAL_ROBOT_PARAMS


# Connect to UR robot
ip_address = UNIVERSAL_ROBOT_PARAMS["ip_address"]
rtde_control_interface = rtde_control.RTDEControlInterface(ip_address)
rtde_receive_interface = rtde_receive.RTDEReceiveInterface(ip_address)

# Retrieve actual TCP pose
tcp_pose = rtde_receive_interface.getActualTCPPose()
UR_x, UR_y, UR_z, UR_rx, UR_ry, UR_rz = tcp_pose

# Example: Override with a fixed test TCP pose
tcp_pose = [
    0.6773973514703341, 0.34778435810601754, 
    0.6808982730077721, 2.2159557980970006,
    0.006550863337124177, 2.2214592596556475
]

# Example path (circular trajectory) — commented out unless needed
t_vals = np.arange(0, 10, 0.1)
x_offset = 0.1 * np.sin(t_vals)
y_offset = 0.1 * np.cos(t_vals)

"""
# Uncomment for testing circular motion around TCP
rtde_control_interface.moveL(tcp_pose, 0.05, 0.01, asynchronous=False)
time.sleep(5)

for k in range(len(t_vals)):
    modified_pose = tcp_pose.copy()
    modified_pose[0] += x_offset[k]
    modified_pose[1] += y_offset[k]
    rtde_control_interface.moveL(modified_pose, 0.05, 0.01, asynchronous=False)
"""

# Predefined robot joint configurations
sleep_configuration = [np.pi, -np.pi / 2, 0, -np.pi, -np.pi / 2, -np.pi / 2]
ready_configuration = [np.pi, -np.pi / 2, np.pi / 2, -np.pi, -np.pi / 2, -np.pi / 2]

# Move robot to sleep configuration
rtde_control_interface.moveJ(sleep_configuration, 0.05, 0.01, asynchronous=False)

# Print TCP information
print(f"Robot TCP position: {UR_x}, {UR_y}, {UR_z}, {UR_rx}, {UR_ry}, {UR_rz}.")

# Clean shutdown
rtde_control_interface.stopScript()
rtde_receive_interface.disconnect()
