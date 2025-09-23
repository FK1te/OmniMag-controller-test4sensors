# ================= UBUNTU SIDE =================
# Robot controller script that listens for pose updates via JSON

import json
import time
import rtde_control
import rtde_receive

robot_command_file = 'shared/robot_command.json'
robot_feedback_file = 'shared/robot_feedback.json'

ip_address = "192.168.1.100"
rtde_c = rtde_control.RTDEControlInterface(ip_address)
rtde_r = rtde_receive.RTDEReceiveInterface(ip_address)

last_command = None

while True:
    try:
        # Read command from Windows
        with open(robot_command_file, 'r') as f:
            command_data = json.load(f)
    except Exception:
        command_data = {}

    # If a move command is present and new
    if command_data.get("move", False):
        target_pose = command_data.get("target_pose", None)
        if target_pose and target_pose != last_command:
            try:
                print("Moving to:", target_pose)
                rtde_c.moveL(target_pose, 0.05, 0.01)
                last_command = target_pose
            except Exception as e:
                print("Move failed:", e)

    # Always write feedback
    try:
        pose = rtde_r.getActualTCPPose()
        with open(robot_feedback_file, 'w') as f:
            json.dump({'tcp_pose': pose}, f)
    except Exception as e:
        print("Pose feedback write failed:", e)

    time.sleep(0.1)
