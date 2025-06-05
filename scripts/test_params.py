import numpy as np

ARDUINO_MINIMACS6_DEFAULT_PARAMS = {
    "board": {
        "com": "/dev/ttyACM0",          # COM port of the Arduino Mega
        "baudrate": 1000000,     # baudrate of the Arduino Mega
        "timeout": 1,           # seconds
    },
    "minimacs6_comm": {
        "ip_address": "169.254.1.1",    # IP address of the MiniMACS
        "port": 4123,                   # port of the MiniMACS,
        "socket_recv_size": 1024,       # nb bytes,
    },
    "S_dagger_matrix": np.array(
        [[-195.72770368345368, -16.54694125301256, 25051.645136975898, -195.7277036834543, 16.546941253012555, 25051.645136975898], 
        [-167.55523565932188, -25047.313299033165, 13.798666466061793, 167.55523565932188, -25047.31329903316, -13.798666466061789], 
        [-27412.21992963547, -200.92714378658658, 195.7277036834501, -27412.219929635477, 200.92714378657533, 195.7277036834501]])
}

MAGNET_CONTROLLER_PARAMS = {
    "currents": {
        "I_nominal": 2,     # A
        "I_hold": 0.3,      # A
        "I_max": 2,         # A     # MiniMACS6 will reject any value superior to 2000 mA
        "action_current_maintaining_delay": 2,  # seconds
        "current_direction_in_coils": {
            "coil_x": 1,    # should normally be correct, values can be 1 or -1
            "coil_y": 1,    # should normally be correct, values can be 1 or -1
            "coil_z": 1,     # should normally be correct, values can be 1 or -1
        },
        "I_coefficients": [1, 1, 1],
    },
    "pid_params" : {
    "kp" : 1.0,
    "ki" : 1.0,
    "kd" : 1.0, 
    "saturation_integral_term" : 1.0,
    "saturation_derivative_term" : 1.0,
    "lambda" : 1.0
    }
}

UNIVERSAL_ROBOT_PARAMS = {
    "ip_address": "192.168.1.100",
    "should_reset_pos_on_startup": False,
    "speed": 0.3,
    "acceleration": 0.3,
    "vertical_joints_position": np.array(
        [np.pi, -np.pi / 2, 0, -np.pi, -np.pi / 2, -np.pi / 2]
    ),
    "base_joints_position": np.array(
        [np.pi, -np.pi / 2, np.pi / 2, -np.pi, -np.pi / 2, -np.pi / 2]
    ),
    "update_based_on_theorical_move": True,  # check position only if a new position make any sense (e.g.: no move => no new position)
    "translate_precision": 4,  # precision: 0.0001 m = 0.1 mm
    "rotation_precision": 3,  # precision: 0.001 rad = 0.058 deg
    "boundaries": {    # allowable coordinates for TCP
        "x_min": -0.9,
        "x_max": 0.9, # was -0.1
        "y_min": -0.9, # was -0.45
        "y_max": 0.9, # was 0.45
        "z_min": 0.28,
        "z_max": 0.9, # was 0.85
    },
    "speed_control": { 
        "time_before_return": 2e-3,  # sec
        "acceleration": 0.25,  # m / sec^2
        "deceleration": 5,  # m / sec^2
    },
    "speed_tcp": { # allowable speed for TCP translation 
        "update_interval": 0.1,  # sec
        "max_trans": 0.07,  # m / sec
        "max_rot": 0.2,  # rad / sec
    },
    "custom_pid": {
        "loop_refresh_dt": 0.1,
        "acceptable_dist_error_to_complete": 0.5e-3,  # m
        "gain_kp": 5,  # No unit
    },
}

CAMERA_PARAMS = {}

VISION_PARAMS = {}

