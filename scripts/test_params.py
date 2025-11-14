"""
Default configuration parameters for:
- Arduino & MiniMACS6 communication
- Magnetic field controllers (closed-loop and open-loop)
- Universal Robot TCP motion and joint settings
"""

import numpy as np

ARDUINO_MINIMACS6_DEFAULT_PARAMS = {
    "board": {
        "com": "COM4",            # COM port of the Arduino Mega
        "baudrate": 1000000,      # Baudrate for serial communication
        "timeout": 1,             # Timeout in seconds
    },
    "minimacs6_comm": {
        "ip_address": "169.254.1.1",  # IP address of the MiniMACS
        "port": 4123,                 # TCP port
        "socket_recv_size": 1024,     # Receive buffer size in bytes
    },
    "S_dagger_matrix": np.array([
        [3067.2613085487264, -118.26998751492293, 24782.928394792074, 3067.2613085487264, 118.26998751492293, 24782.928394792074],
        [-5620.229259717352, -24168.688672026736, 462.842409623782, 5620.229259717352, -24168.688672026732, -462.842409623782],
        [12209.789504612403, -1436.1355626812067, -3067.261308548727, 12209.789504612403, 1436.1355626812067, -3067.2613085487274]
    ])
}

MAGNET_CONTROLLER_PARAMS = {
    "currents": {
        "I_nominal": 2,  # A
        "I_hold": 0.3,   # A
        "I_max": 2,      # A (MiniMACS6 rejects values > 2000 mA)
        "action_current_maintaining_delay": 2,  # seconds
        "current_direction_in_coils": {
            "coil_x": 1,   # direction multiplier: 1 or -1
            "coil_y": 1,
            "coil_z": 1,
        },
        # Coil current coefficients (select based on magnet size)
        # 60 mm head: [1, 0.9, 0.7], 80 mm head: [1, 1.0, 1.0]
        "I_coefficients": [1, 0.9, 0.7],
    },
    "closed_loop_params": {
        "kp": 1.0,
        "ki": 1.0,
        "kd": 1.0,
        "saturation_integral_term": 1.0,
        "saturation_derivative_term": 0.04,
        "lambda": 1.0
    }
}

UNIVERSAL_ROBOT_PARAMS = {
    "ip_address": "192.168.1.100",
    "should_reset_pos_on_startup": False,
    "speed": 0.3,
    "acceleration": 0.3,
    "vertical_joints_position": np.array([
        np.pi, -np.pi / 2, 0, -np.pi, -np.pi / 2, -np.pi / 2
    ]),
    "base_joints_position": np.array([
        np.pi, -np.pi / 2, np.pi / 2, -np.pi, -np.pi / 2, -np.pi / 2
    ]),
    "update_based_on_theorical_move": True,  # only update if move is significant
    "translate_precision": 4,  # precision: 0.0001 m = 0.1 mm
    "rotation_precision": 3,   # precision: 0.001 rad = 0.058 deg
    "boundaries": {
        "x_min": -0.9,
        "x_max": 0.9,
        "y_min": -0.9,
        "y_max": 0.9,
        "z_min": 0.28,
        "z_max": 0.9,
    },
    "speed_control": {
        "time_before_return": 2e-3,  # sec
        "acceleration": 0.25,        # m/s²
        "deceleration": 5,           # m/s²
    },
    "speed_tcp": {
        "update_interval": 0.1,  # sec
        "max_trans": 0.07,       # m/s
        "max_rot": 0.2,          # rad/s
    },
    "custom_closed_loop": {
        "loop_refresh_dt": 0.1,
        "acceptable_dist_error_to_complete": 0.5e-3,  # m
        "gain_kp": 5,  # unitless
    },
}
