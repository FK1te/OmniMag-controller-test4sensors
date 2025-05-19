import numpy as np

ARDUINO_MINIMACS6_DEFAULT_PARAMS = {
    "board": {
        "com": "COM4",          # COM port of the Arduino Mega
        "baudrate": 115200,     # baudrate of the Arduino Mega
        "timeout": 1,           # seconds
    },
    "minimacs6_comm": {
        "ip_address": "169.254.1.1",    # IP address of the MiniMACS
        "port": 4123,                   # port of the MiniMACS,
        "socket_recv_size": 1024,       # nb bytes,
    },
    "S_dagger_matrix": np.array(
        [[195.7277036834506, -16.54694125301256, -25051.645136975898, 195.72770368345124, 16.546941253012555, -25051.645136975898], 
         [167.55523565932188, -25047.313299033165, -13.798666466061814, -167.55523565932188, -25047.31329903316, 13.79866646606181], 
         [27412.21992963547, -200.92714378658658, -195.72770368345346, 27412.219929635477, 200.92714378657533, -195.72770368345346]])
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

UNIVERSAL_ROBOT_PARAMS = {}

CAMERA_PARAMS = {}

VISION_PARAMS = {}

