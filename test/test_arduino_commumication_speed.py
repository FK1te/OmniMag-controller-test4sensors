import serial
import struct
import time

PORT = "COM4"  # Adjust as needed
BAUD = 1000000
TIMEOUT = 1

ser = serial.Serial(PORT, BAUD, timeout=TIMEOUT)
time.sleep(2)

def read_sensor_data():
    start = time.perf_counter()

    ser.write(b'U')

    flags = ser.read(1)
    if len(flags) < 1:
        print("Timeout: No data")
        return

    flag = flags[0]
    float_count = 0
    if flag & 0x01: float_count += 3
    if flag & 0x02: float_count += 3

    float_bytes = ser.read(float_count * 4)
    if len(float_bytes) < float_count * 4:
        print("Incomplete float data")
        return

    floats = struct.unpack('<' + 'f' * float_count, float_bytes)
    idx = 0
    data = {}

    if flag & 0x01:
        data["x1"], data["y1"], data["z1"] = floats[idx:idx+3]
        idx += 3
    else:
        data["x1"] = data["y1"] = data["z1"] = None

    if flag & 0x02:
        data["x2"], data["y2"], data["z2"] = floats[idx:idx+3]
    else:
        data["x2"] = data["y2"] = data["z2"] = None

    elapsed = (time.perf_counter() - start) * 1000  # ms
    print(f"Data: {data} | Time: {elapsed:.2f} ms")

while True:
    read_sensor_data()
    time.sleep(0.05)
