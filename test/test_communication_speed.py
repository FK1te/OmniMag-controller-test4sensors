import sys
import random
from time import time, sleep

sys.path.append('./')
sys.path.append('./scripts')

from scripts.communication import ArduinoMinimacsCommunication

def main():
    comm = ArduinoMinimacsCommunication()
    if not comm.is_connected():
        print("[!] Device not connected. Exiting.")
        return
    
    comm.change_status_enable_disable_current(enable=True)
    try:
        while True:
            t0 = time()
            _ = comm.get_magnetic_field()
            t_arduino = time() - t0
            
            current_input = [1000, 0, 0]
            t0 = time() 
            comm.set_currents_in_coils(current_input)
            t_minimacs = time() - t0

            print(f"[✓] Arduino communication time interval: {t_arduino * 1e3} ms.")
            print(f"[✓] Minimacs communication time interval: {t_minimacs * 1e3} ,s.")
            sleep(0.1)

    except KeyboardInterrupt:
        print("\n[✓] Interrupted by user.")
    finally:
        comm.shutdown()

if __name__ == "__main__":
    main()