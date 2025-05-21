import random
from time import time, sleep

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
            
            current_input = [10, 10, 10]
            t0 = time() 
            comm.set_currents_in_coils(current_input)
            t_minimacs = time() - t0

            print(f"[✓] Arduino communication time: {t_arduino} seconds")
            print(f"[✓] Minimacs communication time: {t_minimacs} seconds")

    except KeyboardInterrupt:
        print("\n[✓] Interrupted by user.")
    finally:
        comm.shutdown()

if __name__ == "__main__":
    main()