import os
import time
import numpy as np
import pandas as pd
import tkinter as tk
from datetime import datetime
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg

from controller_magnet import MagnetControllerPID  
from communication import ArduinoMinimacsCommunication

class MagneticFieldControllerApp(tk.Frame):
    def __init__(self, controller: MagnetControllerPID, communication : ArduinoMinimacsCommunication, master=None):
        super().__init__(master)
        self.controller = controller
        self.communication = communication
        
        self.grid(padx=10, pady=10)

        self.magnetic_field_meas = np.array([0.0, 0.0, 0.0])
        self.target_vec = np.array([1.0, 0.0, 0.0])

        self.figure = Figure((12, 10), dpi=80)
        self.canvas = FigureCanvasTkAgg(self.figure, master=self)
        self.canvas.get_tk_widget().grid(row=0, column=0, rowspan=10, padx=10, pady=10)

        self.plot_updating = tk.BooleanVar(value=False)

        self.create_plots()
        self.create_input_frames()
        self.create_control_buttons()

        self.start_time = time.time()

        # Data Log Setup
        self.data_log_columns = [
            'time', 'm_x', 'm_y', 'm_z',
            'm_target_x', 'm_target_y', 'm_target_z',
            'angular_error_degrees', 'p_term_mag', 'i_term_mag', 'd_term_mag', 'input_torque_mag',
            'KP_x', 'KI_x', 'KD_x', 'KP_y', 'KI_y', 'KD_y', 'KP_z', 'KI_z', 'KD_z',
            'i_x', 'i_y', 'i_z'
        ]
        self.data_log = pd.DataFrame(columns=self.data_log_columns)

        self.read_sensor_measurements()
        self.update_plots()

    def create_plots(self):
        self.axes = []
        self.lines = []
        self.plot_data = np.zeros((5, 100))
        self.x_data = np.linspace(-100, 0, 100)
        plot_labels = ["|P(t)|", "|I(t)|", "|D(t)|", "|u(t)|", "|e(t)|"]

        for i in range(5):
            ax = self.figure.add_subplot(5, 1, i + 1, sharex=self.axes[0] if self.axes else None)
            line, = ax.plot(self.x_data, self.plot_data[i], label=plot_labels[i])
            ax.set_ylabel(plot_labels[i], fontsize=12)
            ax.set_ylim(0, 100)
            if i < 4:
                ax.tick_params(labelbottom=False)
            self.axes.append(ax)
            self.lines.append(line)

        self.axes[-1].set_xlabel("Time (s)", fontsize=12)
        self.figure.tight_layout(h_pad=1.5)

    def create_input_frames(self):
        target_field_frame = tk.LabelFrame(self, text="Target Magnetic Field Direction", padx=10, pady=10)
        target_field_frame.grid(row=0, column=1, sticky='new', pady=5, columnspan=2)

        self.target_x_entry = self._create_labeled_entry(target_field_frame, "X:", 0, 0)
        self.target_y_entry = self._create_labeled_entry(target_field_frame, "Y:", 0, 2)
        self.target_z_entry = self._create_labeled_entry(target_field_frame, "Z:", 0, 4)
        tk.Button(target_field_frame, text="Set Target", command=self.set_target).grid(row=0, column=6, padx=5)

        pid_frame = tk.LabelFrame(self, text="PID Gains", padx=10, pady=5)
        pid_frame.grid(row=1, column=1, columnspan=2, sticky="new", pady=10)

        self.kp_slider_x = self._create_pid_slider(pid_frame, "Kp X", 0, default_value=600)
        self.ki_slider_x = self._create_pid_slider(pid_frame, "Ki X", 1, default_value=893)
        self.kd_slider_x = self._create_pid_slider(pid_frame, "Kd X", 2, default_value=20)

        self.kp_slider_y = self._create_pid_slider(pid_frame, "Kp Y", 3, default_value=600)
        self.ki_slider_y = self._create_pid_slider(pid_frame, "Ki Y", 4, default_value=702)
        self.kd_slider_y = self._create_pid_slider(pid_frame, "Kd Y", 5, default_value=16)

        self.kp_slider_z = self._create_pid_slider(pid_frame, "Kp Z", 6, default_value=510)
        self.ki_slider_z = self._create_pid_slider(pid_frame, "Ki Z", 7, default_value=382)
        self.kd_slider_z = self._create_pid_slider(pid_frame, "Kd Z", 8, default_value=9)

        self.lambda_slider = self._create_pid_slider(pid_frame, "Lambda", 9, to=10, default_value=1.0, resolution=0.1)

        tk.Button(pid_frame, text="Set Gains", command=self.set_gains).grid(row=10, column=0, columnspan=6, pady=10)

    def create_control_buttons(self):
        ctrl_frame = tk.Frame(self)
        ctrl_frame.grid(row=9, column=1, columnspan=2, pady=10)

        tk.Checkbutton(ctrl_frame, text="Enable Live Plot", variable=self.plot_updating).pack(side="left", padx=10)
        tk.Button(ctrl_frame, text="Save Log", command=self.save_data_log).pack(side="left", padx=10)
        tk.Button(ctrl_frame, text="Reset", command=self.reset_data).pack(side="left", padx=10)  # New Reset Button
        tk.Button(ctrl_frame, text="Quit", command=self.shutdown).pack(side="left", padx=10)

    def reset_data(self):
        print("Resetting data and plots...")
        self.data_log = pd.DataFrame(columns=self.data_log_columns)
        self.plot_data = np.zeros((5, 100))
        self.x_data = np.linspace(-100, 0, 100)
        self.start_time = time.time()
        for i in range(5):
            self.lines[i].set_ydata(self.plot_data[i])
            self.lines[i].set_xdata(self.x_data)
            self.axes[i].relim()
            self.axes[i].autoscale_view()
        self.canvas.draw()

    def _create_labeled_entry(self, parent, label_text, row, col):
        tk.Label(parent, text=label_text).grid(row=row, column=col, sticky="e")
        entry = tk.Entry(parent, width=6)
        entry.grid(row=row, column=col + 1, padx=5)
        return entry

    def _create_pid_slider(self, parent, label, row, to=2000, default_value=0, resolution=1.0):
        tk.Label(parent, text=label).grid(row=row, column=0, sticky='w')
        slider = tk.Scale(parent, from_=0.0, to=to, resolution=resolution,
                          orient="horizontal", length=200)
        slider.set(default_value)
        slider.grid(row=row, column=1, columnspan=2, pady=2, sticky='w')
        return slider

    def set_target(self):
        try:
            vec = np.array([
                float(self.target_x_entry.get()),
                float(self.target_y_entry.get()),
                float(self.target_z_entry.get())
            ])
            vec_norm = np.linalg.norm(vec)
            if vec_norm < 1e-7:
                raise Exception("Norm of target vector is close to 0.")
            self.target_vec = vec / vec_norm
            print(f"Target vector set to: {self.target_vec}")
        except Exception as e:
            print(f"Error setting target vector: {e}")
            self.target_vec = np.array([0, 1, 0])

    def set_gains(self):
        gains = {
            'KP_x': self.kp_slider_x.get(),
            'KI_x': self.ki_slider_x.get(),
            'KD_x': self.kd_slider_x.get(),
            'KP_y': self.kp_slider_y.get(),
            'KI_y': self.ki_slider_y.get(),
            'KD_y': self.kd_slider_y.get(),
            'KP_z': self.kp_slider_z.get(),
            'KI_z': self.ki_slider_z.get(),
            'KD_z': self.kd_slider_z.get(),
            'Lambda': self.lambda_slider.get()
        }
        self.controller.set_pid_gains(gains)
        print("Gains set:", gains)

    def update_plots(self):
        if self.plot_updating.get():
            current_time = time.time() - self.start_time
            self.x_data = np.roll(self.x_data, -1)
            self.x_data[-1] = current_time

            u, data = self.controller.compute_control_currents(self.magnetic_field_meas, self.target_vec)
            self.communication.set_currents_in_coils(u)

            plot_values = [data['p_term_mag'],
                           data['i_term_mag'],
                           data['d_term_mag'],
                           data['input_torque_mag'],
                           data['angular_error_degrees']]

            for i in range(5):
                self.plot_data[i] = np.roll(self.plot_data[i], -1)
                self.plot_data[i, -1] = plot_values[i]
                self.lines[i].set_ydata(self.plot_data[i])
                self.lines[i].set_xdata(self.x_data)
                self.axes[i].relim()
                self.axes[i].autoscale_view()
                
                y_min = np.min(self.plot_data[i])
                y_max = np.max(self.plot_data[i])
                padding = (y_max - y_min) * 0.1 if y_max != y_min else 1
                self.axes[i].set_ylim(y_min - padding, y_max + padding)

            self.canvas.draw()
            
            data['time'] = current_time
            self.data_log.loc[len(self.data_log)] = data

        self.after(100, self.update_plots)

    def read_sensor_measurements(self):
        self.magnetic_field_meas = self.communication.get_magnetic_field()
        self.after(100, self.read_sensor_measurements)

    def save_data_log(self):
        filename = f"./data/log_{datetime.now().strftime('%Y-%m-%d_%H-%M-%S')}.csv"
        self.data_log.to_csv(filename, index=False)
        print(f"Data log saved to {filename}")

    def shutdown(self):
        # self.save_data_log()
        self.communication.shutdown()
        self.quit()


if __name__ == "__main__":
    root = tk.Tk()
    root.title("Magnetic Field Controller")

    controller = MagnetControllerPID()
    comm = ArduinoMinimacsCommunication()
    comm.change_status_enable_disable_current(True)

    app = MagneticFieldControllerApp(master=root, 
                                     communication=comm, 
                                     controller=controller)
    
    app.mainloop()
