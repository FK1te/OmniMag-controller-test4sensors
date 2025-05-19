"""
GUI script to visualize Magnet Controller PID Values
and select PID Gains
"""

import numpy as np
import tkinter as tk
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg

from controller_magnet import MagnetControllerPID  
from communication import ArduinoMinimacsCommunication

class MagneticFieldControllerApp(tk.Frame):
    def __init__(self, controller: MagnetControllerPID, communication : ArduinoMinimacsCommunication, master=None):
        super().__init__(master)
        self.grid(padx=10, pady=10)
        self.controller = controller
        self.communication = communication

        # Initialize vectors
        self.init_vec = np.array([1.0, 0.0, 0.0])
        self.target_vec = np.array([0.0, 1.0, 0.0])
        self.magnetic_field_meas = np.array([1.0, 0.0, 0.0])

        # Create plot figure
        self.figure = Figure((14, 8), dpi=80)
        self.canvas = FigureCanvasTkAgg(self.figure, master=self)
        self.canvas.get_tk_widget().grid(row=0, column=0, columnspan=4, pady=(0, 10))

        self.plot_updating = tk.BooleanVar(value=False)

        self.create_plots()
        self.create_input_frames()
        self.create_control_buttons()

        # Loop functions
        self.update_plots()
        self.read_sensor_measurements()

    def create_plots(self):
        self.axes = []
        self.lines = []
        self.plot_data = np.zeros((5, 100))
        self.x_data = np.linspace(0, 100, 100)
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
        # Initial magnetic field input
        init_field_frame = tk.LabelFrame(self, text="Initial Magnetic Field Direction", padx=10, pady=5)
        init_field_frame.grid(row=1, column=0, columnspan=5, sticky='ew', pady=5)
        self.init_x_entry = self._create_labeled_entry(init_field_frame, "X:", 0, 1)
        self.init_y_entry = self._create_labeled_entry(init_field_frame, "Y:", 0, 3)
        self.init_z_entry = self._create_labeled_entry(init_field_frame, "Z:", 0, 5)

        # Target magnetic field input
        target_field_frame = tk.LabelFrame(self, text="Target Magnetic Field Direction", padx=10, pady=5)
        target_field_frame.grid(row=2, column=0, columnspan=5, sticky='ew', pady=5)
        self.target_x_entry = self._create_labeled_entry(target_field_frame, "X:", 0, 1)
        self.target_y_entry = self._create_labeled_entry(target_field_frame, "Y:", 0, 3)
        self.target_z_entry = self._create_labeled_entry(target_field_frame, "Z:", 0, 5)

    def create_control_buttons(self):
        # Buttons for control
        button_frame = tk.Frame(self)
        button_frame.grid(row=3, column=2, columnspan=3, pady=10, sticky='e')

        tk.Checkbutton(button_frame, text="Enable Live Plot",
                       variable=self.plot_updating).grid(row=0, column=0, padx=10)

        tk.Button(button_frame, text="Set Target", command=self.set_target).grid(row=0, column=1, padx=10)
        tk.Button(button_frame, text="Quit and Shutdown", command=self.shutdown).grid(row=0, column=2, padx=10)

        # PID Sliders
        pid_slider_frame = tk.LabelFrame(self, text="PID Gains", padx=10, pady=5)
        pid_slider_frame.grid(row=3, column=0, columnspan=2, sticky="w", pady=10)

        self.kp_slider = self._create_pid_slider(pid_slider_frame, "Kp", 0, 100, 10)
        self.ki_slider = self._create_pid_slider(pid_slider_frame, "Ki", 1, 100, 0)
        self.kd_slider = self._create_pid_slider(pid_slider_frame, "Kd", 2, 100, 0)

        tk.Button(pid_slider_frame, text="Set Gains", command=self.set_gains).grid(row=1, column=0, columnspan=3, pady=(10, 0))

    def _create_labeled_entry(self, parent, label_text, row, col):
        tk.Label(parent, text=label_text).grid(row=row, column=col, padx=5, sticky='e')
        entry = tk.Entry(parent, width=6)
        entry.grid(row=row, column=col + 1, padx=5)
        return entry

    def _create_pid_slider(self, parent, label, column, max_gain, default_value):
        frame = tk.Frame(parent)
        frame.grid(row=0, column=column, padx=10)
        tk.Label(frame, text=label).pack()
        slider = tk.Scale(frame, from_=0.0, to=max_gain, resolution=1.0, orient="horizontal", length=150)
        slider.set(default_value)
        slider.pack()
        return slider

    def shutdown(self):
        self.quit()

    def set_target(self):
        try:
            vec = np.array([
                float(self.init_x_entry.get()),
                float(self.init_y_entry.get()),
                float(self.init_z_entry.get())
            ])
            self.init_vec = vec / np.linalg.norm(vec)
        except Exception as e:
            print(f"Invalid init vector input: {e}")
            self.init_vec = np.array([1, 0, 0])

        try:
            vec = np.array([
                float(self.target_x_entry.get()),
                float(self.target_y_entry.get()),
                float(self.target_z_entry.get())
            ])
            self.target_vec = vec / np.linalg.norm(vec)
        except Exception as e:
            print(f"Invalid target vector input: {e}")
            self.target_vec = np.array([0, 1, 0])

        self.controller.reset()
        print("Target vector is set.")

    def set_gains(self):
        kp = self.kp_slider.get()
        ki = self.ki_slider.get()
        kd = self.kd_slider.get()
        self.controller.set_pid_gains(kp, ki, kd)
        print(f"Set Gains -> Kp: {kp}, Ki: {ki}, Kd: {kd}")

    def update_plots(self):
        if self.plot_updating.get() and self.magnetic_field_meas:
            data_list = self.controller.get_error_angles_in_degrees(self.magnetic_field_meas, self.target_vec, True)
            for i in range(5):
                self.plot_data[i] = np.roll(self.plot_data[i], -1)
                self.plot_data[i, -1] = data_list[i]
                self.lines[i].set_ydata(self.plot_data[i])
                max_val = np.max(self.plot_data[i]) + 1
                min_val = np.min(self.plot_data[i]) - 1
                self.axes[i].set_ylim(min_val, max_val)

            self.canvas.draw()

        self.after(250, self.update_plots)

    def read_sensor_measurements(self):
        self.magnetic_field_meas = self.communication.get_magnet_direction()
        self.after(100, self.read_sensor_measurements)

if __name__ == "__main__":
    root = tk.Tk()
    root.title("Magnetic Field Controller")
    comm = ArduinoMinimacsCommunication()
    comm.change_status_enable_disable_current(True)
    app = MagneticFieldControllerApp(master=root, controller=MagnetControllerPID())
    app.mainloop()
