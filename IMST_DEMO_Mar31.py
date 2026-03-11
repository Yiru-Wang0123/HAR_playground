#!/usr/bin/env python3
from __future__ import print_function
import numpy as np
from smbus2 import SMBus
from time import sleep, time
import sys
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import statistics
import csv
import glob
import os
import threading
import tkinter as tk
from tkinter import ttk
import qwiic_oled_display  # OLED library from SparkFun

# Import your sensor configuration details.
from config_file import *
from registers import *
from definitions import *

###############################################################################
# BMI270 Class (IMU Initialization and Sensor Functions)
###############################################################################
class BMI270:
    def __init__(self, i2c_addr) -> None:
        # Use I2C_BUS from config_file (or use 1 if not defined)
        try:
            self.bus = SMBus(I2C_BUS)
        except Exception:
            self.bus = SMBus(1)
        if (self.bus == -1):
            print("---- ERROR: I2C BUS NOT FOUND ----")
            exit(1)
        else:
            print("---- I2C BUS FOUND ----")
        self.address = i2c_addr
        chip_id = self.bus.read_byte_data(i2c_addr, CHIP_ID_ADDRESS)
        print(hex(self.address), " --> Chip ID: " + hex(chip_id))
        self.acc_range = 1 * GRAVITY
        self.acc_odr = 100
        self.gyr_range = 1000
        self.gyr_odr = 200

    def __unsignedToSigned__(self, n, byte_count) -> int:
        return int.from_bytes(n.to_bytes(byte_count, 'little', signed=False), 'little', signed=True)

    def __signedToUnsigned__(self, n, byte_count) -> int:
        return int.from_bytes(n.to_bytes(byte_count, 'little', signed=True), 'little', signed=False)

    def read_register(self, register_address) -> int:
        return self.bus.read_byte_data(self.address, register_address)

    def write_register(self, register_address, byte_data) -> None:
        self.bus.write_byte_data(self.address, register_address, byte_data)

    def load_config_file(self) -> None:
        if (self.read_register(INTERNAL_STATUS) == 0x01):
            print(hex(self.address), " --> Initialization already done")
        else:
            print(hex(self.address), " --> Initializing...")
            self.write_register(PWR_CONF, 0x00)
            sleep(0.00045)
            self.write_register(INIT_CTRL, 0x00)
            for i in range(256):
                self.write_register(INIT_ADDR_0, 0x00)
                self.write_register(INIT_ADDR_1, i)
                self.bus.write_i2c_block_data(self.address, INIT_DATA, bmi270_config_file[i*32:(i+1)*32])
                sleep(0.000020)
            self.write_register(INIT_CTRL, 0x01)
            sleep(0.02)
        print(hex(self.address), " --> Initialization status: " +
              '{:08b}'.format(self.read_register(INTERNAL_STATUS)) +
              "\t(00000001 --> OK)")

    def set_mode(self, mode="performance") -> None:
        if (mode == "low_power"):
            self.write_register(PWR_CTRL, 0x04)
            self.write_register(ACC_CONF, 0x17)
            self.write_register(GYR_CONF, 0x28)
            self.write_register(PWR_CONF, 0x03)
            self.acc_odr = 50
            self.gyr_odr = 100
            print(hex(self.address), " --> Mode set to: LOW_POWER_MODE")
        elif (mode == "normal"):
            self.write_register(PWR_CTRL, 0x0E)
            self.write_register(ACC_CONF, 0xA8)
            self.write_register(GYR_CONF, 0xA9)
            self.write_register(PWR_CONF, 0x02)
            self.acc_odr = 100
            self.gyr_odr = 200
            print(hex(self.address), " --> Mode set to: NORMAL_MODE")
        elif (mode == "performance"):
            self.write_register(PWR_CTRL, 0x0E)
            self.write_register(ACC_CONF, 0xA8)
            self.write_register(GYR_CONF, 0xE9)
            self.write_register(PWR_CONF, 0x02)
            self.acc_odr = 100
            self.gyr_odr = 200
            print(hex(self.address), " --> Mode set to: PERFORMANCE_MODE")
        else:
            print("Wrong mode. Use 'low_power', 'normal' or 'performance'")

    def get_sensor_time(self) -> int:
        sensortime_0 = self.read_register(SENSORTIME_0)
        sensortime_1 = self.read_register(SENSORTIME_1)
        sensortime_2 = self.read_register(SENSORTIME_2)
        return (sensortime_2 << 16) | (sensortime_1 << 8) | sensortime_0

    def get_raw_acc_data(self) -> np.ndarray:
        acc_value_x_lsb = self.read_register(ACC_X_7_0)
        acc_value_x_msb = self.read_register(ACC_X_15_8)
        acc_value_x = (acc_value_x_msb << 8) | acc_value_x_lsb

        acc_value_y_lsb = self.read_register(ACC_Y_7_0)
        acc_value_y_msb = self.read_register(ACC_Y_15_8)
        acc_value_y = (acc_value_y_msb << 8) | acc_value_y_lsb

        acc_value_z_lsb = self.read_register(ACC_Z_7_0)
        acc_value_z_msb = self.read_register(ACC_Z_15_8)
        acc_value_z = (acc_value_z_msb << 8) | acc_value_z_lsb

        return np.array([acc_value_x, acc_value_y, acc_value_z]).astype(np.int16)

    def get_raw_gyr_data(self) -> np.ndarray:
        gyr_value_x_lsb = self.read_register(GYR_X_7_0)
        gyr_value_x_msb = self.read_register(GYR_X_15_8)
        gyr_value_x = (gyr_value_x_msb << 8) | gyr_value_x_lsb

        gyr_value_y_lsb = self.read_register(GYR_Y_7_0)
        gyr_value_y_msb = self.read_register(GYR_Y_15_8)
        gyr_value_y = (gyr_value_y_msb << 8) | gyr_value_y_lsb

        gyr_value_z_lsb = self.read_register(GYR_Z_7_0)
        gyr_value_z_msb = self.read_register(GYR_Z_15_8)
        gyr_value_z = (gyr_value_z_msb << 8) | gyr_value_z_lsb

        return np.array([gyr_value_x, gyr_value_y, gyr_value_z]).astype(np.int16)

    def get_raw_temp_data(self) -> int:
        temp_value_lsb = self.read_register(TEMP_7_0)
        temp_value_msb = self.read_register(TEMP_15_8)
        temp_value = (temp_value_msb << 8) | temp_value_lsb
        return self.__unsignedToSigned__(temp_value, 2)

    def get_acc_data(self) -> np.ndarray:
        raw_acc_data = self.get_raw_acc_data()
        acceleration = raw_acc_data / 40000 * self.acc_range  # in m/s²
        return acceleration

    def get_gyr_data(self) -> np.ndarray:
        raw_gyr_data = self.get_raw_gyr_data()
        angular_velocity = np.deg2rad(1) * raw_gyr_data / 40000 * self.gyr_range  # in rad/s
        return angular_velocity

    def get_temp_data(self) -> float:
        raw_data = self.get_raw_temp_data()
        temp_celsius = raw_data * 0.001952594 + 23.0
        return temp_celsius

###############################################################################
# Global Variables and Functions for GUI, Recording, and OLED Display
###############################################################################
recording_flag = False
recording_start_time = None
trial_data = []  # Sensor data samples

# Global OLED object; will be initialized later.
oled = None

def record_data_thread(sensor, start_button, stop_button, status_label, folder_entry, naming_entry, duration_entry):
    global recording_flag, recording_start_time, trial_data

    # Retrieve GUI inputs.
    folder_name = folder_entry.get().strip() or "INSERT FOLDER NAME"
    naming_convention = naming_entry.get().strip() or "NAMING CONVENTION"
    duration_input = duration_entry.get().strip()
    try:
        record_duration = float(duration_input) if duration_input != "" else 0
    except ValueError:
        record_duration = 0

    # Create the folder if it doesn't exist.
    if not os.path.exists(folder_name):
        os.makedirs(folder_name)

    # Set recording globals.
    recording_start_time = time()
    recording_flag = True
    trial_data = []  # Reset data
    sample_interval = 0.01  # 100 Hz sampling rate

    status_label.config(text="Recording started...")
    while recording_flag:
        current_time = time() - recording_start_time
        acc = sensor.get_acc_data()  # numpy array [acc_x, acc_y, acc_z]
        gyr = sensor.get_gyr_data()  # numpy array [gyr_x, gyr_y, gyr_z]
        trial_data.append([current_time, acc[0], acc[1], acc[2], gyr[0], gyr[1], gyr[2]])
        sleep(sample_interval)
        
        # Stop recording automatically if record duration is exceeded.
        if record_duration > 0 and current_time >= record_duration:
            recording_flag = False

    # Determine next available trial number.
    pattern = os.path.join(folder_name, f"{naming_convention}_*.csv")
    existing_files = glob.glob(pattern)
    if existing_files:
        nums = []
        for f in existing_files:
            try:
                base = os.path.basename(f)
                num_part = base.split('_')[-1].split('.')[0]
                nums.append(int(num_part))
            except ValueError:
                continue
        trial_num = max(nums) + 1 if nums else 1
    else:
        trial_num = 1

    csv_filename = os.path.join(folder_name, f"{naming_convention}_{trial_num}.csv")
    # Save the CSV file.
    with open(csv_filename, "w", newline="") as csvfile:
        writer = csv.writer(csvfile)
        writer.writerow(["Time (s)", "Acc_X (m/s²)", "Acc_Y (m/s²)", "Acc_Z (m/s²)",
                         "Gyr_X (rad/s)", "Gyr_Y (rad/s)", "Gyr_Z (rad/s)"])
        writer.writerows(trial_data)
    print(f"Trial data saved to {csv_filename}")

    # Create and save the plot.
    times = [row[0] for row in trial_data]
    acc_x = [row[1] for row in trial_data]
    acc_y = [row[2] for row in trial_data]
    acc_z = [row[3] for row in trial_data]
    gyr_x = [row[4] for row in trial_data]
    gyr_y = [row[5] for row in trial_data]
    gyr_z = [row[6] for row in trial_data]

    fig, axs = plt.subplots(2, 1, figsize=(10, 8))
    # Plot acceleration data.
    axs[0].plot(times, acc_x, label="Acc X")
    axs[0].plot(times, acc_y, label="Acc Y")
    axs[0].plot(times, acc_z, label="Acc Z")
    axs[0].set_title("Acceleration")
    axs[0].set_xlabel("Time (s)")
    axs[0].set_ylabel("G (m/s²)")
    axs[0].legend()

    # Plot gyroscope data.
    axs[1].plot(times, gyr_x, label="Gyr X")
    axs[1].plot(times, gyr_y, label="Gyr Y")
    axs[1].plot(times, gyr_z, label="Gyr Z")
    axs[1].set_title("Gyroscope")
    axs[1].set_xlabel("Time (s)")
    axs[1].set_ylabel("rad/s")
    axs[1].legend()
    
    # Calculate average acceleration for each axis.
    mean_Gx = statistics.mean(acc_x)
    mean_Gy = statistics.mean(acc_y)
    mean_Gz = statistics.mean(acc_z)
    total_G = (mean_Gx + mean_Gy + mean_Gz)
    
    print(f"Mean G value for x = {mean_Gx}")
    print(f"Mean G value for y = {mean_Gy}")
    print(f"Mean G value for z = {mean_Gz}")
    print(f"Mean G value for xyz = {total_G}")

    plt.tight_layout()
    plot_filename = os.path.join(folder_name, f"{naming_convention}_{trial_num}.png")
    fig.savefig(plot_filename)
    plt.close(fig)
    print(f"Plot saved to {plot_filename}")

    status_label.config(text=f"Trial {trial_num} complete and saved in folder '{folder_name}'.")
    start_button.config(state='normal')
    stop_button.config(state='disabled')

def start_recording(sensor, start_button, stop_button, status_label, folder_entry, naming_entry, duration_entry):
    """Starts recording in a separate thread."""
    start_button.config(state='disabled')
    stop_button.config(state='normal')
    threading.Thread(
        target=record_data_thread,
        args=(sensor, start_button, stop_button, status_label, folder_entry, naming_entry, duration_entry),
        daemon=True
    ).start()

def stop_recording():
    """Sets the flag to stop recording."""
    global recording_flag
    recording_flag = False

def update_timer():
    """Update the live timer label and OLED display every 100 ms."""
    global recording_flag, recording_start_time
    if recording_flag and recording_start_time is not None:
        elapsed = time() - recording_start_time
        timer_text = f"Recording: {elapsed:.1f}s"
        timer_label.config(text=timer_text)
        
        # Update the OLED display if connected.
        if oled is not None and oled.connected:
            oled.clear(oled.PAGE)
            oled.set_cursor(0, 0)
            oled.print(timer_text)
            oled.display()
    else:
        timer_label.config(text="Not recording")
        if oled is not None and oled.connected:
            oled.clear(oled.PAGE)
            oled.display()
    
    root.after(100, update_timer)

###############################################################################
# Main Program: Initialize IMU, OLED, and Launch the GUI
###############################################################################
if __name__ == "__main__":
    SENSOR_I2C_ADDRESS = 0x68  # Replace with your sensor's I2C address

    try:
        # --------------------------
        # IMU Initialization Section
        # --------------------------
        print("Initializing BMI270 IMU...")
        sensor = BMI270(SENSOR_I2C_ADDRESS)
        sensor.load_config_file()
        sensor.set_mode("performance")
        # Optionally, print a sample reading.
        print("Acceleration Data (m/s²):", sensor.get_acc_data())
        print("Gyroscope Data (rad/s):", sensor.get_gyr_data())
        print("Temperature (°C):", sensor.get_temp_data())
    except Exception as e:
        print("An error occurred during IMU initialization:", e)
        sys.exit(1)

    # --------------------------
    # OLED Initialization Section
    # --------------------------
    oled = qwiic_oled_display.QwiicOledDisplay()
    if not oled.connected:
        print("OLED not connected. Please check your wiring.", file=sys.stderr)
    else:
        oled.begin()
        # Clear the OLED display initially.
        oled.clear(oled.ALL)
        oled.display()

    # --------------------------
    # GUI Setup Section
    # --------------------------
    root = tk.Tk()
    root.title("IMST GUI")
    root.configure(background='#ADD8E6')  # Light blue background.

    # Configure ttk style.
    style = ttk.Style(root)
    style.theme_use('clam')
    style.configure('TFrame', background='#ADD8E6')
    style.configure('TLabel', background='#ADD8E6', font=('Helvetica', 10))
    style.configure('TButton', font=('Helvetica', 10))
    style.configure('TEntry', font=('Helvetica', 10))

    main_frame = ttk.Frame(root, padding=20)
    main_frame.pack(fill='both', expand=True)

    status_label = ttk.Label(main_frame, text="Enter folder name, naming convention and record duration then press 'Start Recording'.")
    status_label.pack(pady=(0, 10))

    # Folder name field.
    folder_frame = ttk.Frame(main_frame)
    folder_frame.pack(pady=(0, 5), fill='x')
    folder_label = ttk.Label(folder_frame, text="Folder Name:")
    folder_label.pack(side='left')
    folder_entry = ttk.Entry(folder_frame)
    folder_entry.pack(side='left', expand=True, fill='x', padx=(5, 0))
    folder_entry.insert(0, "Demo")

    # Naming convention field.
    naming_frame = ttk.Frame(main_frame)
    naming_frame.pack(pady=(0, 5), fill='x')
    naming_label = ttk.Label(naming_frame, text="Naming Convention:")
    naming_label.pack(side='left')
    naming_entry = ttk.Entry(naming_frame)
    naming_entry.pack(side='left', expand=True, fill='x', padx=(5, 0))
    naming_entry.insert(0, "Trial")

    # Record duration field.
    duration_frame = ttk.Frame(main_frame)
    duration_frame.pack(pady=(0, 10), fill='x')
    duration_label = ttk.Label(duration_frame, text="Record Duration (s):")
    duration_label.pack(side='left')
    duration_entry = ttk.Entry(duration_frame)
    duration_entry.pack(side='left', expand=True, fill='x', padx=(5, 0))
    duration_entry.insert(0, "30")

    # Live timer label.
    timer_label = ttk.Label(main_frame, text="Not recording")
    timer_label.pack(pady=(0, 10))

    # Buttons frame.
    button_frame = ttk.Frame(main_frame)
    button_frame.pack(pady=(0, 10))
    start_button = ttk.Button(
        button_frame,
        text="Start Recording",
        command=lambda: start_recording(sensor, start_button, stop_button, status_label, folder_entry, naming_entry, duration_entry)
    )
    start_button.pack(side='left', padx=5)

    stop_button = ttk.Button(
        button_frame,
        text="Stop Recording",
        command=stop_recording,
        state='disabled'
    )
    stop_button.pack(side='left', padx=5)

    # Start timer updates.
    update_timer()

    # Launch the GUI event loop.
    root.mainloop()

