import tkinter as tk
from tkinter import ttk
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import serial
import serial.tools.list_ports
import threading
import time

# Serial Port Configuration
SERIAL_PORT = ""  # Ubah sesuai port Arduino kamu
BAUD_RATE = 0
is_connected_f = False
myserial = None

# Data Storage
data_values = []
max_points = 100

# Function to Read Serial Data
def read_serial():
    while True:
        if myserial and myserial.in_waiting:
            line = myserial.readline().decode('utf-8').strip()
            try:
                value = float(line)
                data_values.append(value)
                if len(data_values) > max_points:
                    data_values.pop(0)
            except ValueError:
                pass
        time.sleep(0.01)

# Function to Update Plot
def update_plot():
    ax.clear()
    ax.plot(data_values, label="Sensor Value")
    ax.set_title("Real-time Data from Arduino")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Value")
    ax.grid(True)
    ax.legend()
    plotArea.draw()
    root.after(50, update_plot)

def scanAvailablePort():
    ports = serial.tools.list_ports.comports()
    scanAvailablePort = []
    for port in ports:
        scanAvailablePort.append(port.device)
    return scanAvailablePort
    
def connectToPort(port, baud):
    global myserial, is_connected_f, SERIAL_PORT, BAUD_RATE
    port = port_comboBox.get()
    baud = baud_comboBox.get()
    SERIAL_PORT = port
    BAUD_RATE = baud
    try:
        myserial = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
        is_connected_f = True
    except serial.SerialException:
        print('Failed to connect to serial port.')
        myserial = None
        is_connected_f = False

# Initialize GUI
root = tk.Tk()
root.title("PID App")
screen_height = int(root.winfo_screenheight() / 1.2)
screen_width = int(root.winfo_screenwidth() / 1.2)
screen_dimension = "{}x{}".format(screen_width, screen_height)
root.geometry(screen_dimension)

# Matplotlib Figure
plotFrame_width = screen_width * 0.75;
plotFrame = tk.Frame(root, width=plotFrame_width, bg='blue')
plotFrame.pack(padx=5, pady=5, side=tk.LEFT, fill=tk.Y)

fig, ax = plt.subplots()
plotArea = FigureCanvasTkAgg(fig, master=plotFrame)
plotArea.get_tk_widget().pack(fill=tk.BOTH, expand=True)

#control panel figure
controlFrame_width = screen_width * 0.25;
controlFrame =  tk.Frame(root, width=controlFrame_width, bg='red')
controlFrame.pack(padx=5, pady=5, side=tk.RIGHT, fill=tk.Y)

port_comboBox_val = [""]
port_comboBox_val = port_comboBox_val + scanAvailablePort()

port_comboBox = ttk.Combobox(controlFrame, values=port_comboBox_val, state='readonly')
port_comboBox.set("")
port_comboBox.pack(padx=0, pady=0)

baud_comboBox_val = [
    9600,
    19200,
    38400,
    57600,
    115200
]

baud_comboBox = ttk.Combobox(controlFrame, values=baud_comboBox_val, state='readonly')
baud_comboBox.set(9600)
baud_comboBox.pack(padx=0, pady=10)

connect_btn = tk.Button(controlFrame, text="connect", command=connectToPort)
connect_btn.pack(padx=0, pady=10)

connect_status_text = "Connected" if (is_connected_f) else "Disconnected"

connect_status = tk.Label(controlFrame, text="{}".format(connect_status_text))
connect_status.pack(expand=True)

# Start Serial Reading Thread
thread = threading.Thread(target=read_serial, daemon=True)
thread.start()

# Start Plot Update
update_plot()

# Run GUI
root.mainloop()
