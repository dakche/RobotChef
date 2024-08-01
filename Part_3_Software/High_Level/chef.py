# Robot Chef Code

# Import our Serial libraries so we can communicate with the Teensy
import serial
import serial.tools.list_ports
import numpy as np

# Find the Teensy's USB port
ports = list(serial.tools.list_ports.comports())
for p in ports:
    if "USB Serial" in p.description:
        print(p.device)
        print("Connected!")
        break

# Connect to that port
arm = serial.Serial(port = p.device, baudrate = 115200, timeout = .1)

# Poll the current arm angles
arm.write(b"p\n")

# Wait for Teensy's response
while arm.in_waiting == 0:
	continue

# Grab the data
resp = arm.readline().decode('utf-8').rstrip(";\r\n").split(";")

# Convert strings to floats
curr_angles = np.array(resp).astype(float).tolist()

# Work in progress...

# Inverse Kinematics




# command = f';;;;;\n'

# arm.write(command.encode('ascii'))
