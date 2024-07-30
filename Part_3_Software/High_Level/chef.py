# Robot Chef Code

# Import our Serial libraries so we can communicate with the Teensy
import serial
import serial.tools.list_ports

# Find the Teensy's USB port
ports = list(serial.tools.list_ports.comports())
for p in ports:
    if "USB Serial" in p.description:
        break

# Connect to that port
arm = serial.Serial(port = p.device, baudrate = 115200, timeout = .1)



command = f';;;;;\n'

arm.write(command.encode('ascii'))