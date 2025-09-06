import serial
import time

arduino_port = '/dev/ttyACM0'
baud_rate = 115200

ser = serial.Serial(arduino_port, baud_rate, timeout=1)
time.sleep(2)  # wait for Arduino to initialize


def send_command(cmd):
    ser.write((cmd + '\n').encode())  # send command with newline
    print(f"Starting: {cmd}")
    time.sleep(0.1)  # small delay to avoid flooding


send_command("FORWARD_LOW")
time.sleep(5)

send_command("FORWARD_HIGH")
time.sleep(10)

send_command("STOP")
time.sleep(2)

send_command("REVERSE")
time.sleep(5)

send_command("STOP")
ser.close()
