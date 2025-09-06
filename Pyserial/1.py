import serial
import time
import os

arduino_port = '/dev/ttyACM0'
baud_rate = 115200

ser = serial.Serial(arduino_port, baud_rate, timeout=1)
time.sleep(2)  # wait for Arduino to initialize

log_file = "encoders_log.txt"

if not os.path.exists(log_file):
    with open(log_file, "w") as f:
        f.write("Encoder log started\n")


def send_command(cmd):
    ser.write((cmd + '\n').encode())  # send command with newline
    print(f"Running: {cmd}")
    time.sleep(0.1)  # avoid flooding


def log_serial():
    with open(log_file, "a") as f:
        while ser.in_waiting > 0:
            line = ser.readline().decode(errors="ignore").strip()
            if line:
                f.write(line + "\n")


send_command("FORWARD_LOW")
start_time = time.time()
while time.time() - start_time < 5:
    log_serial()

send_command("FORWARD_HIGH")
start_time = time.time()
while time.time() - start_time < 5:
    log_serial()

send_command("STOP")
time.sleep(2)

send_command("REVERSE")
start_time = time.time()
while time.time() - start_time < 5:
    log_serial()

send_command("STOP")
time.sleep(2)

ser.close()
