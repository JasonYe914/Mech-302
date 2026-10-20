import serial
import csv
import time
import os

# === SETTINGS ===
arduino_port = "COM4"       # Update if needed
baud_rate = 9600            # Must match Arduino code
output_file = "lab41.csv" 

# === CONNECT ===
try:
    ser = serial.Serial(arduino_port, baud_rate, timeout=1)
    time.sleep(2)  # Wait for Arduino to reset
    print(f"Connected to {arduino_port} at {baud_rate} baud.")
except serial.SerialException as e:
    print(f"Error connecting to {arduino_port}: {e}")
    exit()

# === PREPARE FILE ===
# Open in append mode if you want to keep existing data
file_exists = os.path.isfile(output_file)
with open(output_file, mode='a', newline='') as file:
    writer = csv.writer(file)

    # Write header only once (if file is new)
        # Write header only once (if file is new)
    if not file_exists:
        writer.writerow(["Time", "Temperature (°C)"])

    print("Recording data... Press Ctrl+C to stop.\n")

    try:
        while True:
            if ser.in_waiting > 0:
                line = ser.readline().decode('utf-8', errors='ignore').strip()
                if line:
                    print(line)
                    # Split incoming comma-separated data and write each field to its own column
                    writer.writerow(line.split(","))
    except KeyboardInterrupt:
        print("\nData recording stopped.")
    finally:
        ser.close()
        print("Serial connection closed.")
