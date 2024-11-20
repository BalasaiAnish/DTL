import serial
import struct
import csv
import os
from datetime import datetime

ser = serial.Serial('/dev/tty.usbmodem1103', 115200, timeout=1)

# Define thresholds
ldr_thresholds = [1500, 2000, 5000]
raindrops_thresholds = [1500, 2000, 2800]
"""
ldr_thresholds = [100, 500, 1000]
raindrops_thresholds = [50, 150, 300]
"""

def categorize_value(value, thresholds, labels):
    """Categorize a value based on thresholds and labels."""
    for i, threshold in enumerate(thresholds):
        if value <= threshold:
            return labels[i]
    return labels[-1]  # Return the last label if value exceeds all thresholds

# CSV file setup
csv_file = 'sensor_data.csv'
file_exists = os.path.isfile(csv_file)

with open(csv_file, mode='a', newline='') as file:
    csv_writer = csv.writer(file)

    # Write the header only if the file doesn't already exist
    if not file_exists:
        csv_writer.writerow(['Timestamp', 'Pressure (Pa)', 'DHT11 Temperature (°C)', 'DHT11 Humidity (%)', 'Raindrops Voltage (mV)', 'Raindrops Category', 'LDR Voltage (mV)', 'LDR Category'])

    def receive_and_parse_data():
        # Read 20 bytes from UART
        data = ser.read(20)

        if len(data) == 20:
            # Parse the first 16 bytes as 4 floats (each 4 bytes)
            _, press, dht_temp, dht_hum = struct.unpack('4f', data[:16])

            # Parse the next 4 bytes as two 16-bit unsigned integers (each 2 bytes)
            ldr_voltage, raindrops_voltage = struct.unpack('2H', data[16:20])

            ldr_voltage, raindrops_voltage = raindrops_voltage, ldr_voltage
            
            if(press == 0): return

            ldr_category = categorize_value(ldr_voltage, ldr_thresholds, ["Sunny", "Partly Cloudy", "Overcast"])
            raindrops_category = categorize_value(raindrops_voltage, raindrops_thresholds, ["No Rain", "Light Rain", "Heavy Rain"])

            timestamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S')

            print(f"Timestamp: {timestamp}")
            print(f"Pressure: {press:.2f} Pa")
            print(f"DHT11 Temperature: {dht_temp:.2f} °C")
            print(f"DHT11 Humidity: {dht_hum:.2f} %")
            print(f"{raindrops_category}")
            print(f"{ldr_category}")
            print("\n\n\n")

            csv_writer.writerow([
                timestamp,
                round(press, 2),
                round(dht_temp, 2),
                round(dht_hum, 2),
                round(raindrops_voltage, 2),
                raindrops_category,
                round(ldr_voltage, 2),
                ldr_category
            ])

    while True:
        receive_and_parse_data()
