import serial
import struct
import csv
import os
from datetime import datetime

# Open the serial port (adjust the port name as necessary)
ser = serial.Serial('/dev/tty.usbmodem11403', 9600, timeout=1)

# Define thresholds
# Example thresholds (adjust these values based on your requirements)
ldr_thresholds = [100, 500, 1000]  # Example LDR thresholds for "Sunny", "Partly Cloudy", "Overcast"
raindrops_thresholds = [50, 150, 300]  # Example Raindrops thresholds for "No Rain", "Light Rain", "Heavy Rain"

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

            # Switch LDR and Raindrops values
            ldr_voltage, raindrops_voltage = raindrops_voltage, ldr_voltage

            # Categorize LDR and Raindrops values
            ldr_category = categorize_value(ldr_voltage, ldr_thresholds, ["Sunny", "Partly Cloudy", "Overcast"])
            raindrops_category = categorize_value(raindrops_voltage, raindrops_thresholds, ["No Rain", "Light Rain", "Heavy Rain"])

            # Get the current timestamp
            timestamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S')

            # Print the parsed values (optional)
            print(f"Timestamp: {timestamp}")
            print(f"Pressure: {press:.2f} Pa")
            print(f"DHT11 Temperature: {dht_temp:.2f} °C")
            print(f"DHT11 Humidity: {dht_hum:.2f} %")
            print(f"Raindrops Voltage: {raindrops_voltage} mV ({raindrops_category})")
            print(f"LDR Voltage: {ldr_voltage} mV ({ldr_category})")

            # Write the timestamp and data to the CSV file
            csv_writer.writerow([timestamp, press, dht_temp, dht_hum, raindrops_voltage, raindrops_category, ldr_voltage, ldr_category])

    # Example usage: continuously receive and parse data
    while True:
        receive_and_parse_data()
