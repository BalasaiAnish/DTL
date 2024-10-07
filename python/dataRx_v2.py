import time

def read_csv(file_name):
    """Reads the CSV file and returns its contents as a list of rows."""
    try:
        with open(file_name, mode='r') as file:
            header = file.readline().strip()  # Read and skip the header
            print(f"Header: {header}")  # Debug print to show that the header is read
            
            rows = []  # Initialize a list to store the rows
            for line in file:
                row = line.strip().split(',')  # Split the line into columns
                rows.append(row)  # Append the row to the list

        return rows  # Return the list of rows
    except FileNotFoundError:
        print(f"The file {file_name} does not exist.")
        return []
    except Exception as e:
        print(f"An error occurred while reading the CSV file: {e}")
        return []

def print_row(row, row_number):
    """Prints a single row of data."""
    if len(row) < 8:
        print(f"Incomplete row at row {row_number}: {row}")
        return  # Skip incomplete rows

    # Assign values from the row
    # timestamp = row[0]
    press = float(row[1])
    dht_temp = float(row[2])
    dht_hum = float(row[3])
    raindrops_voltage = int(row[4])
    raindrops_category = row[5]
    ldr_voltage = int(row[6])
    ldr_category = row[7]

    # Print the data similar to the terminal output in the main script, with row number
    # print(f"Row {row_number}:")
    # print(f"Timestamp: {timestamp}")
    print(f"Pressure: {press:.2f} Pa")
    print(f"DHT11 Temperature: {dht_temp:.2f} °C")
    print(f"DHT11 Humidity: {dht_hum:.2f} %")
    print(f"Raindrops Voltage: {raindrops_voltage} mV ({raindrops_category})")
    print(f"LDR Voltage: {ldr_voltage} mV ({ldr_category})")
    print("\n\n\n")

# Example usage: periodically read and print the CSV data
csv_file = 'sensor_data.csv'
rows = read_csv(csv_file)  # Read the CSV file

# Print each row one by one with a delay
for index, row in enumerate(rows, start=1):
    print_row(row, index)  # Print the current row
    time.sleep(10)  # Wait for 3 seconds
