import serial
import time
import struct
import csv
from datetime import datetime

with open('./data.csv', 'r', newline='') as file:
    if len(file.read()) == 0:
        writer = csv.writer(file)
        writer.writerow(["Timestamp","Temperature from BMP","Pressure from BMP","Temperature from DHT11","Humidity from DHT11","Brightness","Rain"])

ser = serial.Serial('/dev/ttyACM0',9600,8,'N',1)

time.sleep(3)

ser.open()

if ser.is_open():
    ser.flush()
    receive_buffer = ser.read(20)
    bmp_temp = struct.unpack('f',receive_buffer[:4])
    bmp_press = struct.unpack('f',receive_buffer[4:8])
    dht_temp = struct.unpack('f',receive_buffer[8:12])
    dht_hum = struct.unpack('f',receive_buffer[12:16])
    ldr_voltage = struct.unpack('<h',receive_buffer[16:18])*3.3/4095
    raindrops_voltage = struct.unpack('<h',receive_buffer[18:20])*3.3/4095

    if(ldr_voltage>2.2):
        brightness = "Sunny"

    elif(ldr_voltage>1.1):
        brightness = "Partly cloudy"

    else:
        brightness = "Cloudy"

    if(raindrops_voltage>2.2):
        rain = "Rainy"

    elif(raindrops_voltage>1.1):
        rain = "Drizzling"

    else:
        rain = "Clear"

    timestamp = str(datetime.now())

    with open('data.csv', 'a', newline='') as file:
        writer = csv.writer(file)
        writer.writerow([timestamp,bmp_temp,bmp_press,dht_temp,dht_hum,brightness,rain])
