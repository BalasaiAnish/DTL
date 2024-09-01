import serial
import time
import struct
import csv
import random
from datetime import datetime

with open('./data.csv', 'r', newline='') as file:
    empty = 1 if len(file.read())==0 else 0

if(empty):
    with open('./data.csv', 'w', newline='') as file:
        writer = csv.writer(file)
        writer.writerow(["Timestamp","Temperature from BMP","Pressure from BMP","Temperature from DHT11","Humidity from DHT11","Brightness","Rain"])


timestamp = str(datetime.now())
bmp_temp = 24.0 + 2*(random.random()-0.5)
bmp_press = 0.9 + 0.01*(random.random()-0.5)
dht_temp = 24.0 + 4*(random.random()-0.5)
dht_hum = str(80.0 + 0.5*(random.random()-0.5)) + '%'
brightness = "Sunny"
rain = "Clear"

with open('./data.csv', 'a', newline='\n') as file:
    writer = csv.writer(file)
    writer.writerow([timestamp,bmp_temp,bmp_press,dht_temp,dht_hum,brightness,rain])


