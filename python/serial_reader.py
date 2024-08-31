import serial
import time
import struct
import csv

ser = serial.Serial('ttyACM0',9600,8,'N',1)

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

    with open('data.csv', 'a', newline='') as file:
        writer = csv.writer(file)
        writer.writerow([bmp_temp,bmp_press,dht_temp,dht_hum,ldr_voltage,raindrops_voltage])