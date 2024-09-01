# DTL

## Pinout 
PA0 - LDR\
PA1 - Raindrop sensor\
PB7 - TX for FS1008A\
PA10 - SDA for BMP180\
PA9 - SCL for BMP180\
PA2 - Pin for DHT11

## General description
Transmitter is in a remote localtion collecting weather data. The receiver(Arduino) takes this data and transmits it to a computer using UART. This data is parsed using pySerial and written to a CSV file. Do change the port name based on your system. The wireless transmission uses ASK/On Off Shift Keying and USART for communication.
