# DTL

## Pinout 
PA0 - LDR\
PA1 - Raindrop sensor\
PB7 - TX for FS100A\
PA10 - SDA for BMP180\
PA9 - SCL for BMP180\
PA2 - Pin for DHT11

## General description
Transmitter is in a remote location collecting weather data. The receiver(Arduino) takes this data and transmits it to a computer using UART. This data is parsed using pySerial and written to a CSV file. Do change the port name based on your system. The wireless transmission uses ASK/On Off Shift Keying and USART for communication. 

Please note that the MCU goes into sleep mode to conserve power. The low power timer LPTIM2 triggers an interrupt every 20 seconds using the LSE as a source. This interrupt is used to wake up the MCU from it's sleep mode.
