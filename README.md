# DTL

## Pinout 
A0 - LDR\
A1 - Raindrop sensor\
D4 - TX for UART\
D0 - SDA for BMP180\
D1 - SCL for BMP180\
A2 - Pin for DHT11\
A4, A5, A6 - SCK, MISO, MOSI (SPI1)\
D3 - NSS\
D6 - CE

## General description
Transmitter is in a remote location collecting weather data. The receiver takes this data and transmits it to a computer using UART. This data is parsed using pySerial and written to a CSV file. Do change the port name based on your system. The wireless transmission uses the NRF24L01 modules.

The MCU goes into sleep mode to conserve power. The low power timer LPTIM2 triggers an interrupt every 20 seconds using the LSE as the clock source source. This interrupt is used to wake up the MCU from it's sleep mode.
