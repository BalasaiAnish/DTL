# DTL

## Pinout 
A0 - Raindrop sensor\
A1 - LDR\
D4 - TX for UART\
D0 - BMP_SDA\
D1 - BMP_SCL\
D9 - Pin for DHT11\
D13, D12, D11 - NRF_SCK, NRF_MISO, NRF_MOSI (SPI3)\
A3 - CSN\
A5 - CE

## General description
Transmitter is in a remote location collecting weather data. The receiver takes this data and transmits it to a computer using UART. This data is parsed using pySerial and written to a CSV file. Do change the port name based on your system. The wireless transmission uses the NRF24L01 modules.

The MCU goes into sleep mode to conserve power. The low power timer LPTIM2 triggers an interrupt every 20 seconds using the LSE as the clock source source. This interrupt is used to wake up the MCU from it's sleep mode.
