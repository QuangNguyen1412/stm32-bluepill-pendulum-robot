MPU6050
Input voltage: 2.7-3.5V
I2C address:
-	AD0: 1101000
-	AD1: 1101001
I2C Freq:
-	Normal mode: 100kHz
-	Fast mode: 400kHz
Need to configure:
Gyroscope:
-	Full-scale range
Accelerometer:
-	Full-scale-range
Speed: normal mode 100kHz

Read more information about memory map for configuring and getting data of MPU6050 from this link
https://www.invensense.com/wp-content/uploads/2015/02/MPU-6000-Register-Map1.pdf
https://store.invensense.com/datasheets/invensense/MPU-6050_DataSheet_V3%204.pdf


UART
There are 2 uart ports on the device that I have already configure: uart1 (debugging) and uart2 (Bluetooth)
UART 1 debugging
-	Baud rate 115200
-	Pin connection:   
o	PA9     ------> USART1_TX
o	PA10     ------> USART1_RX
UART 2 Bluetooth
-	Baud rate 57600 (check the default baud rate of HC-10 module)
-	Pin connection:
o	    PA2     ------> USART2_TX
o	    PA3     ------> USART2_RX


Bluetooth UART 2
Module: HM-10 BLE
Send AT command through UART 2 to communicate with Bluetooth module
Interrupt routine for connection and receive command

Default baudrate: 9600
Test AT command: Send "AT", module returns "OK"

HM-10 Data sheet:
ftp://imall.iteadstudio.com/Modules/IM130614001_Serial_Port_BLE_Module_Master_Slave_HM-10/DS_IM130614001_Serial_Port_BLE_Module_Master_Slave_HM-10.pdf

Note: you will have a better view of AT command through the HM-10 document. There is some free tools from the internet that can be used to test AT command
