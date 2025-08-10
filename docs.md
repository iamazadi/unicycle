In order to enable floating points using the sprintf function, do the following steps:
1. Open the file "gcc-arm-none-eabi.cmake" that is created by CubeMX.
2. Add the option "-u _printf_float" to "CMAKE_C_FLAGS".

Set the baudrate of uart6 to 921600, for the wifi module (HC25).
Print uart6 serial messages bu executing `nc 192.168.4.1 10000`

Set the baudrate of uart1 to 9600 for the GY-25 IMU module.

USART1 -> IMU #1
USART6 -> WiFi module

Set the Pin6 (PS: IIC/USART output mode selection) of the second GY-25 IMU to zero, in order to use the I2C protocol.

The HC-25 module settings are on 192.168.4.1 as a web page. Here is the checklist to set a new module:
1. The password is not set for brand new modules. So just login without a password to access the settings page.
2. Set a username and password for the access point.
3. Set the *WiFi Mode* to **AP**.
4. Change the port number from *8080* to *10000*.
5. Set the *Baud Rate* parameter to 921600 Bits/s.

Add a DMA request with USART1_RX and DMA2 Stream 2 from peripheral to memory and low priority. The mode is circular and the request call is made once in the main function by passing the usart1 handle and the receive buffer. The request increments the address of memory. The data width is one Byte for both the preipheral and memory.