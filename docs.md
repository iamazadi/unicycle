In order to enable floating points using the sprintf function, do the following steps:
1. Open the file "gcc-arm-none-eabi.cmake" that is created by CubeMX.
2. Add the option "-u _printf_float" to "CMAKE_C_FLAGS".

Set the baudrate of uart6 to 921600, for the wifi module (HC25).
Print uart6 serial messages bu executing `nc 192.168.4.1 10000`

Set the baudrate of uart1 to 9600 for the GY-25 IMU module.