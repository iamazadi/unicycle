In order to enable floating points using the sprintf function, do the following steps:
1. Open the file "gcc-arm-none-eabi.cmake" that is created by CubeMX.
2. Add the option "-u _printf_float" to "CMAKE_C_FLAGS".