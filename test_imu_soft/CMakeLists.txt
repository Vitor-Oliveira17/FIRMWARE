# Teste ISOLADO da IMU usando I2C POR SOFTWARE (bit-bang) nos GPIO 2 e 3.
# Nao usa o controlador I2C do ESP32-C6 nem depende do firmware principal.
cmake_minimum_required(VERSION 3.16)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(TEST_IMU_SOFT)
