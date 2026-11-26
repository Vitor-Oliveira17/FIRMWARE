idf_component_register(
    SRCS "imu_test.c"
    INCLUDE_DIRS "."
    REQUIRES esp_driver_i2c esp_driver_gpio
)
