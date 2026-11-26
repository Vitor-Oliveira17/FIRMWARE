idf_component_register(
    SRCS "imu_soft_test.c" "soft_i2c.c"
    INCLUDE_DIRS "."
    REQUIRES esp_driver_gpio esp_rom
)
