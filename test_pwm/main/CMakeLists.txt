idf_component_register(
    SRCS "pwm_test.c"
    INCLUDE_DIRS "."
    REQUIRES esp_driver_ledc esp_driver_gpio
)
