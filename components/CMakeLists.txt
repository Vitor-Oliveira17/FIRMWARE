# components/CMakeLists.txt
idf_component_register(
    SRCS "src/imu.c" "src/motor.c" "src/encoder_magnetic.c" "src/state_controller.c"
    INCLUDE_DIRS "inc"
    REQUIRES driver esp_driver_i2c esp_driver_ledc esp_driver_gpio esp_timer
)