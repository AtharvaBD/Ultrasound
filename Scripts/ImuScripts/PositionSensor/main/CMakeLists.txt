idf_component_register(
    SRCS "main.cpp" "calibration.cpp" "imu.c" "mahony.c" "position.c"
    INCLUDE_DIRS "."
    REQUIRES nvs_flash esp_driver_i2c esp_timer
)