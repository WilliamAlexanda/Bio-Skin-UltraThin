idf_component_register(SRCS "main.c" "mlx90393.c" "mux_adc.c"
                       INCLUDE_DIRS "."
                       PRIV_REQUIRES esp_driver_i2c esp_driver_gpio esp_adc esp_timer esp_driver_uart)
