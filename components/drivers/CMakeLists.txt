idf_component_register(
    SRCS "src/ledc_driver.c" "src/pcnt_driver.c" "src/ultrasonic_driver.c"
    INCLUDE_DIRS "include"
    PRIV_REQUIRES board esp_driver_gpio esp_driver_ledc esp_driver_pcnt
                  esp_driver_rmt esp_timer esp_rom
)