idf_component_register(
    SRCS "src/hardware_init.c"
    INCLUDE_DIRS "include"
    PRIV_REQUIRES esp_driver_gpio
)