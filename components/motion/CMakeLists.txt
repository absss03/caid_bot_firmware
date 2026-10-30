idf_component_register(
    SRCS "src/motion_control.c" "src/servo_control.c"
    INCLUDE_DIRS "include"
    PRIV_REQUIRES board drivers freertos
)