idf_component_register(
    SRCS "main.c" "core_zero.c" "core_one.c"
    INCLUDE_DIRS "include"
    REQUIRES board drivers motion micro_ros_espidf_component esp_timer
)

target_compile_definitions(${COMPONENT_LIB} PRIVATE ESP_CAID_ENABLE_MICROROS=1)