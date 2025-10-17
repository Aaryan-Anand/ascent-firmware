idf_component_register(
    SRCS 
        "main.c"
        "../../src/memory/nvs_interface.c"
        "../../src/memory/flash_interface.c"
        "../../src/sensor/orientation.c"
        "../../src/sitl/serial_util.c"
    INCLUDE_DIRS 
        "${CMAKE_CURRENT_SOURCE_DIR}/../"
        "${CMAKE_CURRENT_SOURCE_DIR}/../../src/memory"
        "${CMAKE_CURRENT_SOURCE_DIR}/../../src/sensor"
        "${CMAKE_CURRENT_SOURCE_DIR}/../../src/sitl"
    REQUIRES esp_timer
)