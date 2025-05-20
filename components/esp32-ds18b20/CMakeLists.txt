set(COMPONENT_ADD_INCLUDEDIRS include)
set(COMPONENT_SRCS "ds18b20.c")
set(COMPONENT_REQUIRES "driver" "esp_timer" "esp32-owb")
register_component()
