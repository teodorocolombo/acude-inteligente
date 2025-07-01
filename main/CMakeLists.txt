set(COMPONENT_SRCS "main.c")
set(COMPONENT_REQUIRES "telegram-sender" "temp-sensor" "wifi-station" "sct013" "pushbutton" "relay" nvs_flash)
register_component()
