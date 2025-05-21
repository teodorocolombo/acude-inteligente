#include "esp_log.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdio.h>

#include "ds18b20.h"
#include "owb.h"
#include "owb_rmt.h"

#define GPIO_DS18B20 (2)
#define DS18B20_RESOLUTION (DS18B20_RESOLUTION_12_BIT)
#define SAMPLE_PERIOD_MS (1000)

static const char *TAG = "temp_sensor";

static OneWireBus *owb;
static DS18B20_Info *ds18b20;
static owb_rmt_driver_info rmt_driver_info;

void temp_sensor_init() {
    vTaskDelay(2000 / portTICK_PERIOD_MS);

    owb = owb_rmt_initialize(&rmt_driver_info, GPIO_DS18B20, RMT_CHANNEL_0, RMT_CHANNEL_1);

    if (owb == NULL) {
        ESP_LOGE(TAG, "Failed to initialize OneWire bus.");
        return;
    }

    owb_use_crc(owb, true);

    OneWireBus_SearchState search_state = {0};
    bool found = false;
    owb_search_first(owb, &search_state, &found);

    if (!found) {
        ESP_LOGE(TAG, "No DS18B20 devices found!");
        return;
    }

    char rom_code_s[OWB_ROM_CODE_STRING_LENGTH];
    owb_string_from_rom_code(search_state.rom_code, rom_code_s,
                             sizeof(rom_code_s));
    ESP_LOGI(TAG, "Found DS18B20 device: %s", rom_code_s);

    ds18b20 = ds18b20_malloc();
    ds18b20_init_solo(ds18b20, owb);
    ds18b20_use_crc(ds18b20, true);
    ds18b20_set_resolution(ds18b20, DS18B20_RESOLUTION);

    bool parasitic = false;
    ds18b20_check_for_parasite_power(owb, &parasitic);
    owb_use_parasitic_power(owb, parasitic);
    ESP_LOGI(TAG, "Parasitic power: %s", parasitic ? "YES" : "NO");
}

DS18B20_ERROR temp_sensor_read(float *value) {
    if (!ds18b20) {
        ESP_LOGE(TAG, "Sensor not initialized.");
        return DS18B20_ERROR_UNKNOWN;
    }

    ds18b20_convert_all(owb);

    const float waited = ds18b20_wait_for_conversion(ds18b20);
    ESP_LOGD(TAG, "Waited %.1f ms", waited);

    const DS18B20_ERROR err = ds18b20_read_temp(ds18b20, value);
    if (err != DS18B20_OK) {
        ESP_LOGE(TAG, "Failed to read temperature: %d", err);
        return err;
    }

    return DS18B20_OK;
}
