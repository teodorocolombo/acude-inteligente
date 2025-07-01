#include "relay.h"
#include "driver/gpio.h"
#include "esp_log.h"

#define RELAY_GPIO_PIN GPIO_NUM_16

#define RELAY_ACTIVE_LEVEL 0
#define RELAY_INACTIVE_LEVEL 1

static const char *TAG = "relay";

esp_err_t relay_init(void) {
    esp_err_t ret = ESP_OK;

    ESP_LOGI(TAG, "Initializing relay on GPIO %d...", RELAY_GPIO_PIN);

    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = (1ULL << RELAY_GPIO_PIN);
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    ret = gpio_config(&io_conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure GPIO %d for relay: %s", RELAY_GPIO_PIN, esp_err_to_name(ret));
        return ret;
    }

    ret = gpio_set_level(RELAY_GPIO_PIN, RELAY_INACTIVE_LEVEL);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set initial level for relay on GPIO %d: %s", RELAY_GPIO_PIN, esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "Relay initialized on GPIO %d (initial state: %s)", RELAY_GPIO_PIN,
             (RELAY_INACTIVE_LEVEL == 0) ? "LOW/Deactivated" : "HIGH/Deactivated");
    return ESP_OK;
}

esp_err_t relay_active(void) {
    esp_err_t ret = gpio_set_level(RELAY_GPIO_PIN, RELAY_ACTIVE_LEVEL);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Relay activated on GPIO %d", RELAY_GPIO_PIN);
    } else {
        ESP_LOGE(TAG, "Failed to activate relay on GPIO %d: %s", RELAY_GPIO_PIN, esp_err_to_name(ret));
    }
    return ret;
}

esp_err_t relay_desactive(void) {
    esp_err_t ret = gpio_set_level(RELAY_GPIO_PIN, RELAY_INACTIVE_LEVEL);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Relay deactivated on GPIO %d", RELAY_GPIO_PIN);
    } else {
        ESP_LOGE(TAG, "Failed to deactivate relay on GPIO %d: %s", RELAY_GPIO_PIN, esp_err_to_name(ret));
    }
    return ret;
}
