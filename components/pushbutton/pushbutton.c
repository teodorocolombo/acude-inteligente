#include "pushbutton.h"
#include "esp_log.h"

static const char *TAG = "pushbutton";


esp_err_t init_shutdown_isr(gpio_num_t gpio_num, gpio_isr_t isr_handler_func, void *isr_args) {
    esp_err_t ret = ESP_OK;

    if (isr_handler_func == NULL) {
        ESP_LOGE(TAG, "ISR handler function cannot be NULL.");
        return ESP_ERR_INVALID_ARG;
    }

    ESP_LOGI(TAG, "Initializing button on GPIO %d with internal pull-up and custom ISR...", gpio_num);

    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_NEGEDGE;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = (1ULL << gpio_num);
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
    ret = gpio_config(&io_conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure GPIO %d: %s", gpio_num, esp_err_to_name(ret));
        return ret;
    }


    ret = gpio_install_isr_service(ESP_INTR_CPU_AFFINITY_AUTO);
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "Failed to install GPIO ISR service: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = gpio_isr_handler_add(gpio_num, isr_handler_func, isr_args);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add ISR handler for GPIO %d: %s", gpio_num, esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "Button initialization complete. Custom ISR hooked to GPIO %d.", gpio_num);
    return ESP_OK;
}
