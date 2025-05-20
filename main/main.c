#include <string.h>

#include "esp_event.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "nvs_flash.h"
#include "lwip/err.h"
#include "lwip/sys.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "temp_sensor.h"
#include "telegram_sender.h"
#include "wifi_station.h"

/* START DEFINE */
#define DEFAULT_TASK_STACK_SIZE_BYTES 4096
#define DEFAULT_QUEUE_SIZE 10
#define FLOAT_ITEM_SIZE sizeof(float)

/* Timer interval once every day (24 Hours) */
#define TIME_PERIOD (86400000000ULL)

/* START ReadTemperatureSensorTask */
StackType_t xReadTemperatureSensorStack[DEFAULT_TASK_STACK_SIZE_BYTES];
StaticTask_t xReadTemperatureSensorBuffer;
TaskHandle_t xReadTemperatureSensorHandle;

void vReadTemperatureSensorTask(void *pvParameters);

/* START WebhookPublisherTask */
StackType_t xWebhookPublisherTaskStack[DEFAULT_TASK_STACK_SIZE_BYTES];
StaticTask_t xWebhookPublisherTaskBuffer;
TaskHandle_t xWebhookPublisherTaskHandle;

void vWebhookPublisherTask(void *pvParameters);

/* START TemperatureSensorQueue */

QueueHandle_t xTemperatureSensorQueueHandle;
uint8_t xTemperatureSensorQueueStorageArea[DEFAULT_QUEUE_SIZE * FLOAT_ITEM_SIZE];
StaticQueue_t xTemperatureSensorQueueBuffer;

/* COMMONS */
static const char *TAG = "main";

void create_tasks();

void create_queues();

uint8_t float_to_char(const float *number, char *buffer, size_t buffer_size);

void initialize_nvs();

void app_main() {
    initialize_nvs();
    wifi_init_sta();
    ESP_LOGI(TAG, "Initializing Tasks");
    create_tasks();
    create_queues();
}

void initialize_nvs() {
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
}

void create_tasks() {
    xReadTemperatureSensorHandle = xTaskCreateStatic(vReadTemperatureSensorTask,
                                                     "ReadTemperatureSensorTask",
                                                     DEFAULT_TASK_STACK_SIZE_BYTES,
                                                     NULL,
                                                     tskIDLE_PRIORITY,
                                                     xReadTemperatureSensorStack,
                                                     &xReadTemperatureSensorBuffer);
    xReadTemperatureSensorHandle = xTaskCreateStatic(vWebhookPublisherTask,
                                                     "WebhookPublisherTask",
                                                     DEFAULT_TASK_STACK_SIZE_BYTES,
                                                     NULL,
                                                     tskIDLE_PRIORITY,
                                                     xWebhookPublisherTaskStack,
                                                     &xWebhookPublisherTaskBuffer);
}

void create_queues() {
    xTemperatureSensorQueueHandle = xQueueCreateStatic(DEFAULT_QUEUE_SIZE,
                                                       FLOAT_ITEM_SIZE,
                                                       &xTemperatureSensorQueueStorageArea[0],
                                                       &xTemperatureSensorQueueBuffer);
}

void vReadTemperatureSensorTask(void *pvParameters) {
    ESP_LOGI(TAG, "Starting vReadTemperatureSensorTask");
    temp_sensor_init();
    float temperature = 0.0f;
    while (true) {
        if (temp_sensor_read(&temperature) == DS18B20_OK) {
            ESP_LOGI(TAG, "Temperature: %.2f °C", temperature);
            xQueueSend(xTemperatureSensorQueueHandle, &temperature, portMAX_DELAY);
        }
        vTaskDelay(2000 / portTICK_PERIOD_MS);
    }
}

void vWebhookPublisherTask(void *pvParameters) {
    float temperature;
    char buffer[32];
    char message[64];
    int bufferCleaner;
    while (1) {
        if (xQueueReceive(xTemperatureSensorQueueHandle, &temperature, 100 / portTICK_PERIOD_MS) == pdFALSE) {
            continue;
        }

        if (float_to_char(&temperature, buffer, sizeof(buffer)) != EXIT_SUCCESS) {
            ESP_LOGE(TAG, "Erro ao converter float para string!!");
            continue;
        }

        strcpy(message, "Temperatura: ");
        strcat(message, buffer);
        strcat(message, "°C");

        ESP_LOGI(TAG, "%s", message);

        send_to_telegram(message);

        while ((bufferCleaner = getchar()) != '\n' && bufferCleaner != EOF) {
        }

        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

uint8_t float_to_char(const float *number, char *buffer, const size_t buffer_size) {
    const int ret = snprintf(buffer, buffer_size, "%.1f", *number);

    if (ret < 0 || (size_t) ret >= buffer_size) {
        return EXIT_FAILURE;
    }
    return EXIT_SUCCESS;
}
