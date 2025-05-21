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
#define WORD_SIZE_IN_BYTES 4
#define DEFAULT_TASK_STACK_SIZE_BYTES 4096
#define DEFAULT_QUEUE_SIZE 10
#define FLOAT_ITEM_SIZE sizeof(float)

#define TEMPERATURE_SAMPLE_BUFFER_SIZE 25

#define WEBHOOK_PUBLISHER_MESSAGE_LENGTH 512
#define WEBHOOK_PUBLISHER_MESSAGE_SIZE sizeof(char) * WEBHOOK_PUBLISHER_MESSAGE_LENGTH

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

/* START TemperatureProcessorTask */
StackType_t xTemperatureProcessorTaskStack[DEFAULT_TASK_STACK_SIZE_BYTES];
StaticTask_t xTemperatureProcessorTaskBuffer;
TaskHandle_t xTemperatureProcessorTaskHandle;
float temperature_sample_buffer[TEMPERATURE_SAMPLE_BUFFER_SIZE] = {0};
uint8_t temperature_sample_buffer_index = 0;

void vTemperatureProcessorTask(void *pvParameters);

/* START TemperatureSensorQueue */
QueueHandle_t xTemperatureSensorQueueHandle;
uint8_t xTemperatureSensorQueueStorageArea[DEFAULT_QUEUE_SIZE * FLOAT_ITEM_SIZE];
StaticQueue_t xTemperatureSensorQueueBuffer;

/* START WebhookPublisherQueue */
QueueHandle_t xWebhookPublisherQueueHandle;
uint8_t xWebhookPublisherQueueStorageArea[DEFAULT_QUEUE_SIZE * WEBHOOK_PUBLISHER_MESSAGE_SIZE];
StaticQueue_t xWebhookPublisherQueueBuffer;

/* START HealthCheckTask */
StackType_t xHealthCheckTaskStack[DEFAULT_TASK_STACK_SIZE_BYTES];
StaticTask_t xHealthCheckTaskBuffer;
TaskHandle_t xHealthCheckTaskHandle;

void vHealthCheckTask(void *pvParameters);


/* COMMONS */
static const char *TAG = "main";

void create_tasks();

void create_queues();

void initialize_nvs();

void get_temperature_mean_and_flush_to_queue(void);

void print_stack_high_water_mark(const TaskHandle_t *handle);

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
    xWebhookPublisherTaskHandle = xTaskCreateStatic(vWebhookPublisherTask,
                                                    "WebhookPublisherTask",
                                                    DEFAULT_TASK_STACK_SIZE_BYTES,
                                                    NULL,
                                                    tskIDLE_PRIORITY,
                                                    xWebhookPublisherTaskStack,
                                                    &xWebhookPublisherTaskBuffer);
    xTemperatureProcessorTaskHandle = xTaskCreateStatic(vTemperatureProcessorTask,
                                                        "TemperatureProcessorTask",
                                                        DEFAULT_TASK_STACK_SIZE_BYTES,
                                                        NULL,
                                                        tskIDLE_PRIORITY,
                                                        xTemperatureProcessorTaskStack,
                                                        &xTemperatureProcessorTaskBuffer);
    xHealthCheckTaskHandle = xTaskCreateStatic(vHealthCheckTask,
                                               "HealthCheckTask",
                                               DEFAULT_TASK_STACK_SIZE_BYTES,
                                               NULL,
                                               tskIDLE_PRIORITY,
                                               xHealthCheckTaskStack,
                                               &xHealthCheckTaskBuffer);
}

void create_queues() {
    xTemperatureSensorQueueHandle = xQueueCreateStatic(DEFAULT_QUEUE_SIZE,
                                                       FLOAT_ITEM_SIZE,
                                                       &xTemperatureSensorQueueStorageArea[0],
                                                       &xTemperatureSensorQueueBuffer);
    xWebhookPublisherQueueHandle = xQueueCreateStatic(DEFAULT_QUEUE_SIZE,
                                                      WEBHOOK_PUBLISHER_MESSAGE_SIZE,
                                                      &xWebhookPublisherQueueStorageArea[0],
                                                      &xWebhookPublisherQueueBuffer);
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
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}

void vWebhookPublisherTask(void *pvParameters) {
    char message[WEBHOOK_PUBLISHER_MESSAGE_LENGTH];
    while (1) {
        if (xQueueReceive(xWebhookPublisherQueueHandle, &message, 100 / portTICK_PERIOD_MS) == pdFALSE) {
            continue;
        }

        send_to_telegram(message);

        memset(message, '\0', sizeof(message));

        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

void vTemperatureProcessorTask(void *pvParameters) {
    ESP_LOGI(TAG, "Starting vTemperatureProcessorTask");
    float current_temperature = 0.0f;
    while (true) {
        if (xQueueReceive(xTemperatureSensorQueueHandle, &current_temperature, 1000 / portTICK_PERIOD_MS) == pdTRUE) {
            if (temperature_sample_buffer_index >= TEMPERATURE_SAMPLE_BUFFER_SIZE) {
                get_temperature_mean_and_flush_to_queue();
                continue;
            }
            temperature_sample_buffer[temperature_sample_buffer_index++] = current_temperature;
        }
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}

void get_temperature_mean_and_flush_to_queue(void) {
    float sum = 0.0f;
    char message[WEBHOOK_PUBLISHER_MESSAGE_LENGTH] = {0};

    for (int i = 0; i < TEMPERATURE_SAMPLE_BUFFER_SIZE; i++) {
        sum += temperature_sample_buffer[i];
    }

    const float mean = sum / TEMPERATURE_SAMPLE_BUFFER_SIZE;
    sprintf(message, "Temperatura média das últimas %d amostras: %.2f °C", TEMPERATURE_SAMPLE_BUFFER_SIZE, mean);

    xQueueSend(xWebhookPublisherQueueHandle, &message, 100/ portTICK_PERIOD_MS);

    memset(temperature_sample_buffer, 0, sizeof(temperature_sample_buffer));
    temperature_sample_buffer_index = 0;
}

void vHealthCheckTask(void *pvParameters) {
    ESP_LOGI(TAG, "Starting vHealthCheckTask");
    while (1) {
        ESP_LOGI(TAG, "Starting Health Check");
        ESP_LOGI(TAG, "Minimum free heap size: %" PRIu32 " bytes", esp_get_minimum_free_heap_size());
        print_stack_high_water_mark(&xReadTemperatureSensorHandle);
        print_stack_high_water_mark(&xWebhookPublisherTaskHandle);
        print_stack_high_water_mark(&xTemperatureProcessorTaskHandle);
        print_stack_high_water_mark(&xHealthCheckTaskHandle);
        ESP_LOGI(TAG, "Ending Health Check");
        vTaskDelay(30000 / portTICK_PERIOD_MS);
    }
}

void print_stack_high_water_mark(const TaskHandle_t *handle) {
    ESP_LOGI(TAG, "Minimum free stack for %s Task: %u bytes",
        pcTaskGetName(*handle),
        uxTaskGetStackHighWaterMark(*handle) * WORD_SIZE_IN_BYTES);
}
