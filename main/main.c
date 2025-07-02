#include <string.h>

#include "esp_event.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "nvs_flash.h"
#include "lwip/err.h"
#include "lwip/sys.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "temp_sensor.h"
#include "telegram_sender.h"
#include "wifi_station.h"
#include "pushbutton.h"
#include "relay.h"

/* START DEFINE */
#define WORD_SIZE_IN_BYTES 4
#define DEFAULT_TASK_STACK_SIZE_BYTES 4096
#define DEFAULT_QUEUE_SIZE 10
#define FLOAT_ITEM_SIZE sizeof(float)
#define RELAY_COMMAND_T_SIZE sizeof(relay_command_t)

#define TEMPERATURE_SAMPLE_BUFFER_SIZE 25

#define WEBHOOK_PUBLISHER_MESSAGE_LENGTH 512
#define WEBHOOK_PUBLISHER_MESSAGE_SIZE sizeof(char) * WEBHOOK_PUBLISHER_MESSAGE_LENGTH

static gpio_num_t shutdown_isr_button_gpio = GPIO_NUM_23;
StackType_t xGracefulShutdownTaskStack[DEFAULT_TASK_STACK_SIZE_BYTES];
StaticTask_t xGracefulShutdownTaskBuffer;
TaskHandle_t xGracefulShutdownTaskHandle;

void vGracefulShutdownTask(void *pvParameters);

static void IRAM_ATTR gpio_isr_shutdown_handler(void *arg);

QueueHandle_t xShutdownRequestQueueHandle;
uint8_t xShutdownRequestQueueStorageArea[DEFAULT_QUEUE_SIZE * WORD_SIZE_IN_BYTES];
StaticQueue_t xShutdownRequestQueueBuffer;

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

/* START RelayControlTask */
StackType_t xControlRelayTaskStack[DEFAULT_TASK_STACK_SIZE_BYTES];
StaticTask_t xControlRelayTaskBuffer;
TaskHandle_t xControlRelayTaskHandle;

/* START RelayCommandQueue */
typedef enum {
    RELAY_CMD_NONE = 0,
    RELAY_CMD_ACTIVATE,
    RELAY_CMD_DEACTIVATE
} relay_command_t;

QueueHandle_t xRelayCommandQueueHandle;
uint8_t xRelayCommandQueueStorageArea[DEFAULT_QUEUE_SIZE * RELAY_COMMAND_T_SIZE];
StaticQueue_t xRelayCommandQueueBuffer;

void vControlRelayTask(void *pvParameters);

/* START RelayTestTask */
StackType_t xRelayTestTaskStack[DEFAULT_TASK_STACK_SIZE_BYTES];
StaticTask_t xRelayTestTaskBuffer;
TaskHandle_t xRelayTestTaskHandle;

void vRelayTestTask(void *pvParameters);


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
    ESP_LOGI(TAG, "Initializing Queues");
    create_queues();
    ESP_LOGI(TAG, "Initializing Tasks");
    create_tasks();

    ESP_LOGI(TAG, "Setting up shutdown button using pushbutton module");
    const esp_err_t err = init_shutdown_isr(shutdown_isr_button_gpio, gpio_isr_shutdown_handler, NULL);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize shutdown button: %s", esp_err_to_name(err));
    }
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
    xGracefulShutdownTaskHandle = xTaskCreateStatic(vGracefulShutdownTask,
                                                    "GracefulShutdownTask",
                                                    DEFAULT_TASK_STACK_SIZE_BYTES,
                                                    NULL,
                                                    tskIDLE_PRIORITY + 1,
                                                    xGracefulShutdownTaskStack,
                                                    &xGracefulShutdownTaskBuffer);
    xControlRelayTaskHandle = xTaskCreateStatic(vControlRelayTask,
                                                "ControlRelayTask",
                                                DEFAULT_TASK_STACK_SIZE_BYTES,
                                                NULL,
                                                tskIDLE_PRIORITY,
                                                xControlRelayTaskStack,
                                                &xControlRelayTaskBuffer);
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
    xShutdownRequestQueueHandle = xQueueCreateStatic(DEFAULT_QUEUE_SIZE,
                                                     WORD_SIZE_IN_BYTES,
                                                     &xShutdownRequestQueueStorageArea[0],
                                                     &xShutdownRequestQueueBuffer);
    xRelayCommandQueueHandle = xQueueCreateStatic(DEFAULT_QUEUE_SIZE,
                                                  RELAY_COMMAND_T_SIZE,
                                                  xRelayCommandQueueStorageArea,
                                                  &xRelayCommandQueueBuffer);
    xRelayTestTaskHandle = xTaskCreateStatic(vRelayTestTask,
                                             "RelayTestTask",
                                             DEFAULT_TASK_STACK_SIZE_BYTES,
                                             NULL,
                                             tskIDLE_PRIORITY,
                                             xRelayTestTaskStack,
                                             &xRelayTestTaskBuffer);
}

static void IRAM_ATTR gpio_isr_shutdown_handler(void *arg) {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    vTaskNotifyGiveFromISR(xGracefulShutdownTaskHandle, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void vGracefulShutdownTask(void *pvParameters) {
    ESP_LOGI(TAG, "Starting vGracefulShutdownTask");
    while (true) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        vTaskDelay(pdMS_TO_TICKS(50));
        if (gpio_get_level(shutdown_isr_button_gpio) == 0) {
            ESP_LOGW(TAG, "Shutdown button pressed on GPIO %d! Initiating graceful shutdown...",
                     shutdown_isr_button_gpio);

            vTaskSuspend(xReadTemperatureSensorHandle);
            vTaskDelete(xReadTemperatureSensorHandle);
            ESP_LOGI(TAG, "Finished xReadTemperatureSensorHandle");

            ESP_LOGW(TAG, "Simulating graceful shutdown complete. Restarting now...");
            vTaskDelay(pdMS_TO_TICKS(2500));
            esp_restart();
        }
    }
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
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void vWebhookPublisherTask(void *pvParameters) {
    char message[WEBHOOK_PUBLISHER_MESSAGE_LENGTH];
    while (true) {
        if (xQueueReceive(xWebhookPublisherQueueHandle, &message, pdMS_TO_TICKS(100)) == pdTRUE) {
            send_to_telegram(message);
            memset(message, '\0', sizeof(message));
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void vTemperatureProcessorTask(void *pvParameters) {
    ESP_LOGI(TAG, "Starting vTemperatureProcessorTask");
    float current_temperature = 0.0f;
    while (true) {
        if (xQueueReceive(xTemperatureSensorQueueHandle, &current_temperature, pdMS_TO_TICKS(1000)) == pdTRUE) {
            if (temperature_sample_buffer_index >= TEMPERATURE_SAMPLE_BUFFER_SIZE) {
                get_temperature_mean_and_flush_to_queue();
                continue;
            }
            temperature_sample_buffer[temperature_sample_buffer_index++] = current_temperature;
        }
        vTaskDelay(pdMS_TO_TICKS(100));
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
    while (true) {
        ESP_LOGI(TAG, "Starting Health Check");
        ESP_LOGI(TAG, "Minimum free heap size: %" PRIu32 " bytes", esp_get_minimum_free_heap_size());
        print_stack_high_water_mark(&xReadTemperatureSensorHandle);
        print_stack_high_water_mark(&xWebhookPublisherTaskHandle);
        print_stack_high_water_mark(&xTemperatureProcessorTaskHandle);
        print_stack_high_water_mark(&xHealthCheckTaskHandle);
        print_stack_high_water_mark(&xGracefulShutdownTaskHandle);
        ESP_LOGI(TAG, "Ending Health Check");
        vTaskDelay(30000 / portTICK_PERIOD_MS);
    }
}

void print_stack_high_water_mark(const TaskHandle_t *handle) {
    ESP_LOGI(TAG, "Minimum free stack for %s Task: %u bytes",
             pcTaskGetName(*handle),
             uxTaskGetStackHighWaterMark(*handle) * WORD_SIZE_IN_BYTES);
}

void vControlRelayTask(void *pvParameters) {
    ESP_LOGI(TAG, "Starting vControlRelayTask");
    char message[WEBHOOK_PUBLISHER_MESSAGE_LENGTH] = {0};

    esp_err_t err = relay_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize low-level relay driver: %s", esp_err_to_name(err));
    }

    vTaskDelay(pdMS_TO_TICKS(2000));
    relay_active();
    vTaskDelay(pdMS_TO_TICKS(2000));
    relay_desactive();

    memset(temperature_sample_buffer, 0, sizeof(temperature_sample_buffer));
    relay_command_t received_command;
    while (true) {
        if (xQueueReceive(xRelayCommandQueueHandle, &received_command, portMAX_DELAY) == pdTRUE) {
            switch (received_command) {
                case RELAY_CMD_ACTIVATE:
                    ESP_LOGI(TAG, "Received RELAY_CMD_ACTIVATE");
                    relay_active();
                    sprintf(message, "Relé ativado");
                    xQueueSend(xWebhookPublisherQueueHandle, &message, pdMS_TO_TICKS(100));
                    break;
                case RELAY_CMD_DEACTIVATE:
                    ESP_LOGI(TAG, "Received RELAY_CMD_DEACTIVATE");
                    relay_desactive();
                    sprintf(message, "Relé desativado");
                    xQueueSend(xWebhookPublisherQueueHandle, &message, pdMS_TO_TICKS(100));
                    break;
                default:
                    ESP_LOGW(TAG, "Received unknown relay command: %d", received_command);
                    break;
            }
        }
    }
}

void vRelayTestTask(void *pvParameters) {
    ESP_LOGI(TAG, "Starting vRelayTestTask");
    relay_command_t command;
    bool activate = true;

    while (true) {
        if (activate) {
            command = RELAY_CMD_ACTIVATE;
            ESP_LOGI(TAG, "Sending RELAY_CMD_ACTIVATE to xRelayCommandQueueHandle");
        } else {
            command = RELAY_CMD_DEACTIVATE;
            ESP_LOGI(TAG, "Sending RELAY_CMD_DEACTIVATE to xRelayCommandQueueHandle");
        }

        if (xQueueSend(xRelayCommandQueueHandle, &command, pdMS_TO_TICKS(100)) != pdTRUE) {
            ESP_LOGE(TAG, "Failed to send relay command to queue.");
        }

        activate = !activate;

        vTaskDelay(pdMS_TO_TICKS(30000));
    }
}
