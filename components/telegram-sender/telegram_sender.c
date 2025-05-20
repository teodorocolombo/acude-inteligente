#include "telegram_sender.h"

#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#include "esp_http_client.h"

#include "esp_err.h"
#include "esp_log.h"

#include "esp_crt_bundle.h"

#define CHAT_ID "-4715049106"
#define BOT_KEY "8147287770:AAH9JW-X1XdUT2xdQLU-o4X73Py29O_zwtY"

static const char *TAG = "telegram_sender";
static const char ENDPOINT[] = "https://api.telegram.org/bot" BOT_KEY "/sendMessage";

esp_err_t send_to_telegram(const char *message) {
    ESP_LOGI(TAG, "Sending message: %s", message);

    const esp_http_client_config_t config = {
        .url = ENDPOINT,
        .method = HTTP_METHOD_POST,
        .timeout_ms = 50000,
        .disable_auto_redirect = false,
        .crt_bundle_attach = esp_crt_bundle_attach
    };

    esp_http_client_handle_t client = esp_http_client_init(&config);
    esp_http_client_set_header(client, "Content-Type", "application/json");

    char payload[512];
    snprintf(payload, sizeof(payload),
             "{\"chat_id\":\"%s\","
             "\"text\":\"%s\","
             "\"parse_mode\":\"Markdown\"}",
             CHAT_ID,
             message);

    esp_http_client_set_post_field(client, payload, strlen(payload));
    const esp_err_t err = esp_http_client_perform(client);

    esp_http_client_cleanup(client);
    return err;
}
