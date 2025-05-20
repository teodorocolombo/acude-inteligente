#include "telegram_sender.h"

#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#include "esp_http_client.h"

#include "esp_err.h"

#define CHAT_ID ""
#define BOT_KEY ""

esp_err_t send_to_telegram(const char *message) {
    char url[128];
    snprintf(url, sizeof(url), "https://api.telegram.org/bot%s/sendMessage", BOT_KEY);

    const esp_http_client_config_t config = {
        .url = url,
        .method = HTTP_METHOD_POST,
        .timeout_ms = 5000,
        .disable_auto_redirect = true,
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
