#ifndef TELEGRAM_SENDER_H
#define TELEGRAM_SENDER_H

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif


esp_err_t send_to_telegram(const char *message);


#ifdef __cplusplus
}
#endif


#endif // TELEGRAM_SENDER_H
