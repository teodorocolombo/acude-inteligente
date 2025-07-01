#ifndef RELAY_H
#define RELAY_H

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t relay_init(void);

esp_err_t relay_active(void);

esp_err_t relay_desactive(void);

#ifdef __cplusplus
}
#endif

#endif // RELAY_H
