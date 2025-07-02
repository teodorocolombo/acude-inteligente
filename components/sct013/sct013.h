#ifndef SCT_013_H
#define SCT_013_H

#include <stdbool.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

    esp_err_t sct013_init(void);

    esp_err_t sct013_read_current(double *current_rms);

#ifdef __cplusplus
}
#endif

#endif // SCT_013_H
