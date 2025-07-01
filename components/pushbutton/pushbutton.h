#ifndef PUSHBUTTON_H
#define PUSHBUTTON_H

#include "driver/gpio.h"

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t init_shutdown_isr(gpio_num_t gpio_num, gpio_isr_t isr_handler_func, void *isr_args);

#ifdef __cplusplus
}
#endif

#endif // PUSHBUTTON_H
