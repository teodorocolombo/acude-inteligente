#ifndef TEMP_SENSOR_H
#define TEMP_SENSOR_H

#include "ds18b20.h"

#ifdef __cplusplus
extern "C" {
#endif

void temp_sensor_init();

DS18B20_ERROR temp_sensor_read(float *value);

#ifdef __cplusplus
}
#endif

#endif // TEMP_SENSOR_H
