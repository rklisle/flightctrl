#ifndef __APP_PRESSURE_SENSOR_H__
#define __APP_PRESSURE_SENSOR_H__
#include <stdint.h>
#include "tx_api.h"

typedef struct pressure_status_tag
{
    float abs_pressure;
    float diff_pressure;
    float temperature;
}pressure_status_t;

extern pressure_status_t pressureData;

int32_t app_pressure_init(TX_BYTE_POOL *pmem);

int32_t pressure_get_status(pressure_status_t * pout);

#endif