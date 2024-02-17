#pragma once

#include <stdint.h>
#include "esp_system.h"

typedef struct {
    float humidity;
    float temperature;
} AHT20_Sensor_t;

class AHT20
{
    public:
        esp_err_t initialize(uint8_t i2c_master_num);
        esp_err_t readData(AHT20_Sensor_t *sensorData);
    private:
        uint8_t i2c_master_num;
};