
#pragma once
#include <stdint.h>
#include "esp_system.h"
#include "driver/i2c.h"

class AGS10
{
    public:
        /**
         * @brief Initialize the sensor
         * 
         * @param i2c_master_num I2C port number
         * @return esp_err_t
         *     - ESP_OK Success
        */
        esp_err_t initialize(uint8_t i2c_master_num);
        /**
         * @brief Read the sensor data
         * 
         * @param vocPpb VOC concentration in parts per billion
         * @return esp_err_t
         *     - ESP_OK Success
        */
        esp_err_t readData(uint32_t *vocPpb);
    private:
        i2c_port_t i2c_master_num;

        uint8_t Calc_CRC8(uint8_t *dat, uint8_t Num);
};

