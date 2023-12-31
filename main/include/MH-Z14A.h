#pragma once

#include "esp_system.h"
#define MHZ14A_RX_BUF_SIZE 128

class MHZ14A{
    public:
        esp_err_t initialize();
        esp_err_t readData(uint32_t *co2Ppm);
    private:
        uint8_t buffer[MHZ14A_RX_BUF_SIZE];

        uint8_t crc(uint8_t msg[9]);
};
