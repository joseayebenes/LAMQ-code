#pragma once
#include "esp_system.h"

static const int PMS5003_RX_BUF_SIZE = 128;

typedef struct {
    uint16_t pm1_0_standard;
    uint16_t pm2_5_standard;
    uint16_t pm10_0_standard;
    uint16_t pm1_0_atmospheric;
    uint16_t pm2_5_atmospheric;
    uint16_t n_particles_0_3;
    uint16_t n_particles_0_5;
    uint16_t n_particles_1_0;
    uint16_t n_particles_2_5;
    uint16_t n_particles_5_0;
    uint16_t n_particles_10_0;
}pms5003_data_t;

class PMS5003
{
    public:
        esp_err_t initialize();
        esp_err_t readData(pms5003_data_t *sensorData);
    private:
        void parseData();
        uint16_t crc(uint8_t *data, uint8_t len);
        uint8_t buffer[PMS5003_RX_BUF_SIZE];
        pms5003_data_t sensorData;
};
