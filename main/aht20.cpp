#include "aht20.h"
#include <stdio.h>
#include "esp_log.h"
#include "driver/i2c.h"

#define I2C_MASTER_TIMEOUT_MS       1000
#define AHT20_SENSOR_ADDR 0x38

enum registers
{
    sfe_aht20_reg_reset = 0xBA,
    sfe_aht20_reg_initialize = 0xBE,
    sfe_aht20_reg_measure = 0xAC,
};


esp_err_t AHT20::initialize(uint8_t i2c_master_num){
    int ret;
    uint8_t write_buf[] = {sfe_aht20_reg_initialize, 0x08, 0x00};
    this->i2c_master_num=i2c_master_num;
    ret = i2c_master_write_to_device((i2c_port_t)i2c_master_num, AHT20_SENSOR_ADDR, write_buf, sizeof(write_buf), I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);

    return ret;
}

esp_err_t AHT20::readData(AHT20_Sensor_t *sensorData){
    int ret;

    uint32_t temperature = 0;
    uint32_t humidity = 0;

    uint8_t data[7] = {0};
    uint8_t write_buf[] = {sfe_aht20_reg_measure, 0x33, 0x00};

    ret = i2c_master_write_to_device((i2c_port_t)i2c_master_num, AHT20_SENSOR_ADDR, write_buf, sizeof(write_buf), I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
    vTaskDelay(pdMS_TO_TICKS(80));
    ret = i2c_master_read_from_device(i2c_master_num, AHT20_SENSOR_ADDR, data, sizeof(data), I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);

    for ( int i = 0; i< sizeof(data); i++ )
    {
    printf("0x%02x ", data[i]);
    }
    printf("\n");

    uint32_t incoming=0;
    incoming |= (uint32_t)data[1]<<16;
    incoming |= (uint32_t)data[2]<<8;
    incoming |= (uint32_t)data[3];
    humidity = incoming >> 4;

    temperature |= (uint32_t)data[3]<<16;
    temperature |= (uint32_t)data[4]<<8;
    temperature |= (uint32_t)data[5];
    temperature = temperature & ~(0xFFF00000);

    sensorData->temperature = ((float)temperature / 1048576 )*200 - 50;
    sensorData->humidity = ((float)humidity / 1048576 )*100;

    return ret;
}

