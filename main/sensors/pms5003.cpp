
#include "pms5003.h"
#include "esp_system.h"
#include "esp_log.h"
#include "driver/uart.h"
#include "driver/gpio.h"

#define PMS_TXD_PIN (GPIO_NUM_16)
#define PMS_RXD_PIN (GPIO_NUM_17)

// #define PMS_RX 17  // D7
// #define PMS_TX 16  // D6

esp_err_t PMS5003::initialize(){
    int ret = 0;

    const uart_config_t uart_config = {
        .baud_rate = 9600,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    // We won't use a buffer for sending data.
    uart_driver_install(UART_NUM_2, PMS5003_RX_BUF_SIZE * 2, 0, 0, NULL, 0);
    uart_param_config(UART_NUM_2, &uart_config);
    uart_set_pin(UART_NUM_2, PMS_TXD_PIN, PMS_RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    // Set Passive Mode
    uart_write_bytes(UART_NUM_2, "\x42\x4d\xe1\x00\x00\x01\x70", 7);
    return ret;
}

void PMS5003::parseData(){
    sensorData.pm1_0_standard = (uint16_t)buffer[4]<<8 | buffer[5];
    sensorData.pm2_5_standard = (uint16_t)buffer[6]<<8 | buffer[7];
    sensorData.pm10_0_standard = (uint16_t)buffer[8]<<8 | buffer[9];
    sensorData.pm1_0_atmospheric = (uint16_t)buffer[10]<<8 | buffer[11];
    sensorData.pm2_5_atmospheric = (uint16_t)buffer[12]<<8 | buffer[13];
    sensorData.n_particles_0_3  = (uint16_t)buffer[16]<<8 | buffer[17];
    sensorData.n_particles_0_5  = (uint16_t)buffer[18]<<8 | buffer[19];
    sensorData.n_particles_1_0  = (uint16_t)buffer[20]<<8 | buffer[21];
    sensorData.n_particles_2_5  = (uint16_t)buffer[22]<<8 | buffer[23];
    sensorData.n_particles_5_0  = (uint16_t)buffer[24]<<8 | buffer[25];
    sensorData.n_particles_10_0  = (uint16_t)buffer[26]<<8 | buffer[27];
}

esp_err_t PMS5003::readData(pms5003_data_t *sensorData){
    int ret = 0;

    // Write request read
    uart_write_bytes(UART_NUM_2, "\x42\x4d\xe2\x00\x00\x01\x71", 7);
    const int rxBytes = uart_read_bytes(UART_NUM_2, buffer, PMS5003_RX_BUF_SIZE, 1000 / portTICK_PERIOD_MS);
    if (rxBytes > 0) {
        if(buffer[0] != 0x42 || buffer[1] != 0x4d){
            ESP_LOGE("PMS5003", "Invalid response from PMS5003");
            return ESP_FAIL;
        }
        if(buffer[2] != 0x00 || buffer[3] != 0x1c){
            ESP_LOGE("PMS5003", "Invalid response from PMS5003");
            return ESP_FAIL;
        }
        uint16_t checksum = (uint16_t)buffer[30]<<8 | buffer[31];
        uint16_t sum = crc(buffer, 30);
        if(checksum != sum){
            ESP_LOGE("PMS5003", "Invalid CRC from PMS5003");
            return ESP_FAIL;
        }
        parseData();
        sensorData->pm1_0_standard = this->sensorData.pm1_0_standard;
        sensorData->pm2_5_standard = this->sensorData.pm2_5_standard;
        sensorData->pm10_0_standard = this->sensorData.pm10_0_standard;
    }

    return ret;
}

uint16_t PMS5003::crc(uint8_t *data, uint8_t len){
    uint16_t sum = 0;
    for(int i=0; i<len; i++){
        sum += data[i];
    }
    return sum;
}