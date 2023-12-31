
#include "MH-Z14A.h"
#include "esp_system.h"
#include "esp_log.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#define PMS_TXD_PIN (GPIO_NUM_15)
#define PMS_RXD_PIN (GPIO_NUM_2)


esp_err_t MHZ14A::initialize()
{
    const uart_config_t uart_config = {
        .baud_rate = 9600,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    // We won't use a buffer for sending data.
    uart_driver_install(UART_NUM_1, MHZ14A_RX_BUF_SIZE * 2, 0, 0, NULL, 0);
    uart_param_config(UART_NUM_1, &uart_config);
    uart_set_pin(UART_NUM_1, PMS_TXD_PIN, PMS_RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    return ESP_OK;
}

esp_err_t MHZ14A::readData(uint32_t *co2Ppm)
{

    printf("Reading data from MH-Z14A\n");
    uart_write_bytes(UART_NUM_1, "\xff\x01\x86\x00\x00\x00\x00\x00\x79", 9);
    const int rxBytes = uart_read_bytes(UART_NUM_1, buffer, MHZ14A_RX_BUF_SIZE, 1000 / portTICK_PERIOD_MS);
    if (rxBytes > 0) {
        if(buffer[0] != 0xff || buffer[1] != 0x86){
            printf("Invalid response from MH-Z14A\n");
            return ESP_FAIL;
        }
        if(buffer[8] != crc(buffer)){
            printf("Invalid CRC from MH-Z14A\n");
            return ESP_FAIL;
        }
        *co2Ppm = (uint32_t)buffer[2]<<8 | buffer[3];
        // print buffer
        for (int i = 0; i < rxBytes; i++) {
            printf("%02x ", buffer[i]);
        }
        printf("\n");
    }
    return ESP_OK;
}


uint8_t MHZ14A::crc(uint8_t msg[9])
{
    uint8_t chksum = 0;
    for(int i=1; i<8; i++)
    {
        chksum += msg[i];
    }
    chksum = 0xff - chksum;
    chksum += 1;
    return chksum;
}