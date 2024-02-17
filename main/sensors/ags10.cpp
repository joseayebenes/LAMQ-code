#include "ags10.h"
#include "driver/i2c.h"
#include <stdio.h>

//  REGISTERS
#define AGS10_WRITE                 0x34
#define AGS10_READ                  0x35
#define AGS10_TIMEOUT_MS            1500
#define AGS10_DEVICE_ADDRESS        0x1a


//**************************************************************
// Function name: Calc_CRC8
// Function: CRC8 calculation, initial value: 0xFF, polynomial:
// 0x31 (x8+ 5+ x4+1)
//**************************************************************
uint8_t AGS10::Calc_CRC8(uint8_t *dat, uint8_t Num){
    uint8_t i,byte,crc=0xFF;
    for(byte=0; byte<Num; byte++)
    {
        crc^=(dat[byte]);
        for(i=0;i<8;i++)
        {
            if(crc & 0x80) crc=(crc<<1)^0x31;
            else crc=(crc<<1);
        }
    }
    return crc;
}

esp_err_t AGS10::initialize(uint8_t i2c_master_num){
    int ret = 0;
    this->i2c_master_num = (i2c_port_t)i2c_master_num;
    return ret;
}

esp_err_t AGS10::readData(uint32_t *vocPpb){
    int ret;

    uint8_t write_buf[] = {0x00};
    uint8_t read_buf[5] = {0};

    ret = i2c_master_write_read_device(i2c_master_num, AGS10_DEVICE_ADDRESS, write_buf, sizeof(write_buf), read_buf, 5, AGS10_TIMEOUT_MS / portTICK_PERIOD_MS);
    //ret = i2c_master_read_from_device(i2c_master_num, AGS10_DEVICE_ADDRESS, read_buf, 5, AGS10_TIMEOUT_MS / portTICK_PERIOD_MS);

    if(ret!=0){
        return ret;
    }

    uint8_t ch = (read_buf[0]&0x0E)>>1;
    bool rdy = read_buf[0]&0x01;

    uint32_t data = 0;
    data |= (uint32_t)read_buf[1]<<16;
    data |= (uint32_t)read_buf[2]<<8;
    data |= (uint32_t)read_buf[3];
    *vocPpb = data;

    return ret;
}

