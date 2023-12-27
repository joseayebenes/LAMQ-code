#ifndef AGS_10_H
#define AGS_10_H

#include <stdint.h>
#include "esp_system.h"

esp_err_t ags10_initialize(uint8_t i2c_master_num);

esp_err_t ags10_readData(uint8_t i2c_master_num, uint32_t *vocPpb);

#endif // !AGS_10_H