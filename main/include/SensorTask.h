#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "freertos/FreeRTOS.h"
#include <platform/CHIPDeviceLayer.h>

#define SENSOR_ERROR_CREATE_TASK_FAILED CHIP_APPLICATION_ERROR(0x01)

class SensorTask
{

public:
    CHIP_ERROR StartSensorTask();
    static void SensorTaskMain(void * pvParameter);

private:
    friend SensorTask & GetSensorTask(void);
    CHIP_ERROR Init();
    static SensorTask sSensorTask;
};

inline SensorTask & GetSensorTask(void)
{
    return SensorTask::sSensorTask;
}
