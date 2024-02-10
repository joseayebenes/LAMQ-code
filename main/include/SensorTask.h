#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "air-quality-sensor-manager.h"
#include "freertos/FreeRTOS.h"
#include <platform/CHIPDeviceLayer.h>

#define SENSOR_ERROR_CREATE_TASK_FAILED CHIP_APPLICATION_ERROR(0x01)
using namespace ::chip;
using namespace chip::app::Clusters;

class SensorTask
{

public:
    CHIP_ERROR StartSensorTask();
    static void SensorTaskMain(void * pvParameter);
    void Set(bool state){
        mState = state;
    }
private:
    friend SensorTask & GetSensorTask(void);
    CHIP_ERROR Init();
    static SensorTask sSensorTask;
    bool mState;
};

inline SensorTask & GetSensorTask(void)
{
    return SensorTask::sSensorTask;
}
