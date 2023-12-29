
#include "SensorTask.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"

#include "driver/i2c.h"
#include "aht20.h"

#include "air-quality-sensor-manager.h"

#define APP_TASK_NAME "APP"
#define APP_EVENT_QUEUE_SIZE 10
#define APP_TASK_STACK_SIZE (3*1024)
#define AIR_QUALITY_SENSOR_ENDPOINT 3

#define I2C_MASTER_SCL_IO           18      /*!< GPIO number used for I2C master clock */
#define I2C_MASTER_SDA_IO           19      /*!< GPIO number used for I2C master data  */
#define I2C_MASTER_NUM              0                          /*!< I2C master i2c port number, the number of i2c peripheral interfaces available will depend on the chip */
#define I2C_MASTER_FREQ_HZ          25000                     /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE   0                          /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE   0                          /*!< I2C master doesn't need buffer */
#define I2C_MASTER_TIMEOUT_MS       1000

using namespace ::chip;
using namespace chip::app::Clusters;

namespace {
constexpr EndpointId kLightEndpointId = 1;
    TaskHandle_t sSensorTaskHandle;
} // namespace

SensorTask SensorTask::sSensorTask;

AHT20 aht20sensor;

CHIP_ERROR SensorTask::StartSensorTask()
{
    // Start App task.
    BaseType_t xReturned;
    xReturned = xTaskCreate(SensorTaskMain, APP_TASK_NAME, APP_TASK_STACK_SIZE, NULL, 1, &sSensorTaskHandle);
    return (xReturned == pdPASS) ? CHIP_NO_ERROR : SENSOR_ERROR_CREATE_TASK_FAILED;
}

CHIP_ERROR SensorTask::Init(){
    CHIP_ERROR err = CHIP_NO_ERROR;

    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master ={.clk_speed = I2C_MASTER_FREQ_HZ},
    };
    i2c_param_config((i2c_port_t)I2C_MASTER_NUM, &conf);
    i2c_driver_install((i2c_port_t)I2C_MASTER_NUM, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);

    aht20sensor.initialize(I2C_MASTER_NUM);

    AirQualitySensorManager::InitInstance(EndpointId(AIR_QUALITY_SENSOR_ENDPOINT));
    AirQualitySensorManager::GetInstance()->Init();

    return err;
}

void SensorTask::SensorTaskMain(void * pvParameter){

    CHIP_ERROR err = sSensorTask.Init();
    AHT20_Sensor_t sensorData;
    
    while (true)
    {
        aht20sensor.readData(&sensorData);
        AirQualitySensorManager::GetInstance()->OnTemperatureMeasurementChangeHandler(sensorData.temperature);
        AirQualitySensorManager::GetInstance()->OnHumidityMeasurementChangeHandler(sensorData.humidity);
        vTaskDelay(pdMS_TO_TICKS(30000));
    }
}