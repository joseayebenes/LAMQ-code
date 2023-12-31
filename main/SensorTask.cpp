
#include "SensorTask.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"

#include "driver/i2c.h"
#include "aht20.h"
#include "ags10.h"
#include "pms5003.h"
#include "air-quality-sensor-manager.h"
#include "MH-Z14A.h"
#include "esp_log.h"

#define APP_TASK_NAME "SENSOR"
#define APP_EVENT_QUEUE_SIZE 10
#define APP_TASK_STACK_SIZE (3*1024)
#define AIR_QUALITY_SENSOR_ENDPOINT 0x2

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
AGS10 ags10sensor;
PMS5003 pms5003sensor;
MHZ14A mhz14asensor;

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
    ags10sensor.initialize(I2C_MASTER_NUM);
    pms5003sensor.initialize();
    mhz14asensor.initialize();

    chip::DeviceLayer::PlatformMgr().LockChipStack();
    AirQualitySensorManager::InitInstance(EndpointId(AIR_QUALITY_SENSOR_ENDPOINT));
    AirQualitySensorManager::GetInstance()->Init();
    chip::DeviceLayer::PlatformMgr().UnlockChipStack();

    return err;
}

/**
 *  function to calculate air quality index from the sensor data
 * 
 *
 */
void calculateAirQualityIndex(AirQuality::AirQualityEnum *airQuality, uint16_t pm25, uint16_t pm10, uint16_t voc, uint32_t co2)
{
    if(pm25 < 12 && pm10 < 55 && voc < 200 && co2 < 1000){
        *airQuality = AirQuality::AirQualityEnum::kGood;
    }else if(pm25 < 35 && pm10 < 155 && voc < 400 && co2 < 2000){
        *airQuality = AirQuality::AirQualityEnum::kFair;
    }else if(pm25 < 55 && pm10 < 255 && voc < 600 && co2 < 5000){
        *airQuality = AirQuality::AirQualityEnum::kModerate;
    }else if(pm25 < 150 && pm10 < 355 && voc < 1000 && co2 < 10000){
        *airQuality = AirQuality::AirQualityEnum::kPoor;
    }else{
        *airQuality = AirQuality::AirQualityEnum::kVeryPoor;
    }
    // Calculo basado en https://www.airveda.com/blog/what-is-aqi-how-is-it-calculated/
}


void SensorTask::SensorTaskMain(void * pvParameter){

    CHIP_ERROR err = sSensorTask.Init();
    AHT20_Sensor_t sensorData;
    uint32_t vocPpb;
    pms5003_data_t pms5003Data;
    uint32_t co2Ppm;
    AirQuality::AirQualityEnum airQuality;
    while (true)
    {
        aht20sensor.readData(&sensorData);
        ags10sensor.readData(&vocPpb);
        pms5003sensor.readData(&pms5003Data);
        mhz14asensor.readData(&co2Ppm);       
        calculateAirQualityIndex(&airQuality, pms5003Data.pm2_5_standard, pms5003Data.pm10_0_standard, vocPpb, co2Ppm);
        chip::DeviceLayer::PlatformMgr().LockChipStack();
        AirQualitySensorManager::GetInstance()->OnTemperatureMeasurementChangeHandler((uint16_t)(sensorData.temperature*100));
        AirQualitySensorManager::GetInstance()->OnHumidityMeasurementChangeHandler((uint16_t)(sensorData.humidity*100));
        AirQualitySensorManager::GetInstance()->OnCarbonDioxideMeasurementChangeHandler(500.0);
        AirQualitySensorManager::GetInstance()->OnTotalVolatileOrganicCompoundsMeasurementChangeHandler((float)vocPpb);
        AirQualitySensorManager::GetInstance()->OnPm25MeasurementChangeHandler((uint16_t)pms5003Data.pm2_5_standard);
        AirQualitySensorManager::GetInstance()->OnPm10MeasurementChangeHandler((float)pms5003Data.pm10_0_standard); 
        AirQualitySensorManager::GetInstance()->OnPm1MeasurementChangeHandler((float)pms5003Data.pm1_0_standard);
        AirQualitySensorManager::GetInstance()->OnCarbonDioxideMeasurementChangeHandler((float)co2Ppm);
        AirQualitySensorManager::GetInstance()->OnAirQualityChangeHandler(airQuality);
        chip::DeviceLayer::PlatformMgr().UnlockChipStack();
        ESP_LOGI(APP_TASK_NAME, "Temperature: %f, Humidity: %f\n", sensorData.temperature, sensorData.humidity);
        ESP_LOGI(APP_TASK_NAME, "VOC: %ld\n", vocPpb);
        ESP_LOGI(APP_TASK_NAME, "PM1.0: %d, PM2.5: %d, PM10: %d\n", pms5003Data.pm1_0_standard, pms5003Data.pm2_5_standard, pms5003Data.pm10_0_standard);
        ESP_LOGI(APP_TASK_NAME, "CO2: %ld\n", co2Ppm);

        switch(airQuality){
            case AirQuality::AirQualityEnum::kGood:
                ESP_LOGI(APP_TASK_NAME, "Good\n");
                break;
            case AirQuality::AirQualityEnum::kFair:
                ESP_LOGI(APP_TASK_NAME, "Fair\n");
                break;
            case AirQuality::AirQualityEnum::kModerate:
                ESP_LOGI(APP_TASK_NAME, "Moderate\n");
                break;
            case AirQuality::AirQualityEnum::kPoor:
                ESP_LOGI(APP_TASK_NAME, "Poor\n");
                break;
            case AirQuality::AirQualityEnum::kVeryPoor:
                ESP_LOGI(APP_TASK_NAME, "VeryPoor\n");
                break;
            default:
                ESP_LOGI(APP_TASK_NAME, "Unknown\n");
                break;
        }
        vTaskDelay(pdMS_TO_TICKS(20000));
    }
}

