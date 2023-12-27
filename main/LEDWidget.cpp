/*
 *
 *    Copyright (c) 2021-2023 Project CHIP Authors
 *
 *    Licensed under the Apache License, Version 2.0 (the "License");
 *    you may not use this file except in compliance with the License.
 *    You may obtain a copy of the License at
 *
 *        http://www.apache.org/licenses/LICENSE-2.0
 *
 *    Unless required by applicable law or agreed to in writing, software
 *    distributed under the License is distributed on an "AS IS" BASIS,
 *    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *    See the License for the specific language governing permissions and
 *    limitations under the License.
 */

#include "LEDWidget.h"
#include "ColorFormat.h"

static const char TAG[] = "LEDWidget";

void LEDWidget::Init(void)
{
    mState      = false;
    mBrightness = UINT8_MAX;
    mHue        = 0;
    mSaturation = 0;

    // LED strip general initialization, according to your led board design
    led_strip_config_t strip_config = {
        .strip_gpio_num = CONFIG_LED_GPIO,   // The GPIO that connected to the LED strip's data line
        .max_leds = 12,        // The number of LEDs in the strip,
        .led_pixel_format = LED_PIXEL_FORMAT_GRB, // Pixel format of your LED strip
        .led_model = LED_MODEL_WS2812,            // LED strip model
        .flags={.invert_out = false},                // whether to invert the output signal
    };

    // LED strip backend configuration: RMT
    led_strip_rmt_config_t rmt_config = {
        .clk_src = RMT_CLK_SRC_DEFAULT,        // different clock source can lead to different power consumption
        .resolution_hz = 10 * 1000 * 1000, // RMT counter clock frequency
        .mem_block_symbols = 128,
        .flags={.with_dma = false},               // DMA feature is available on ESP target like ESP32-S3
    };


    ESP_ERROR_CHECK(led_strip_new_rmt_device(&strip_config, &rmt_config, &mStrip));

}

void LEDWidget::Set(bool state)
{
    ESP_LOGI(TAG, "Setting state to %d", state ? 1 : 0);
    if (state == mState)
        return;

    mState = state;
    printf("SET\n");
    DoSet();
}

void LEDWidget::Toggle()
{
    ESP_LOGI(TAG, "Toggling state to %d", !mState);
    mState = !mState;
    printf("Toggle\n");
    DoSet();
}

void LEDWidget::SetBrightness(uint8_t brightness)
{
    ESP_LOGI(TAG, "Setting brightness to %d", brightness);
    if (brightness == mBrightness)
        return;

    mBrightness = brightness;

    DoSet();
}

uint8_t LEDWidget::GetLevel()
{
    return this->mBrightness;
}

bool LEDWidget::IsTurnedOn()
{
    return this->mState;
}

void LEDWidget::SetColor(uint8_t Hue, uint8_t Saturation)
{
    if (Hue == mHue && Saturation == mSaturation)
        return;

    mHue        = Hue;
    mSaturation = Saturation;
    printf("SetColor\n");
    DoSet();
}

void LEDWidget::DoSet(void)
{
    uint8_t brightness = mState ? mBrightness : 0;

    if (mStrip)
    {

        HsvColor_t hsv = { mHue, mSaturation, brightness };
        RgbColor_t rgb = HsvToRgb(hsv);
        //led_strip_clear(mStrip);
        led_strip_set_pixel(mStrip, 0,      rgb.r,       rgb.g,      rgb.b);
        led_strip_set_pixel(mStrip, 1,    rgb.r,       rgb.g,      rgb.b);
        led_strip_set_pixel(mStrip, 2,      rgb.r,       rgb.g,      rgb.b);
        led_strip_set_pixel(mStrip, 3,      rgb.r,       rgb.g,      rgb.b);
        led_strip_set_pixel(mStrip, 4,    rgb.r,       rgb.g,      rgb.b);
        led_strip_set_pixel(mStrip, 5,      rgb.r,       rgb.g,      rgb.b);
        led_strip_set_pixel(mStrip, 6,      rgb.r,       rgb.g,      rgb.b);
        led_strip_set_pixel(mStrip, 7,    rgb.r,       rgb.g,      rgb.b);
        led_strip_set_pixel(mStrip, 8,      rgb.r,       rgb.g,      rgb.b);
        led_strip_set_pixel(mStrip, 9,      rgb.r,       rgb.g,      rgb.b);
        led_strip_set_pixel(mStrip, 10,     rgb.r,       rgb.g,      rgb.b);
        led_strip_set_pixel(mStrip, 11,      rgb.r,       rgb.g,      rgb.b);

        led_strip_refresh(mStrip);
    }

}
