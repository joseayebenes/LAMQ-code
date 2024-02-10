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

#pragma once

#include "esp_log.h"
#include "esp_err.h"
#include "led_strip.h"

class LEDWidget
{
public:
    void Init(void);
    void Set(bool state);
    void Toggle(void);

    void SetBrightness(uint8_t brightness);
    void UpdateState();
    void SetColor(uint8_t Hue, uint8_t Saturation);
    uint8_t GetLevel(void);
    bool IsTurnedOn(void);

private:
    bool mState;
    uint8_t mBrightness;
    uint8_t mHue;
    uint8_t mSaturation;
    led_strip_handle_t  mStrip;

    void DoSet(void);
};
