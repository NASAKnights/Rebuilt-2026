// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/LEDController.h"

LEDController::LEDController()
{
    // candle.ConfigLEDType(ctre::phoenix::led::LEDStripType::GRB);
    // candle.SetLEDs(0, 0, 0);
    // 70 leds not including the 8
    ctre::phoenix6::configs::LEDConfigs ledConfig;
    ledConfig.WithStripType(ctre::phoenix6::signals::StripTypeValue::GRB);
    candleConfig.WithLED(ledConfig);
    m_candle.GetConfigurator().Apply(candleConfig);
    m_candle.SetControl(ctre::phoenix6::controls::EmptyAnimation(0));
    
}

void LEDController::ClearLEDs() {
    for (int i = 0; i < 8; ++i) {
        m_candle.SetControl(ctre::phoenix6::controls::EmptyAnimation{i});
    }
}

void LEDController::DefaultAnimation()
{
    m_candle.SetControl(ctre::phoenix6::controls::RainbowAnimation(8,77));
    // candle.SetControl(ctre::phoenix6::controls::RainbowAnimation);
    // for (int i = 0; i < 10; i++)
    // {
    //     candle.ClearAnimation(i);
    // }
    // auto rgbfade = ctre::phoenix::led::RgbFadeAnimation(1.0, 0.7, -1, 8);
    // auto rainbow = ctre::phoenix::led::RainbowAnimation(1.0, 0.7, -1, false, 8);
    // auto fire = ctre::phoenix::led::FireAnimation(1.0, 0.7, -1, 1, 0.0, false, 8);
    // auto twinkle = ctre::phoenix::led::TwinkleAnimation(0, 0, 255, 1, 0.7, -1, ctre::phoenix::led::TwinkleAnimation::Percent100, 8);
    // ledGroup1.SetColor(50, 0, 0);
    // ledGroup1.SetRainbow(0);
    // ledGroup2.SetRainbow(2);
    // ledGroup3.SetRainbow(5);
    // ledGroup5.SetInvertedRainbow(3);
    // ledGroup4.SetRainbow(4);
}

void LEDController::TeleopLED()
{
    // for (int i = 0; i < 10; i++)
    // {
    //     candle.ClearAnimation(i);
    // }
    // ledGroup1.SetLarson(0, 50, 0, 14);
}



void LEDController::Periodic() {}
