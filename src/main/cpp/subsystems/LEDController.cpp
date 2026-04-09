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
}

void LEDController::RedAlliance()
{
    m_candle.SetControl(ctre::phoenix6::controls::SolidColor{0, 77}.WithColor(kRed));
}

void LEDController::BlueAlliance()
{
    m_candle.SetControl(ctre::phoenix6::controls::SolidColor{0, 77}.WithColor(kBlue));
}

void LEDController::Last10SecondsRed()
{
    m_candle.SetControl(ctre::phoenix6::controls::StrobeAnimation{0, 77}.WithColor(kWhite).FrameRate());
}

void LEDController::Last5SecondsRed()
{
    m_candle.SetControl(ctre::phoenix6::controls::StrobeAnimation{0, 77}.WithColor(kWhite).FrameRate());
}

void LEDController::SetStrobe(RGBWColor color, units::frequency::hertz_t speed)
{
    ClearLEDs();
    m_candle.SetControl(ctre::phoenix6::controls::StrobeAnimation(0, 77).WithColor(color).WithFrameRate(speed));
}

void LEDController::SetStatic(RGBWColor color)
{
    ClearLEDs();
    m_candle.SetControl(ctre::phoenix6::controls::SolidColor(0, 77).WithColor(color).WithUpdateFreqHz(20_Hz));
}

void LEDController::SetFire(units::frequency::hertz_t speed)
{
    ClearLEDs();
    m_candle.SetControl(ctre::phoenix6::controls::FireAnimation(0, 77).WithFrameRate(speed).WithBrightness(1.0));
}

void LEDController::TeleopLED()
{
    // for (int i = 0; i < 10; i++)
    // {
    //     candle.ClearAnimation(i);
    // }
    // ledGroup1.SetLarson(0, 50, 0, 14);
}


void LEDController::TeleopInit()
{
    times = m_defaultTimes;
    states = m_defaultStates;

    auto GameData = frc::DriverStation::GetGameSpecificMessage();
    if(GameData.length() > 0)
    {
        switch (GameData[0])
        {
            case 'B' :
                activeHub = frc::DriverStation::Alliance::kRed;
                break;
            case 'R' :
                activeHub = frc::DriverStation::Alliance::kBlue;
                break;
            default :
                activeHub = frc::DriverStation::Alliance::kBlue;
                break;
        }
    }
    m_timer.Restart();
    SetStatic(frc::DriverStation::GetAlliance() == frc::DriverStation::Alliance::kBlue ? kBlue : kRed);
}


void LEDController::TeleopPeriodic() {
    if(times.empty() || states.empty())
    {
        return;
    }
    auto time = units::second_t{static_cast<double>(times.front())};
    if(m_timer.HasElapsed(time))
    {
        auto current_state = states.front();
        times.erase(times.begin());
        states.erase(states.begin());

        // Set State
        // Color  = activeHub
        if(!times.empty() && times.front() == 15)
        {
            activeHub = activeHub == frc::DriverStation::Alliance::kBlue ? frc::DriverStation::Alliance::kRed : frc::DriverStation::Alliance::kBlue;
        }
        auto color = activeHub == frc::DriverStation::kBlue ? kBlue : kRed;

        switch (current_state.first)
        {
        case LEDState::BLINK:
            SetStrobe(color, current_state.second);
            break;
        case LEDState::FIRE:
            SetFire(current_state.second);
            break;
        case LEDState::STATIC:
            SetStatic(color);
            break;
        default:
            break;
        }

        // Reset Timer
        m_timer.Reset();
    }
}
