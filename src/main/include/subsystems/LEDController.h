// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <ctre/phoenix6/CANdle.hpp>
#include <ctre/phoenix6/configs/CANdleFeaturesConfigs.hpp>
#include <ctre/phoenix6/signals/RGBWColor.hpp>
#include <ctre/phoenix6/controls/RainbowAnimation.hpp>
#include <ctre/phoenix6/controls/EmptyAnimation.hpp>
#include <ctre/phoenix6/signals/SpnEnums.hpp>
#include <frc/DigitalInput.h>
#include <frc2/command/button/Trigger.h>
#include "subsystems/LED_Groups.h"
#include <frc/DriverStation.h>
#include <utility>
#include <vector>

using RGBWColor = ctre::phoenix6::signals::RGBWColor;
enum LEDState
{
  STATIC,
  BLINK,
  FIRE
};

enum LEDIntakeState
{
  NO_NOTE, // Red
  NOTE     // Green
};

enum LEDShooterState
{
  LED_NO_SHOT,  // Orange
  LED_SPIN_UP,  // Yellow
  LED_SHOOTING, // Green
  LED_BAD       // Blinking Red
};

class LEDController : public frc2::SubsystemBase
{
public:
  LEDController();
  void DefaultAnimation();
  void TeleopLED();
  void RedAlliance();
  void BlueAlliance();
  void Last10SecondsRed();
  void Last5SecondsRed();
  void TeleopInit();

  void SetStrobe(RGBWColor color, units::frequency::hertz_t speed);
  void SetStatic(RGBWColor color);
  void SetFire(units::frequency::hertz_t speed);

  void ClearLEDs();
  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void TeleopPeriodic();

  LEDIntakeState m_intakeState = LEDIntakeState::NO_NOTE;
  LEDShooterState m_shooterState = LEDShooterState::LED_BAD;
  ctre::phoenix6::hardware::CANdle m_candle{60, ctre::phoenix6::CANBus::RoboRIO()};
  ctre::phoenix6::configs::CANdleConfiguration candleConfig;
  frc::DriverStation::Alliance activeHub = frc::DriverStation::Alliance::kBlue;

private:
  LEDIntakeState m_intakeStatePrev = LEDIntakeState::NO_NOTE;
  LEDShooterState m_shooterStatePrev = LEDShooterState::LED_BAD;

  frc::Timer m_timer;
  const std::vector<int> m_defaultTimes = {10, 15, 5, 5, 15, 5, 5, 15, 5, 5, 15, 5, 5, 30};
  const std::vector<std::pair<LEDState, units::frequency::hertz_t>> m_defaultStates = {
    {LEDState::STATIC, 5_Hz}, {LEDState::BLINK, 5_Hz}, {LEDState::BLINK, 10_Hz},
    {LEDState::STATIC, 5_Hz}, {LEDState::BLINK, 5_Hz}, {LEDState::BLINK, 10_Hz},
    {LEDState::STATIC, 5_Hz}, {LEDState::BLINK, 5_Hz}, {LEDState::BLINK, 10_Hz},
    {LEDState::STATIC, 5_Hz}, {LEDState::BLINK, 5_Hz}, {LEDState::BLINK, 10_Hz},
    {LEDState::FIRE, 2_Hz}    // Endgame
  };
  std::vector<int> times = m_defaultTimes;
  std::vector<std::pair<LEDState, units::frequency::hertz_t>> states = m_defaultStates;

  units::time::second_t Time{0.2};
  units::time::second_t Speed{0.1};

  int _r = 255;
  int _g = 0;
  int _b = 0;
  int i = 0;


  static constexpr RGBWColor kBlue{11, 61, 145, 0};
  static constexpr RGBWColor kRed{255, 0, 0, 0};
  static constexpr RGBWColor kWhite{0, 0, 0, 255};

  bool P_state;
  bool C_state;

};
