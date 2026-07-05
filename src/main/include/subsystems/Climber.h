// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include "frc/DataLogManager.h"
#include "wpi/DataLog.h"
#include <ctre/phoenix6/TalonFX.hpp>
#include <frc/DigitalInput.h>
#include <frc/Servo.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/SubsystemBase.h>
#include <numbers>
#include <units/length.h>
#include <units/time.h>
#include <units/voltage.h>


enum ResetState
{
    CLIMBER_EXTEND_START,
    CLIMBER_EXTEND_MOVING,
    CLIMBER_EXTEND_DONE,
    CLIMBER_EXTEND_BRAKE_DISENGAGE,
    CLIMBER_RETRACT_START,
    CLIMBER_RETRACT_MOVING,
    CLIMBER_RETRACT_DONE
};

namespace ClimberConstants 
{
    const int ClimbMotorId1 = 13;
    // const int ClimbMotorId2 = 14;
    constexpr double SpoolDiameterInches = 0.6;
    constexpr double GearReduction = 12.0;
    // (Spool Diameter * PI) / Gear Reduction
    constexpr double InchesPerRotation = (SpoolDiameterInches * std::numbers::pi) / GearReduction;
    constexpr units::length::inch_t MinRetractInches{0.5};
    constexpr units::length::inch_t MaxExtensionInches{9.4};
}

class Climber : public frc2::SubsystemBase
{
  public:
    Climber();

    /**
     * Will be called periodically whenever the CommandScheduler runs.
     */
    void Periodic() override;

    void extend();
    void retract();
    // void retractLimit_Pit();

    void Zero();

    units::length::inch_t GetPositionInches();

    void moveMotor();
    void stopMotor();
    // bool atBottomLimit();
    ResetState m_ClimberState;
    bool atBot();

  private:
    // Components (e.g. motor controllers and sensors) should generally be
    // declared private and exposed only through public methods.
    ctre::phoenix6::hardware::TalonFX climberMotor1{ClimberConstants::ClimbMotorId1};
    // ctre::phoenix6::hardware::TalonFX climberMotor2{ClimberConstants::ClimbMotorId2};

    frc::DigitalInput bottomLimit1{2};

    wpi::log::DoubleLogEntry m_PositionLog;
    wpi::log::DoubleLogEntry m_PositionInchesLog;
    wpi::log::IntegerLogEntry m_StateLog;
    wpi::log::BooleanLogEntry m_LimitSwitchLog;
    wpi::log::DoubleLogEntry m_MotorCurrentLog;
    wpi::log::DoubleLogEntry m_MotorVoltageLog;

    units::time::second_t time_brake_released;
};
