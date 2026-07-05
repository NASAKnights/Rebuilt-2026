// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.ßöä
#include "subsystems/Climber.h"

Climber::Climber()
{

    // climberMotor2.SetControl(climberFollower);

    // Initialize Climber Logging
    wpi::log::DataLog& log = frc::DataLogManager::GetLog();
    m_PositionLog = wpi::log::DoubleLogEntry(log, "/Climber/Position");
    m_PositionInchesLog = wpi::log::DoubleLogEntry(log, "/Climber/PositionInches");
    m_StateLog = wpi::log::IntegerLogEntry(log, "/Climber/State");
    m_LimitSwitchLog = wpi::log::BooleanLogEntry(log, "/Climber/LimitSwitch");
    m_MotorCurrentLog = wpi::log::DoubleLogEntry(log, "/Climber/MotorCurrent");
    m_MotorVoltageLog = wpi::log::DoubleLogEntry(log, "/Climber/MotorVoltage");

    ctre::phoenix6::configs::TalonFXConfiguration climbConfig;
    ctre::phoenix6::configs::CurrentLimitsConfigs climbCurrentConfig;
    climbCurrentConfig.SupplyCurrentLimitEnable = true;
    climbCurrentConfig.SupplyCurrentLimit = units::ampere_t{80};
    climbCurrentConfig.SupplyCurrentLowerLimit = units::ampere_t{35};
    climbCurrentConfig.SupplyCurrentLowerTime = units::second_t{0.1};
    climbConfig.CurrentLimits = climbCurrentConfig;

    climberMotor1.SetNeutralMode(ctre::phoenix6::signals::NeutralModeValue::Brake);

    
}

// This method will be called once per scheduler run
void Climber::Periodic() {
  bool atBottom = !bottomLimit1.Get();
//   if (atBottom) {
//     climberMotor1.SetPosition(0_tr);
//   }

  frc::SmartDashboard::PutBoolean("/Climber/AtBottom", atBottom);
  frc::SmartDashboard::PutNumber("/Climber/Position_Rotations", climberMotor1.GetPosition().GetValueAsDouble());
  frc::SmartDashboard::PutNumber("/Climber/Position_Inches", GetPositionInches().value());

  // Write out to Log file
  m_PositionLog.Append(climberMotor1.GetPosition().GetValueAsDouble());
  m_PositionInchesLog.Append(GetPositionInches().value());
  m_StateLog.Append(m_ClimberState);
  m_LimitSwitchLog.Append(atBottom);
  m_MotorCurrentLog.Append(climberMotor1.GetSupplyCurrent().GetValue().value());
  m_MotorVoltageLog.Append(climberMotor1.GetMotorVoltage().GetValue().value());
}

void Climber::moveMotor() {
    // climberMotor1.Set(0.1); // retracts when set to 0.1
    climberMotor1.Set(0.0); // retracts when set to 0.1

}

void Climber::stopMotor() {
    climberMotor1.Set(0.0);
}

void Climber::Zero() {
    if (bottomLimit1.Get())
    {
        // climberMotor1.Set(-0.1);
        climberMotor1.Set(0.0);
    }
    else 
    {
        climberMotor1.Set(0.0);
        climberMotor1.SetPosition(0_tr);
    }
}

units::length::inch_t Climber::GetPositionInches() {
    // TalonFX GetPosition returns rotations (units::angle::turn_t)
    return units::length::inch_t{climberMotor1.GetPosition().GetValueAsDouble() * ClimberConstants::InchesPerRotation};
}

void Climber::extend() {
    // Going up (positive direction), but don't exceed 12 inches
    if (GetPositionInches() < ClimberConstants::MaxExtensionInches) {
        // climberMotor1.Set(0.2);
        climberMotor1.Set(0.0);
    } else {
        climberMotor1.Set(0.0);
    }
}

void Climber::retract() {
    // Going down, but don't pull past min retract inches
    if (bottomLimit1.Get() && GetPositionInches() > ClimberConstants::MinRetractInches) {
        // climberMotor1.Set(-0.2);
        climberMotor1.Set(0.0);
    } else {
        climberMotor1.Set(0.0);
    }
}

bool Climber::atBot() {
    return !bottomLimit1.Get();
}
