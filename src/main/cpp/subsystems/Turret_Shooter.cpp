// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Turret_Shooter.h"

Turret_Shooter::Turret_Shooter()
{
    ctre::phoenix6::configs::TalonFXSConfiguration leftMotorConfig{};
    ctre::phoenix6::configs::TalonFXSConfiguration rightMotorConfig{};
    ctre::phoenix6::configs::Slot0Configs motorSlot0Configs{};
    motorSlot0Configs.kP = kP;
    motorSlot0Configs.kI = kI;
    motorSlot0Configs.kD = kD;
    motorSlot0Configs.kS = kS;
    motorSlot0Configs.kA = kA;
    motorSlot0Configs.kV = kV;
    leftMotorConfig.Slot0 = motorSlot0Configs;
    rightMotorConfig.Slot0 = motorSlot0Configs;

    ctre::phoenix6::configs::CurrentLimitsConfigs currentConfig{};
    currentConfig.SupplyCurrentLimitEnable = kEnableCurrentLimit;
    currentConfig.SupplyCurrentLimit = kPeakCurrentLimit;
    currentConfig.SupplyCurrentLowerLimit = kContinousCurrentLimit;
    currentConfig.SupplyCurrentLowerTime = kPeakCurrentDuration;
    leftMotorConfig.CurrentLimits = currentConfig;
    rightMotorConfig.CurrentLimits = currentConfig;

    leftMotorConfig.MotorOutput.Inverted = false;
    rightMotorConfig.MotorOutput.Inverted = true;

    ctre::phoenix::StatusCode leftStatus = m_leftMotor.GetConfigurator().Apply(leftMotorConfig);
    ctre::phoenix::StatusCode rightStatus = m_rightMotor.GetConfigurator().Apply(rightMotorConfig);

    frc::SmartDashboard::PutBoolean("Shooter/Left_Status", false);
    frc::SmartDashboard::PutBoolean("Shooter/Right_Status", false);

    
    while (!leftStatus.IsOK()) {
        ctre::phoenix::StatusCode leftStatus = m_leftMotor.GetConfigurator().Apply(leftMotorConfig);
    }
    while (!rightStatus.IsOK()) {
        ctre::phoenix::StatusCode rightStatus = m_rightMotor.GetConfigurator().Apply(rightMotorConfig);
    }
    

    frc::SmartDashboard::PutBoolean("Shooter/Left_Status", leftStatus.IsOK());
    frc::SmartDashboard::PutBoolean("Shooter/Right_Status", rightStatus.IsOK());

    frc::SmartDashboard::PutNumber("Shooter/Commanded_Motor_Speed", 0.0);
    frc::SmartDashboard::PutNumber("Shooter/Ball_Speed_Manual_Set", 0.0);
    frc::SmartDashboard::PutNumber("Shooter/Actual_Motor_Speed", 0.0);
    frc::SmartDashboard::PutNumber("Shooter/Actual_Ball_Speed", 0.0);
}

void Turret_Shooter::SetSpeed(units::meters_per_second_t ballSpeed) {
    auto motorRequest = ctre::phoenix6::controls::VelocityVoltage{0_tps};
    units::turns_per_second_t motorSpeed = (ballSpeed * units::radian_t{1} * 4.0) / (kFlywheelDiameter * kGearRatio);
    m_leftMotor.SetControl(motorRequest.WithVelocity(motorSpeed));
    m_rightMotor.SetControl(motorRequest.WithVelocity(motorSpeed));
    frc::SmartDashboard::PutNumber("Shooter/Commanded_Motor_Speed", motorSpeed.value());
}

void Turret_Shooter::StopMotors()
{
    auto motorRequest = ctre::phoenix6::controls::VelocityVoltage{0_tps};
    auto motorSpeed = units::radians_per_second_t{0.0};
    m_leftMotor.SetControl(motorRequest.WithVelocity(motorSpeed));
    m_rightMotor.SetControl(motorRequest.WithVelocity(motorSpeed));
    frc::SmartDashboard::PutNumber("Shooter/Commanded_Motor_Speed", motorSpeed.value());
}

void Turret_Shooter::Periodic()
{
    frc::SmartDashboard::PutNumber("Shooter/Actual_Motor_Speed", m_leftMotor.GetVelocity().GetValue().value()); // left and right motors are same speed
    auto ballSpeed = (m_leftMotor.GetVelocity().GetValue().value() * 2.0 * std::numbers::pi * kFlywheelDiameter * kGearRatio) / 4.0;
    frc::SmartDashboard::PutNumber("Shooter/Actual_Ball_Speed", ballSpeed.value()); // left and right motors are same speed
    SetSpeed(units::meters_per_second_t{frc::SmartDashboard::GetNumber("Shooter/Ball_Speed_Manual_Set", 0.0)});
}
