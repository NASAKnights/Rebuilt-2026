// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/TurretIntake.h"

TurretIntake::TurretIntake()
{
    wpi::log::DataLog& log = frc::DataLogManager::GetLog();
    m_MotorCurrentLog = wpi::log::DoubleLogEntry(log, "/Turret/Intake/MotorCurrent");
    m_MotorVoltageLog = wpi::log::DoubleLogEntry(log, "/Turret/Intake/MotorVoltage");


    ctre::phoenix6::configs::TalonFXConfiguration intakeConfig{};
    // m_intakeMotor.Configure(config, rev::spark::SparkMax::ResetMode::kResetSafeParameters, rev::spark::SparkMax::PersistMode::kPersistParameters);
    ctre::phoenix6::configs::CurrentLimitsConfigs intakeCurrentConfig{};
    intakeCurrentConfig.SupplyCurrentLimitEnable = true;
    intakeCurrentConfig.SupplyCurrentLimit = units::ampere_t{80};
    intakeCurrentConfig.SupplyCurrentLowerLimit = units::ampere_t{35};
    intakeCurrentConfig.SupplyCurrentLowerTime = units::second_t{0.1};
    intakeConfig.CurrentLimits = intakeCurrentConfig;

    m_intakeMotor.GetConfigurator().Apply(intakeConfig);
    m_intakeMotor.SetNeutralMode(ctre::phoenix6::signals::NeutralModeValue::Coast);
}

// This method will be called once per scheduler run
void TurretIntake::Periodic()
{
    m_MotorCurrentLog.Append(m_intakeMotor.GetSupplyCurrent().GetValue().value());
    m_MotorVoltageLog.Append(m_intakeMotor.GetMotorVoltage().GetValue().value());
}

void TurretIntake::Intake()
{
    // m_intakeMotor.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, -0.85);
    m_intakeMotor.Set(-0.75);
}

void TurretIntake::Outtake()
{
    // m_intakeMotor.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0.85);
    m_intakeMotor.Set(0.85);
}

void TurretIntake::StopIntake()
{
    // m_intakeMotor.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0.0);
    m_intakeMotor.Set(0.0);
}
