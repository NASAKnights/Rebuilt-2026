// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <ctre/phoenix/motorcontrol/can/VictorSPX.h>
#include <rev/SparkMax.h>
#include <rev/config/SparkMaxConfig.h>
#include <ctre/phoenix6/configs/Configurator.hpp>
#include <ctre/phoenix6/TalonFX.hpp>
#include <ctre/phoenix6/configs/Configuration.hpp>
#include <frc/DataLogManager.h>
#include <wpi/DataLog.h>

class TurretIntake : public frc2::SubsystemBase
{
public:
  TurretIntake();

  void Intake();
  void Outtake();
  void StopIntake();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

private:
  // ctre::phoenix::motorcontrol::can::VictorSPX m_intakeMotor{8};
  // rev::spark::SparkMax m_intakeMotor{8, rev::spark::SparkLowLevel::MotorType::kBrushless};
  ctre::phoenix6::hardware::TalonFX m_intakeMotor{8};
  wpi::log::DoubleLogEntry m_MotorCurrentLog;
  wpi::log::DoubleLogEntry m_MotorVoltageLog;
  

  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
};
