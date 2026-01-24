// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <rev/SparkFlex.h>
#include <rev/SparkMax.h>
// #include <ctre/phoenix6/TalonFX.hpp>
#include <frc/motorcontrol/PWMMotorController.h>
#include <ctre/phoenix6/TalonFXS.hpp>
#include <units/angle.h>
#include <units/velocity.h>
#include <units/acceleration.h>
#include <units/angular_velocity.h>
#include <units/angular_acceleration.h>
#include <units/voltage.h>
#include <units/moment_of_inertia.h>

#include <rev/SparkBase.h>
#include <frc2/command/PIDCommand.h>
#include <frc2/command/PIDSubsystem.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/controller/SimpleMotorFeedforward.h>

class Turret_Shooter : public frc2::SubsystemBase
{
public:
  Turret_Shooter();

  void Periodic() override;

  void StopMotors();
  void SetSpeed(units::meters_per_second_t speed); // speed of the ball leaving the shooter

private:
  ctre::phoenix6::hardware::TalonFXS m_leftMotor{kMotorIdLeft};
  ctre::phoenix6::hardware::TalonFXS m_rightMotor{kMotorIdRight};

  static constexpr units::inch_t kFlywheelDiameter = units::inch_t{2.625};
  static constexpr int kGearRatio = 2;
  static constexpr units::inch_t kBallDiameter = units::inch_t{5.91};

  double shooterSpeed = 0.0;
  double newShooterSpeed = 0.0;

  double kP = 0.005;
  double kI = 0.0;
  double kD = 0.0;
  double kS = 0.0;
  double kA = 0.0;
  double kV = 0.0;

  const int kMotorIdLeft = 5;
  const int kMotorIdRight = 6;

  bool kEnableCurrentLimit = true;
  units::ampere_t kPeakCurrentLimit = units::ampere_t{53};
  units::ampere_t kContinousCurrentLimit = units::ampere_t{40};
  units::second_t kPeakCurrentDuration = units::second_t{0.1};

  double kMinOutput = -1.0;
  double kMaxOutput = 1.0;
};