// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <rev/SparkMax.h>
#include <rev/rev/REVLibVersion.h>

class FuelSubsystem : 
public frc2::SubsystemBase {
 public:
  FuelSubsystem();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

  void intake();
  void eject();
  void launch();
  void stop();
  void spinup();
  void IntakeLauncherRoller(double voltage);
  void FeederRoller(double voltage);

 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.

  rev::spark::SparkMax intakeMotor{1, rev::spark::SparkMax::MotorType::kBrushed};
  rev::spark::SparkMax feederMotor{2, rev::spark::SparkMax::MotorType::kBrushed};

};
