// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/Launch.h"

#include <frc/smartdashboard/SmartDashboard.h>

#include "Constants.h"

using namespace Constants;


lauch::lauch() {
  // Use addRequirements() here to declare subsystem dependencies.
}

// Called when the command is initially scheduled.
void lauch::Initialize() {FuelSubsystem Set.IntakeLauncherRoller(
      frc::SmartDashboard::GetNumber(
          "Launching launcher roller value",
          LAUNCHING_LAUNCHER_VOLTAGE));
          FuelSubsystem set.FeederRoller(
      frc::SmartDashboard::GetNumber(
          "Launching feeder roller value",
          LAUNCHING_FEEDER_VOLTAGE));}

// Called repeatedly when this Command is scheduled to run
void lauch::Execute() {}

// Called once the command ends or is interrupted.
void lauch::End(bool interrupted) {}

// Returns true when the command should end.
bool lauch::IsFinished() {
  return false;
}
