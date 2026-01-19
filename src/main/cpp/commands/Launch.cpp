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
void lauch::Initialize() 
  {
 m_fuelSubsystem->IntakeLauncherRoller(
        frc::SmartDashboard::GetNumber("Intaking intake roller value", 10));

    m_fuelSubsystem->FeederRoller(
        frc::SmartDashboard::GetNumber("Intaking feeder roller value", -12));
        }

// Called repeatedly when this Command is scheduled to run
void lauch::Execute() {}

// Called once the command ends or is interrupted.
void lauch::End(bool interrupted) {}

// Returns true when the command should end.
bool lauch::IsFinished() {
  return false;
}
