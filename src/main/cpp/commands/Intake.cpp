// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/Intake.h"
#include "Constants.h"
#include <frc/smartdashboard/SmartDashboard.h>
Intake::Intake() {
  // Use addRequirements() here to declare subsystem dependencies.
}

void Intake::Initialize() {
   m_fuelSubsystem->IntakeLauncherRoller(
        frc::SmartDashboard::GetNumber("Intaking intake roller value", 10));

    m_fuelSubsystem->FeederRoller(
        frc::SmartDashboard::GetNumber("Intaking feeder roller value", -12));
}

void Intake::Execute() {

}
void Intake::End(bool interrupted) {
  m_fuelSubsystem->IntakeLauncherRoller(0);
  m_fuelSubsystem->FeederRoller(0);
}


bool Intake::IsFinished() {
  return false;
}
