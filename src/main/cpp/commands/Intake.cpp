// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/Intake.h"
#include "Constants.h"

Intake::Intake(FuelSubsystem* _fuelSubsystem):m_fuelSubsystem{_fuelSubsystem} {
  AddRequirements(m_fuelSubsystem);
}

void Intake::Initialize() {
   m_fuelSubsystem->intake();
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
