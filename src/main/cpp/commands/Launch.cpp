// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/Launch.h"

#include <frc/smartdashboard/SmartDashboard.h>

#include "Constants.h"

using namespace Constants;


launch::launch(FuelSubsystem* _fuelSubsystem):m_fuelSubsystem{_fuelSubsystem} {
  AddRequirements(m_fuelSubsystem);
}

void launch::Initialize() {
  m_fuelSubsystem->launch();
}

void launch::Execute() {

}

void launch::End(bool interrupted) {

}

bool launch::IsFinished() {
  return false;
}
