// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/eject.h"
#include "Constants.hpp"

eject::eject(FuelSubsystem* _fuelSubsystem):m_fuelSubsystem{_fuelSubsystem} {
  AddRequirements(m_fuelSubsystem);
}

void eject::Initialize() {
   m_fuelSubsystem->eject();
}

void eject::Execute() {

}

void eject::End(bool interrupted) {
  m_fuelSubsystem->stop();
}

bool eject::IsFinished() {
  return false;
}

