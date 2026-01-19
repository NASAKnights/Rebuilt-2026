// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/eject.h"

eject::eject(FuelSubsystem fuelSystem) {AddRequirements(fuelSystem)
  this.FuelSubsystem=fuelSystem;
  // Use addRequirements() here to declare subsystem dependencies.
}

// Called when the command is initially scheduled.
void eject::Initialize() {
  FuelSubsystem.setintakeMotor(-10);
  FuelSubsystem.setfeedermotor(12);
}

// Called repeatedly when this Command is scheduled to run
void eject::Execute() {
  }

// Called once the command ends or is interrupted.
void eject::End(bool interrupted) {
   FuelSubsystem.setIntakemotor(0);
   FuelSubsystem.setfeedermotor(0);
}

// Returns true when the command should end.
bool eject::IsFinished() {
  return false;
}
