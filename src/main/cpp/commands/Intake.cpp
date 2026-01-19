// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/Intake.h"
#include "Constants.h"

Intake::Intake() {
  // Use addRequirements() here to declare subsystem dependencies.
}

// Called when the command is initially scheduled.
void Intake::Initialize() {FuelSubsystem
        set.IntakeLauncherRoller(SmartDashboard.getNumber("Intaking intake roller value", INTAKING_INTAKE_VOLTAGE));
    FuelSubsystem set.FeederRoller(SmartDashboard.getNumber("Intaking feeder roller value", INTAKING_FEEDER_VOLTAGE));}

// Called repeatedly when this Command is scheduled to run
void Intake::Execute() {}

// Called once the command ends or is interrupted.
void Intake::End(bool interrupted) {FuelSubsystem set.IntakeLauncherRoller(0);
    FuelSubsystem set.FeederRoller(0);}

// Returns true when the command should end.
bool Intake::IsFinished() {
  return false;
}
