// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/FuelSubsystem.h"

FuelSubsystem::FuelSubsystem(){

}

// This method will be called once per scheduler run
void FuelSubsystem::Periodic() {}

void FuelSubsystem::stop() {
    intakeMotor.Set(0);
    feederMotor.Set(0);

}
void FuelSubsystem::intake() { intakeMotor.set(-12);
    feederMotor.set(10);}
void FuelSubsystem::eject() {
    intakeMotor.set(10);
    feederMotor.set(-12);
}
void FuelSubsystem::launch() {
    feederMotor.set(9);
    intakeMotor.set(10.6);
}
void FuelSubsystem::spinUp() {
    feederMotor.setVoltage(-6);
    intakeMotor.set(10.6);
}