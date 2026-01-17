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
void FuelSubsystem::intake() {
 intakeLauncherRoller.setVoltage(SmartDashboard.getnumber(Intaking intake roller value, Intaking_intake_voltage));

    feederMotor.setVoltage(SmartDashboard.getnumber(Intaking feeder roller value, Intaking_feeder_voltage));
}
void FuelSubsystem::eject() {
    intakeLauncherRoller.setVoltage(-1*SmartDashboard.getnumber("Intaking intake roller value", Intaking_intake_voltage));
    feederMotor.setVoltage(-1*SmartDashboard.getnumber("Intaking feeder roller value", Intaking_feeder_voltage));
}
void FuelSubsystem::launch() {
    feederRoller.setVoltage(SmartDashboard.getNumber("Launching feeder roller value", LAUNCHING_FEEDER_VOLTAGE));
    intakeLauncherRoller.setVoltage(SmartDashboard.getNumber("Launching launcher roller value", LAUNCHING_LAUNCHER_VOLTAGE));
}
void FuwlSubsystem::spinUp() {
    feederRoller.setVoltage(SmartDashboard.getNumber("Spin-up feeder roller value", SPIN_UP_FEEDER_VOLTAGE));
    intakeLauncherRoller.setVoltage(SmartDashboard.getNumber("Launching launcher roller value", LAUNCHING_LAUNCHER_VOLTAGE));
}