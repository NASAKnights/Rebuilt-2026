// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/FuelSubsystem.h"

FuelSubsystem::FuelSubsystem(){
    intakeMotor.ConfigVoltageCompSaturation(12.);
    intakeMotor.EnableVoltageCompensation(true);
    feederMotor.ConfigVoltageCompSaturation(12.);
    feederMotor.EnableVoltageCompensation(true);
}

// This method will be called once per scheduler run
void FuelSubsystem::Periodic() {}

void FuelSubsystem::stop() {
    // intakeMotor.Set(0);
    // feederMotor.Set(0);
    intakeMotor.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput,0);
    feederMotor.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput,0);
}
void FuelSubsystem::intake() {
    // intakeMotor.SetVoltage(units::volt_t{FuelConstants::INTAKING_INTAKE_VOLTAGE});
    // feederMotor.SetVoltage(units::volt_t{FuelConstants::INTAKING_FEEDER_VOLTAGE});
    intakeMotor.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, FuelConstants::INTAKING_INTAKE_VOLTAGE/12.);
    feederMotor.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, FuelConstants::INTAKING_FEEDER_VOLTAGE/12.);
}
void FuelSubsystem::eject() {
    // intakeMotor.SetVoltage(units::volt_t{-1*FuelConstants::INTAKING_INTAKE_VOLTAGE});
    // feederMotor.SetVoltage(units::volt_t{-1*FuelConstants::INTAKING_FEEDER_VOLTAGE});
    intakeMotor.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput,-1*FuelConstants::INTAKING_INTAKE_VOLTAGE/12.);
    feederMotor.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput,-1*FuelConstants::INTAKING_FEEDER_VOLTAGE/12.);
}
void FuelSubsystem::launch() {
    // intakeMotor.SetVoltage(units::volt_t{FuelConstants::LAUNCHING_LAUNCHER_VOLTAGE});
    // feederMotor.SetVoltage(units::volt_t{FuelConstants::LAUNCHING_FEEDER_VOLTAGE});
    intakeMotor.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput,FuelConstants::LAUNCHING_LAUNCHER_VOLTAGE/12.);
    feederMotor.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput,FuelConstants::LAUNCHING_FEEDER_VOLTAGE/12.);
}
void FuelSubsystem::spinup() {
    // intakeMotor.SetVoltage(units::volt_t{FuelConstants::LAUNCHING_LAUNCHER_VOLTAGE});
    // feederMotor.SetVoltage(units::volt_t{FuelConstants::SPIN_UP_FEEDER_VOLTAGE});
    intakeMotor.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput,FuelConstants::LAUNCHING_LAUNCHER_VOLTAGE/12.);
    feederMotor.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput,FuelConstants::SPIN_UP_FEEDER_VOLTAGE/12.);
}

