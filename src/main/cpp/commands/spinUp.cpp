#include "commands/spinUp.h"

#include <frc/smartdashboard/SmartDashboard.h>

#include "Constants.h"

spinUp::spinUp() {

void spinUp::Initialize() {
  m_fuelSubsystem->SetIntakeLauncherRoller(
      frc::SmartDashboard::GetNumber(
          "Launching launcher roller value",
          FuelConstants::LAUNCHING_LAUNCHER_VOLTAGE));

  m_fuelSubsystem->SetFeederRoller(
      frc::SmartDashboard::GetNumber(
          "Launching spin-up feeder value",
          FuelConstants::SPIN_UP_FEEDER_VOLTAGE));
}


void spinUp::Execute() {
  // No periodic updates required
}

void spinUp::End(bool interrupted) {
  // No shutdown behavior (matches Java)
}

bool spinUp::IsFinished() {
  return false;
}
