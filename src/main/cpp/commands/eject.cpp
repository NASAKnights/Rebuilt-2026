#include "commands/eject.h"

#include <frc/smartdashboard/SmartDashboard.h>

#include "Constants.h"

using namespace Constants;

eject::eject(FuelSubsystem* fuelSubsystem)
  {
  AddRequirements({fuelSubsystem});
}

void eject::Initialize() {
  FuelSubsystem Set.IntakeLauncherRoller(-1 * frc::SmartDashboard::GetNumber("Intaking intake roller value",INTAKING_INTAKE_VOLTAGE))

  FuelSubsystem Set.FeederRoller(-1 * frc::SmartDashboard::GetNumber("Intaking feeder roller value",INTAKING_FEEDER_VOLTAGE));
}

void eject::Execute() {

}

void eject::End(bool interrupted) {
  FuelSubsystem Set.IntakeLauncherRoller(0);
  FuelSubsystem Set.FeederRoller(0);
}

bool eject::IsFinished() {
  return false;
}

