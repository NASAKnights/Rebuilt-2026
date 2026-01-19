#include "commands/eject.h"

#include <frc/smartdashboard/SmartDashboard.h>

#include "Constants.h"

using namespace Constants;

eject::eject(FuelSubsystem* fuelSubsystem)
  {
  AddRequirements({fuelSubsystem});
}

void eject::Initialize() {
   m_fuelSubsystem->IntakeLauncherRoller(
        -frc::SmartDashboard::GetNumber("Intaking intake roller value", 10));

    m_fuelSubsystem->FeederRoller(
        -frc::SmartDashboard::GetNumber("Intaking feeder roller value", -12));
}

void eject::Execute() {

}
void eject::End(bool interrupted) {
  m_fuelSubsystem->IntakeLauncherRoller(0);
  m_fuelSubsystem->FeederRoller(0);
}


bool eject::IsFinished() {
  return false;
}

