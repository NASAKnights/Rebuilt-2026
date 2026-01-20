#include "commands/spinUp.h"
#include "Constants.h"

spinUp::spinUp(FuelSubsystem* _fuelSubsystem):m_fuelSubsystem{_fuelSubsystem} {
  AddRequirements(m_fuelSubsystem);
}

void spinUp::Initialize() {
  m_fuelSubsystem->spinup();
  time.Reset();
  time.Start();
}

void spinUp::Execute() {
}

void spinUp::End(bool interrupted) {
}

bool spinUp::IsFinished() {
  if(time.HasElapsed(units::second_t{Constants::FuelConstants::SPIN_UP_SECONDS})) {
    return true;
  } else {
    return false;
  }
}
