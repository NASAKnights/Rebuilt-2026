#include "commands/LaunchSequence.h"

#include "commands/SpinUp.h"
#include "commands/Launch.h"
#include "Constants.h"

LaunchSequence::LaunchSequence(CANFuelSubsystem* fuelSubsystem) {
  AddCommands(
      SpinUp(fuelSubsystem).WithTimeout(FuelConstants::SPIN_UP_SECONDS),
      Launch(fuelSubsystem)
  );
}
