#include "commands/LaunchSequence.h"
#include "commands/spinUp.h"
#include "commands/Launch.h"
#include "Constants.hpp"
#include <units/time.h>
#include <frc2/command/SequentialCommandGroup.h>

LaunchSequence::LaunchSequence(FuelSubsystem* fuelSubsystem) {
  // Add commands to this SequentialCommandGroup instance
  AddCommands(
    spinUp(fuelSubsystem),
    launch(fuelSubsystem)
  );
}
