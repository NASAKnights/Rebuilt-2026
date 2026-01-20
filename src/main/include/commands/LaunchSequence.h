#pragma once

#include <frc2/command/CommandHelper.h>
#include <frc2/command/SequentialCommandGroup.h>
#include "subsystems/FuelSubsystem.h"

class LaunchSequence : public frc2::SequentialCommandGroup {
 public:
  explicit LaunchSequence(FuelSubsystem* fuelSubsystem);
};
