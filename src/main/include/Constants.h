#pragma once

namespace Constants {

  namespace DriveConstants {
    // Motor controller IDs for drivetrain motors
    constexpr int LEFT_LEADER_ID = 1;
    constexpr int LEFT_FOLLOWER_ID = 2;
    constexpr int RIGHT_LEADER_ID = 3;
    constexpr int RIGHT_FOLLOWER_ID = 4;

    // Current limit for drivetrain motors
    constexpr int DRIVE_MOTOR_CURRENT_LIMIT = 60;
  }

  namespace FuelConstants {
    // Motor controller IDs for Fuel Mechanism motors
    constexpr int FEEDER_MOTOR_ID = 6;
    constexpr int INTAKE_LAUNCHER_MOTOR_ID = 5;

    // Current limits for fuel mechanism motors
    constexpr int FEEDER_MOTOR_CURRENT_LIMIT = 60;
    constexpr int LAUNCHER_MOTOR_CURRENT_LIMIT = 60;

    // Voltage values for various fuel operations
    constexpr double INTAKING_FEEDER_VOLTAGE = -12.0;
    constexpr double INTAKING_INTAKE_VOLTAGE = 10.0;
    constexpr double LAUNCHING_FEEDER_VOLTAGE = 9.0;
    constexpr double LAUNCHING_LAUNCHER_VOLTAGE = 10.6;
    constexpr double SPIN_UP_FEEDER_VOLTAGE = -6.0;
    constexpr double SPIN_UP_SECONDS = 1.0;
  }

  namespace OperatorConstants {
    // Controller ports
    constexpr int DRIVER_CONTROLLER_PORT = 0;
    constexpr int OPERATOR_CONTROLLER_PORT = 1;

    // Drive scaling factors
    constexpr double DRIVE_SCALING = 0.7;
    constexpr double ROTATION_SCALING = 0.8;
  }

}  // namespace Constants
