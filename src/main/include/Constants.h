#pragma once

namespace Constants {

    struct DriveConstants {
        static constexpr int LEFT_LEADER_ID = 1;
        static constexpr int LEFT_FOLLOWER_ID = 2;
        static constexpr int RIGHT_LEADER_ID = 3;
        static constexpr int RIGHT_FOLLOWER_ID = 4;

        static constexpr int DRIVE_MOTOR_CURRENT_LIMIT = 60;
    };

    struct FuelConstants {
        static constexpr int FEEDER_MOTOR_ID = 6;
        static constexpr int INTAKE_LAUNCHER_MOTOR_ID = 5;

        static constexpr int FEEDER_MOTOR_CURRENT_LIMIT = 60;
        static constexpr int LAUNCHER_MOTOR_CURRENT_LIMIT = 60;

        static constexpr double INTAKING_FEEDER_VOLTAGE = -12.0;
        static constexpr double INTAKING_INTAKE_VOLTAGE = 10.0;
        static constexpr double LAUNCHING_FEEDER_VOLTAGE = 9.0;
        static constexpr double LAUNCHING_LAUNCHER_VOLTAGE = 10.6;
        static constexpr double SPIN_UP_FEEDER_VOLTAGE = -6.0;
        static constexpr double SPIN_UP_SECONDS = 1.0;
    };

    struct OperatorConstants {
        static constexpr int DRIVER_CONTROLLER_PORT = 0;
        static constexpr int OPERATOR_CONTROLLER_PORT = 1;

        static constexpr double DRIVE_SCALING = 0.7;
        static constexpr double ROTATION_SCALING = 0.8;
    };

}
