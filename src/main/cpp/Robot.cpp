// Copyright (c) FRC Team 122. All Rights Reserved.

#include "Robot.hpp"


Robot::Robot() : networkTableInst(nt::NetworkTableInstance::GetDefault())
{
    this->CreateRobot();
}

// This function is called during startup
void Robot::RobotInit()
{
    frc::DataLogManager::Start();
    wpi::log::DataLog &log = frc::DataLogManager::GetLog();
    
    m_VoltageLog = wpi::log::DoubleLogEntry(log, "/PDP/Voltage");
    m_CurrentLog = wpi::log::DoubleLogEntry(log, "/PDP/Current");
    m_PowerLog = wpi::log::DoubleLogEntry(log, "/PDP/Power");
    m_EnergyLog = wpi::log::DoubleLogEntry(log, "/PDP/Energy");
    m_TemperatureLog = wpi::log::DoubleLogEntry(log, "/PDP/Temperature");
    m_BatteryLog = wpi::log::DoubleLogEntry(log, "Robot/Battery");
    
    frc::SmartDashboard::PutString("POIName", "");
    frc::SmartDashboard::PutData("AddPOI", addPOICommand.get());
    frc::SmartDashboard::PutData("RemovePOI", removePOICommand.get());
    frc::SmartDashboard::PutData("Set", autoWheelOffsetsCommand.get());
    frc::SmartDashboard::PutNumber("hoodAngle", 1.0);
    
    autoChooser = pathplanner::AutoBuilder::buildAutoChooser();

    frc::SmartDashboard::PutData("Auto Chooser", &autoChooser);
    
    auto sdTable = networkTableInst.GetTable("SmartDashboard");
    modelPosePublisher = sdTable->GetStructArrayTopic<frc::Pose3d>("ModelPoses").Publish();
}

// This function is called every 20 ms
void Robot::RobotPeriodic()
{
    frc2::CommandScheduler::GetInstance().Run();
    this->UpdateDashboard();
    m_POVloop.Poll();
    
    m_VoltageLog.Append(m_pdh.GetVoltage());
    m_CurrentLog.Append(m_pdh.GetTotalCurrent());
    m_PowerLog.Append(m_pdh.GetTotalPower());
    m_EnergyLog.Append(m_pdh.GetTotalEnergy());
    m_TemperatureLog.Append(m_pdh.GetTemperature());
    m_BatteryLog.Append(batteryShunt.GetVoltage());
    testingRotation += units::angle::radian_t{(1*3.14159)/180};
    frc::Pose2d pose = frc::Pose2d(units::length::meter_t{0.0}, units::length::meter_t{0.0}, frc::Rotation2d{});

    frc::Pose3d IntakePose3D = frc::Pose3d(pose.X(),
                                        pose.Y(),
                                        0.0_m,
                                        frc::Rotation3d(units::angle::radian_t{(m_wrist.GetMeasurement()*3.14159)/180}, 0.0_rad, 0.0_rad));

    frc::Pose3d SpindexerPose3D = frc::Pose3d(pose.X(),
                                        pose.Y(),
                                        0.0_m,
                                        frc::Rotation3d(0.0_rad, 0.0_rad, 0.0_rad));

    frc::Pose3d ShooterPose3D = frc::Pose3d(pose.X()-0.18_m,
                                        pose.Y()+0.18_m,
                                        0.45_m,
                                        frc::Rotation3d(0.0_rad, 0.0_rad, units::radian_t{m_turret.GetMeasurement()}));

    // frc::Pose3d ShooterPose3D = frc::Pose3d(pose.X()-0.18_m,
    //                                     pose.Y()+0.18_m,
    //                                     0.45_m,
    //                                     frc::Rotation3d(0.0_rad, 0.0_rad, 0.0_rad));

    frc::Pose3d HoodPose3D = frc::Pose3d(units::meter_t{ 1 * TurretConstants::kHoodXOffset *(std::cos(double(m_turret.GetMeasurement()) - 90))},
                                        units::meter_t{ 1 * TurretConstants::kHoodXOffset *(std::sin(double(m_turret.GetMeasurement()) - 90))},
                                        0.545_m,
                                        frc::Rotation3d(0.0_rad, units::radian_t{((90 - m_turret.GetHoodAngle())*3.14159)/180} , units::radian_t{m_turret.GetMeasurement()}));

    // frc::Pose3d HoodPose3D = frc::Pose3d(ShooterPose3D.X() + units::meter_t{TurretConstants::kHoodXOffset *(std::cos(0.0))},
    //                                     ShooterPose3D.Y() + units::meter_t{TurretConstants::kHoodYOffset *(std::sin(0.0))},
    //                                     0.545_m,
    //                                     frc::Rotation3d(0.0_rad, units::radian_t{((90 - m_turret.GetHoodAngle())*3.14159)/180} , 0.0_rad));

    std::vector<frc::Pose3d> modelPoses = {
        ShooterPose3D,
        HoodPose3D,
        IntakePose3D,
        SpindexerPose3D
    };
    modelPosePublisher.Set(modelPoses, 0);
}

// This function is called once each time the robot enters Disabled mode.
void Robot::DisabledInit()
{
    // m_LED_Controller.DefaultAnimation();
    if constexpr (frc::RobotBase::IsSimulation())
    {
        m_swerveDrive.ResetPose(frc::Pose2d());
        m_swerveDrive.ResetDriveEncoders();
    }

    auto turretMap = m_turret.GetCurrentMapState();
    auto hoodMap = m_turret.m_turret_shooter.GetCurrentMapState();

    frc::SmartDashboard::PutBoolean("HELP/", firstBoot);
    if(!firstBoot){
        std::ofstream writeFile(csvName);

        for (const auto& [distance, speed] : turretMap)
        {
            double hood = hoodMap[distance];
            writeFile << distance << "," << speed << "," << hood << "\n";
        }
        writeFile.close();
    }
    else {
        LoadCSVToMap(csvName);
    }
    firstBoot = false;
}

void Robot::SetAutonomousCommand(std::string a)
{
        
}

void Robot::AutonomousInit()
{
    // m_autonomousCommand = this->GetAutonomousCommand();
    // m_swerveDrive.TurnVisionOff(); // don't use vision during Auto
    auto m_autonomousCommand = autoChooser.GetSelected();
    m_swerveDrive.ResetPose(autoStartPose);

    m_swerveDrive.InvertHeading();

    if (m_autonomousCommand)
    {
        m_autonomousCommand->Schedule();
    }
}

void Robot::AutonomousPeriodic() {}

void Robot::AutonomousExit()
{
}

void Robot::TeleopInit()
{
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    // m_wrist.HoldPosition();
    m_turret.Reset();
    /*
    if (m_wrist.GetState() != WristConstants::WristState::ZEROING)
    {
        m_wrist.SetAngle(m_wrist.GetMeasurement().value());
    }
    */

    if (m_autonomousCommand)
    {
        m_autonomousCommand->Cancel();
    }
    m_swerveDrive.TurnVisionOn(); // Turn Vision back on for Teleop
    // m_LED_Controller.TeleopLED();
}

void Robot::TeleopPeriodic()
{

}

void Robot::TeleopExit()
{
}

/**
 * This function is called periodically during test mode.
 */
void Robot::TestPeriodic() {}

/**
 * This function is called once when the robot is first started up.
 */
void Robot::SimulationInit() {}

/**
 * This function is called periodically whilst in simulation.
 */
void Robot::SimulationPeriodic() {}

/**
 * Initializes the robot subsystems and binds commands
 */
void Robot::CreateRobot()
{
    // NOTE: THIS WAS FOR REEFSCAPE PSEDUO-AUTO ALIGNMENT WITH THE REEF,
    //  WE SHOULD LATER ATTEMPT TO SEPARATE THIS FROM THE ROBOT.CPP AND MAKE IT MORE FLEXABLE FOR MORE GENERAL ALIGNMENT TO POI's
    //  scoreClosest = frc2::CommandPtr(
    //      frc2::cmd::RunOnce(
    //          [&]()
    //          {
    //              using namespace pathplanner;
    //              using namespace frc;
    //              Pose2d currentPose = this->m_swerveDrive.GetPose();
    //              // Select Left or Right Branch
    //              frc::Transform2d offset = m_driverController.GetRawButton(7) ?
    //                  frc::Transform2d(0.0_m, 0.35_m, frc::Rotation2d()) :
    //                  frc::Transform2d(0.0_m, 0.0_m, frc::Rotation2d());

    //             // The rotation component in these poses represents the direction of travel
    //             Pose2d startPos = Pose2d(currentPose.Translation(), Rotation2d());
    //             Pose2d endPos = m_poiGenerator.GetClosestPOI().TransformBy(offset);

    //             auto transformedEndPos = endPos.TransformBy(Transform2d(0.25_m, 0_m, 0_rad));
    //             std::vector<Waypoint> waypoints = PathPlannerPath::waypointsFromPoses({startPos, endPos, transformedEndPos});
    //             // Paths must be used as shared pointers
    //             auto path = std::make_shared<PathPlannerPath>(
    //                 waypoints,
    //                 std::vector<RotationTarget>({RotationTarget(0.25, endPos.Rotation())}),
    //                 std::vector<PointTowardsZone>(),
    //                 std::vector<ConstraintsZone>(),
    //                 std::vector<EventMarker>(),
    //                 PathConstraints(1_mps, 1.5_mps_sq, 360_deg_per_s, 940_deg_per_s_sq),
    //                 // PathConstraints(1_mps, 2.0_mps_sq, 360_deg_per_s, 940_deg_per_s_sq),
    //                 std::nullopt, // Ideal starting state can be nullopt for on-the-fly paths
    //                 GoalEndState(0_mps, endPos.Rotation()),
    //                 false
    //             );

    //             // Prevent this path from being flipped on the red alliance, since the given positions are already correct
    //             path->preventFlipping = true;

    //             m_pathfind = frc2::CommandPtr(AutoBuilder::followPath(path).Unwrap());
    //             m_pathfind.Schedule(); })
    //         .Unwrap());

    pathplanner::NamedCommands::registerCommand("Intake", Intake(&m_intake, &m_wrist).ToPtr());
    pathplanner::NamedCommands::registerCommand("FlattenMoonKnight", FlattenMoonKnight(&m_turret, &m_wrist).ToPtr());
    pathplanner::NamedCommands::registerCommand("HalfRaiseIntake", HalfRaiseIntake(&m_intake, &m_wrist).ToPtr());
    pathplanner::NamedCommands::registerCommand("Shoot", Shoot(&m_turret, true).ToPtr());
    pathplanner::NamedCommands::registerCommand("NoShoot", Shoot(&m_turret, false).ToPtr());
    pathplanner::NamedCommands::registerCommand("BetterHalfRaise", BetterHalfRaise(&m_intake, &m_wrist).ToPtr());
    // pathplanner::NamedCommands::registerCommand("ExtendClimb", Climb(&m_climber, true).ToPtr());
    // pathplanner::NamedCommands::registerCommand("RetractClimb", Climb(&m_climber, false).ToPtr());

    // pathplanner::NamedCommands::registerCommand("StopShoot", );

    // pathplanner::EventTrigger("Intake").WhileTrue(std::move(Intake(&m_intake, &m_wrist).ToPtr()));
    // pathplanner::EventTrigger("FlattenMoonKnight").WhileTrue(std::move(FlattenMoonKnight(&m_turret, &m_wrist).ToPtr()));
    // pathplanner::EventTrigger("Shoot").WhileTrue(std::move(Shoot(&m_turret).ToPtr()));

    // pathplanner::EventTrigger("ExtendClimb").WhileTrue(std::move(Climb(&m_climber, true).ToPtr()));
    // pathplanner::EventTrigger("RetractClimb").WhileTrue(std::move(Climb(&m_climber, false).ToPtr()));

    m_swerveDrive.SetDefaultCommand(frc2::RunCommand(
        [this]
        {
            auto controllerIn = m_driverController.GetRawButton(5);
            // bool approach = 0;

            auto leftXAxis = MathUtilNK::calculateAxis(m_driverController.GetRawAxis(1),
                                                       DriveConstants::kDefaultAxisDeadband);
            auto leftYAxis = MathUtilNK::calculateAxis(m_driverController.GetRawAxis(0),
                                                       DriveConstants::kDefaultAxisDeadband);
            auto rightXAxis = MathUtilNK::calculateAxis(m_driverController.GetRawAxis(4),
                                                        DriveConstants::kDefaultAxisDeadband);

            // m_swerveDrive.WeightedDriving(approach, leftXAxis, leftYAxis, rightXAxis, targetKey);

            if (controllerIn)
                // Robot-Oriented Drive
                m_swerveDrive.Drive(frc::ChassisSpeeds::FromFieldRelativeSpeeds(
                    -leftXAxis * DriveConstants::kMaxTranslationalVelocity,
                    -leftYAxis * DriveConstants::kMaxTranslationalVelocity,
                    -rightXAxis * DriveConstants::kMaxRotationalVelocity, frc::Rotation2d()));
            else
            {
                m_swerveDrive.Drive(frc::ChassisSpeeds::FromFieldRelativeSpeeds(
                    -leftXAxis * DriveConstants::kMaxTranslationalVelocity,
                    -leftYAxis * DriveConstants::kMaxTranslationalVelocity,
                    -rightXAxis * DriveConstants::kMaxRotationalVelocity, m_swerveDrive.GetHeading()));
            }
        },
        {&m_swerveDrive}));


    AddPeriodic([this]
                { m_wrist.Periodic(); },
                10_ms, 2_ms);
    
    AddPeriodic([this]
                { m_turret.Periodic(); },
                5_ms, 1_ms);
    // AddPeriodic([this]
    //             { m_climber.Periodic(); },
    //             20_ms, 2_ms);

    // Configure the button bindings
    BindCommands();
    m_swerveDrive.ResetHeading();
    // m_LED_Controller.DefaultAnimation();
}

/**
 * Binds commands to Joystick buttons
 */
void Robot::BindCommands()
{

    // --------------DRIVER BUTTONS----------------------------------
    frc2::JoystickButton(&m_driverController, 3)
        .OnTrue(frc2::CommandPtr(
            frc2::InstantCommand([this]
                                 { return m_swerveDrive.ResetHeading(); })));

    frc2::JoystickButton(&m_driverController, 1)
        .OnTrue(frc2::CommandPtr(
            frc2::InstantCommand([this]
                                { m_turret.FindLimitSwitch();
                                return; })));

    // frc2::JoystickButton(&m_driverController, 4)
    //         .OnTrue(frc2::CommandPtr(
    //             frc2::InstantCommand([this]
    //                                 { m_wrist.SetAngle(90);
    //                                 return; })));
                                
    
    // frc2::JoystickButton(&m_operatorController, 5)
    // .OnTrue(frc2::CommandPtr(
    //             frc2::InstantCommand([this]
    //                                 {m_swerveDrive.SetSlow();
    //                                     return; })))
    //         .OnFalse(frc2::CommandPtr(
    //             frc2::InstantCommand([this]
    //                                 { m_swerveDrive.SetFast();
    //                                 return; })));
    
    

    // frc2::JoystickButton(&m_driverController, 3)
    //     .OnTrue(scoreClosest.get())
    //     .OnFalse(frc2::CommandPtr(
    //         frc2::InstantCommand([this]
    //                              { return m_pathfind.Cancel(); })));

    // --------------OPERATOR BUTTONS--------------------------------

    // frc2::JoystickButton(&m_operatorController,1)
    //     .OnTrue(frc2::CommandPtr(
        //         frc2::InstantCommand([this]
        //                                     { double hoodAngle = 0.7;
        //                                         return m_turret.ChangeHoodAngle(hoodAngle); }))) //0.004 is the smallest movement it can do
        //     .OnFalse(frc2::CommandPtr(
            //         frc2::InstantCommand([this]
            //                                     { return m_turret.ChangeHoodAngle(0); })));
        
        frc2::JoystickButton(&m_operatorController, 1)
                .OnTrue(frc2::CommandPtr(
                    frc2::InstantCommand([this]
                                        { m_wrist.SetAngle(60);
                                        m_intake.Intake();
                                        return; })))
                .OnFalse(frc2::CommandPtr(
                frc2::InstantCommand([this]
                                        { m_wrist.SetAngle(4);
                                        m_intake.StopIntake(); 
                                        return;})));

        frc2::JoystickButton(&m_operatorController, 5)
        .OnTrue(frc2::CommandPtr(frc2::InstantCommand(
            [this]
            {
                m_turret.AllowShooting();
                return;
            })))
            .OnFalse(frc2::CommandPtr(frc2::InstantCommand(
                [this]
        {
            m_turret.PauseShooting();
            return;
        })));

        frc2::JoystickButton(&m_operatorController, 6)
                .OnTrue(frc2::CommandPtr(
                    frc2::InstantCommand([this]
                                        { m_wrist.SetAngle(4.);
                                        m_intake.Intake();
                                        return; })))
                .OnFalse(frc2::CommandPtr(
                frc2::InstantCommand([this]
                                        { return m_intake.StopIntake(); })));

        frc2::JoystickButton(&m_driverController, 5)
        .OnTrue(frc2::CommandPtr(frc2::InstantCommand(
            [this]
            {
                m_turret.AllowShooting();
                return;
            })))
            .OnFalse(frc2::CommandPtr(frc2::InstantCommand(
                [this]
        {
            m_turret.PauseShooting();
            return;
        })));

        frc2::JoystickButton(&m_driverController, 6)
                .OnTrue(frc2::CommandPtr(
                    frc2::InstantCommand([this]
                                        { m_wrist.SetAngle(4.);
                                        m_intake.Intake();
                                        return; })))
                .OnFalse(frc2::CommandPtr(
                frc2::InstantCommand([this]
                                        { return m_intake.StopIntake(); })));


        frc2::JoystickButton(&m_driverController, 2)
        .OnTrue(frc2::CommandPtr(frc2::InstantCommand([this] { m_wrist.SetAngle(101.0);
                                                            m_intake.Intake(); })))
        .OnFalse(frc2::CommandPtr(
                frc2::InstantCommand([this]
                                        { return m_intake.StopIntake(); })));

        
        frc::BooleanEvent downPOVDriverBE = frc::BooleanEvent(
            &m_POVloop,
            [&controller = m_driverController]{
                return (controller.GetPOV()>=135) && (controller.GetPOV()<=225);
            }
        ).Debounce(0.2_s);

        frc2::Trigger POVDownTrigBR = downPOVDriverBE.CastTo<frc2::Trigger>();
        POVDownTrigBR.WhileTrue(
                        frc2::CommandPtr(frc2::RunCommand([this] {
                            m_swerveDrive.MakeX(true);
                        })))
                    .OnFalse(
                        frc2::CommandPtr(frc2::InstantCommand([this] {
                            m_swerveDrive.MakeX(false);
                        })));
        
        // frc2::POVButton(&m_operatorController, 0)
        // .WhileTrue(Climb(&m_climber, true).ToPtr());
        // frc2::POVButton(&m_operatorController, 180)
        // .WhileTrue(Climb(&m_climber, false).ToPtr());
            
    frc2::JoystickButton(&m_operatorController, 4)
        .WhileTrue(frc2::CommandPtr(frc2::InstantCommand([this] { m_intake.Outtake(); })))
        .OnFalse(frc2::CommandPtr(frc2::InstantCommand([this] { m_intake.StopIntake(); })));
        
    frc2::JoystickButton(&m_operatorController, 3)
    .OnTrue(frc2::CommandPtr(frc2::InstantCommand([this] { m_wrist.SetAngle(101.0);
                                                            m_intake.Intake(); })))
    .OnFalse(frc2::CommandPtr(
            frc2::InstantCommand([this]
                                    { return m_intake.StopIntake(); })));

    // frc2::POVButton(&m_operatorController, 180)
    //                 .OnTrue(
    //                     frc2::CommandPtr(frc2::InstantCommand([this] {
    //                         m_turret.PresetShooting(true,"middle");
    //                     })))
    //                 .OnFalse(
    //                     frc2::CommandPtr(frc2::InstantCommand([this] {
    //                         m_turret.PresetShooting(false,"middle");
    //                     })));
    
    
    frc::BooleanEvent downPOVBE = frc::BooleanEvent(
        &m_POVloop,
        [&controller = m_operatorController]{
            return (controller.GetPOV()>=135) && (controller.GetPOV()<=225);
        }
    ).Debounce(0.2_s);

    frc2::Trigger POVDownTrig = downPOVBE.CastTo<frc2::Trigger>();
    POVDownTrig.OnTrue(
                    frc2::CommandPtr(frc2::InstantCommand([this] {
                        m_turret.PresetShooting(true,"middle");
                    })))
                .OnFalse(
                    frc2::CommandPtr(frc2::InstantCommand([this] {
                        m_turret.PresetShooting(false,"middle");
                    })));


    frc2::POVButton(&m_operatorController, 90)
                    .OnTrue(
                        frc2::CommandPtr(frc2::InstantCommand([this] {
                            m_turret.PresetShooting(true,"right");
                        })))
                    .OnFalse(
                        frc2::CommandPtr(frc2::InstantCommand([this] {
                            m_turret.PresetShooting(false,"right");
                        })));

    frc2::POVButton(&m_operatorController, 270)
                    .OnTrue(
                        frc2::CommandPtr(frc2::InstantCommand([this] {
                            m_turret.PresetShooting(true,"left");
                        })))
                    .OnFalse(
                        frc2::CommandPtr(frc2::InstantCommand([this] {
                            m_turret.PresetShooting(false,"left");
                        })));
                

    // frc2::POVButton(&m_operatorController, 180)
    //                 .OnTrue(
    //                     frc2::CommandPtr(frc2::InstantCommand([this] {
    //                         return m_turret.ChangeHoodMapValue(-1.0);
    //                     }))
    //                 );

    // frc2::POVButton(&m_operatorController, 90)
    //                 .OnTrue(
    //                     frc2::CommandPtr(frc2::InstantCommand([this] {
    //                         return m_turret.m_turret_shooter.ChangeSpeedMapValue(5);
    //                     }))
    //                 );

    // frc2::POVButton(&m_operatorController, 270)
    //                 .OnTrue(
    //                     frc2::CommandPtr(frc2::InstantCommand([this] {
    //                         return m_turret.m_turret_shooter.ChangeSpeedMapValue(-5);
    //                     }))
    //                 );
                    
    // frc2::JoystickButton(&m_operatorController, 7)
    //     .WhileTrue(frc2::CommandPtr(frc2::RunCommand([this] { m_climber.Zero(); })))
    //     .OnFalse(frc2::CommandPtr(frc2::InstantCommand([this] { m_climber.stopMotor(); })));

    // frc2::Trigger operatorRightTrigger([&m_operatorController]
    // {
    //     if (m_operatorController.GetRawAxis(3) > 0.05) return true;
    //     /* code */   
    //     else return false;
    // });


    
}

void Robot::DisabledPeriodic()
{
    std::string poiName = std::string("POI/") + frc::SmartDashboard::GetString("POIName", "");
    frc::SmartDashboard::PutBoolean("IsPersist", frc::SmartDashboard::IsPersistent(poiName));
}

void Robot::UpdateDashboard()
{
    frc::SmartDashboard::PutNumber("Robot/Battery Amps", batteryShunt.GetVoltage());
    frc::SmartDashboard::PutNumber("Robot/PDH Total Current", m_pdh.GetTotalCurrent());
}

std::string Robot::CheckActiveHub()
{
    std::string GameData;
    std::string AutoWinner;
    GameData = frc::DriverStation::GetGameSpecificMessage();
    if(GameData.length() > 0)
    {
        switch (GameData[0])
        {
            case 'B' :
                AutoWinner = "Blue";
                break;
            case 'R' :
                AutoWinner = "Red";
                break;
            default :
                AutoWinner = "None";
                break;
        }
    } else {
        //code for no data recieved yet
        AutoWinner = "None";
    }

    units::time::second_t matchtimer = frc::DriverStation::GetMatchTime();
    units::time::second_t startOfMatch = units::time::second_t{160};
    units::time::second_t endOfAuto = units::time::second_t{140};
    units::time::second_t endOfTransition = units::time::second_t{130};
    units::time::second_t endOfPeriod1 = units::time::second_t{105};
    units::time::second_t endOfPeriod2 = units::time::second_t{80};
    units::time::second_t endOfPeriod3 = units::time::second_t{55};
    units::time::second_t endOfPeriod4 = units::time::second_t{30};
    units::time::second_t endOfEndgame = units::time::second_t{0};
    frc::DriverStation::Alliance AllianceColor = frc::DriverStation::GetAlliance().value();
    
    //actual one:
    if ((matchtimer < endOfAuto && matchtimer >= endOfTransition) || (matchtimer < endOfPeriod4 && matchtimer >= endOfEndgame))
    {
        if (AllianceColor == frc::DriverStation::Alliance::kBlue)
        {
            return "BlueActive";
        }
        else 
        {
            return "RedActive";
        }
        
    }
    else if(GameData == "R" || GameData == "B")
    {
        if ((matchtimer < endOfTransition && matchtimer >= endOfPeriod1) || (matchtimer < endOfPeriod2 && matchtimer >= endOfPeriod3))
        {
            if (AutoWinner == "Blue")
            {
                return "RedActive";
            }
            else 
            {
                return "BlueActive";
            }
        
        }
        else if ((matchtimer < endOfPeriod1 && matchtimer >= endOfPeriod2) || (matchtimer < endOfPeriod3 && matchtimer >= endOfPeriod4))
        {
            if (AutoWinner == "Blue")
            {
                return "BlueActive";
            }
            else
            {
                return "RedActive";
            }
        
        }
    }
    else
    {
        if (AllianceColor == frc::DriverStation::Alliance::kBlue)
        {
            return "BlueActive";
        }
        else 
        {
            return "RedActive";
        }
        
    } 

}

void Robot::LoadCSVToMap(const std::string& filename) {
    if(!std::filesystem::exists(filename))
    {
        std::ofstream createFile(filename);

        createFile <<
        "1.0,5,70\n"
        "1.5,10,68\n"
        "2.0,50,65\n"
        "2.5,70,60\n"
        "3.0,100,55\n"
        "3.5,105,53\n"
        "4.0,110,50\n"
        "4.5,115,48\n"
        "5.0,120,45\n"
        "5.5,120,43\n"
        "6.0,120,40\n"
        "6.5,120,38\n"
        "7.0,120,35\n";

        createFile.close();
    }
    
    std::ifstream file(filename);

    std::string line;

    std::map<double, double> hoodAngleMap, flyWheelSpeedMap;
    
    // while (std::getline(file, line)) {
    //     std::stringstream ss(line);
    //     std::string distance, flywheelSpeed, hoodAngle;
        
    //     std::getline(ss, distance, ',');
    //     std::getline(ss, flywheelSpeed, ',');
    //     std::getline(ss, hoodAngle, ',');
        
    //     double key = std::stod(distance);
    //     double flyWheelvalue = std::stod(flywheelSpeed);
    //     double hoodValue = std::stod(hoodAngle);
        
    //     flyWheelSpeedMap.insert({key, flyWheelvalue});
    //     hoodAngleMap.insert({key, hoodValue});
    // }
    
    // m_turret.m_turret_shooter.SetCurrentMapState(hoodAngleMap);
    // m_turret.SetCurrentMapState(flyWheelSpeedMap);
    // file.close();
}


#ifndef RUNNING_FRC_TESTS
int main()
{
    return frc::StartRobot<Robot>();
}
#endif
