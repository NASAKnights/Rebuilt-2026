// Copyright (c) FRC Team 122. All Rights Reserved.

#include "Robot.hpp"

#include <exception>

#include <frc/Errors.h>

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
    
    try
    {
        autoChooser = pathplanner::AutoBuilder::buildAutoChooser();
    }
    catch (const std::exception& e)
    {
        FRC_ReportWarning("Failed to load PathPlanner autos: {}", e.what());
    }
    catch (...)
    {
        FRC_ReportWarning("Failed to load PathPlanner autos: unknown error");
    }

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

    if (frc::SmartDashboard::GetBoolean("/Turret/Shooter/Allow Shooting",false)){
        m_pdh.SetSwitchableChannel(true);
    }
    else{
        m_pdh.SetSwitchableChannel(false);
    }
    
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


    frc::Pose3d HoodPose3D = frc::Pose3d(units::meter_t{ 1 * TurretConstants::kHoodXOffset *(std::cos(double(m_turret.GetMeasurement()) - 90))},
                                        units::meter_t{ 1 * TurretConstants::kHoodXOffset *(std::sin(double(m_turret.GetMeasurement()) - 90))},
                                        0.545_m,
                                        frc::Rotation3d(0.0_rad, units::radian_t{((90 - m_turret.GetHoodAngle())*3.14159)/180} , units::radian_t{m_turret.GetMeasurement()}));


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
    if constexpr (frc::RobotBase::IsSimulation())
    {
        m_swerveDrive.ResetPose(frc::Pose2d());
        m_swerveDrive.ResetDriveEncoders();
    }

    m_turret.SaveLaunchMapToFile();
    m_turret.PublishLaunchMap();
    m_led.DefaultAnimation();
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

    // m_swerveDrive.InvertHeading();

    if (m_autonomousCommand)
    {
        m_autonomousCommand->Schedule();
    }
}

void Robot::AutonomousPeriodic() {}

void Robot::AutonomousExit()
{
    m_swerveDrive.InvertHeading();
}

void Robot::TeleopInit()
{
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    // m_wrist.HoldPosition();
    m_led.TeleopInit();
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
    m_led.TeleopPeriodic();
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
    pathplanner::NamedCommands::registerCommand("Intake", Intake(&m_intake, &m_wrist).ToPtr());
    pathplanner::NamedCommands::registerCommand("FlattenMoonKnight", FlattenMoonKnight(&m_turret, &m_wrist).ToPtr());
    pathplanner::NamedCommands::registerCommand("HalfRaiseIntake", HalfRaiseIntake(&m_intake, &m_wrist).ToPtr());
    pathplanner::NamedCommands::registerCommand("Shoot", Shoot(&m_turret, true).ToPtr());
    pathplanner::NamedCommands::registerCommand("NoShoot", Shoot(&m_turret, false).ToPtr());
    pathplanner::NamedCommands::registerCommand("BetterHalfRaise", BetterHalfRaise(&m_intake, &m_wrist).ToPtr());

    m_swerveDrive.SetDefaultCommand(frc2::RunCommand(
        [this]
        {
            auto controllerIn = m_driverController.GetRawButton(4);
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
                    -leftXAxis * 1.0_mps,
                    -leftYAxis * 1.0_mps,
                    -rightXAxis * 2.0_rad_per_s, m_swerveDrive.GetHeading()));
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

    // --------------OPERATOR BUTTONS--------------------------------
        
        frc2::JoystickButton(&m_operatorController, 1)
                .OnTrue(frc2::CommandPtr(
                    frc2::InstantCommand([this]
                                        { m_wrist.SetAngle(90);
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
            
    frc2::JoystickButton(&m_operatorController, 4)
        .WhileTrue(frc2::CommandPtr(frc2::InstantCommand([this] { m_intake.Outtake(); })))
        .OnFalse(frc2::CommandPtr(frc2::InstantCommand([this] { m_intake.StopIntake(); })));
        
    frc2::JoystickButton(&m_operatorController, 3)
    .OnTrue(frc2::CommandPtr(frc2::InstantCommand([this] { m_wrist.SetAngle(101.0);
                                                            m_intake.Intake(); })))
    .OnFalse(frc2::CommandPtr(
            frc2::InstantCommand([this]
                                    { return m_intake.StopIntake(); })));
    
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
    
}

void Robot::DisabledPeriodic()
{
    m_turret.PublishLaunchMap();
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


#ifndef RUNNING_FRC_TESTS
int main()
{
    return frc::StartRobot<Robot>();
}
#endif
