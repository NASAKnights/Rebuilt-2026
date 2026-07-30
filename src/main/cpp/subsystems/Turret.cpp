// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
#include "subsystems/Turret.h"

#include <frc/geometry/Transform3d.h>
#include <frc/geometry/Translation2d.h>
#include <cmath>
#include <networktables/NetworkTableInstance.h>
#include <algorithm>

namespace
{
    units::degree_t GetTurretAngleCorrection(units::degree_t turretAngle, units::degree_t amplitude)
    {
        // Peak correction at 180 deg, zero at 90/270 deg.
        return amplitude * std::sin(units::radian_t{turretAngle - 90_deg}.value());
    }

    units::turns_per_second_t GetShooterSpeedCorrection(units::degree_t turretAngle, units::turns_per_second_t amplitude)
    {
        // +peak at 90 deg (topspin), -peak at 270 deg (backspin).
        return amplitude * std::sin(units::radian_t{turretAngle}.value());
    }

    units::degree_t GetRobotVelocityTurretAngleCorrection(frc::Pose2d robotPose, units::degree_t KVTurretAngleCompensation, units::meter_t target_distance)
    {

       

        return KVTurretAngleCompensation;
    }

}

// using State = frc::TrapezoidProfile<units::degrees>::State;
using degrees_per_second_squared_t =
    units::unit_t<units::compound_unit<units::angular_velocity::degrees_per_second,
                                       units::inverse<units::time::seconds>>>;

Turret::Turret() : m_controller(
                       TurretConstants::kAngleP, TurretConstants::kAngleI, TurretConstants::kAngleD),
                   // m_motor(TurretConstants::kAngleMotorId, rev::spark::SparkLowLevel::MotorType::kBrushless),
                   m_encoder(m_motor.GetEncoder()),
                   m_feedforward(TurretConstants::kFFks, TurretConstants::kFFkg, TurretConstants::kFFkV, TurretConstants::kFFkA),

                   m_TurretSim(TurretConstants::kSimMotor, TurretConstants::kGearRatio, TurretConstants::kmoi,
                               TurretConstants::kTurretRadius, TurretConstants::kminAngle, TurretConstants::kmaxAngle,
                               TurretConstants::kGravity, TurretConstants::kTurretStartAngle, TurretConstants::kSimNoise)
{
    // m_motor.SetInverted(true);
    m_motor.SetInverted(true);
    rev::spark::SparkBaseConfig config;
    config.SetIdleMode(rev::spark::SparkBaseConfig::IdleMode::kCoast);
    config.encoder.PositionConversionFactor(TurretConstants::turretPositionConversionFactor);
    config.encoder.VelocityConversionFactor(TurretConstants::turretVelocityConversionFactor);
    config.SmartCurrentLimit(30, 0, 20000);

    m_hood.SetBounds(units::microsecond_t{2000}, units::microsecond_t{1550}, units::microsecond_t{1500}, units::microsecond_t{1450}, units::microsecond_t{1000});
    m_hood2.SetBounds(units::microsecond_t{2000}, units::microsecond_t{1550}, units::microsecond_t{1500}, units::microsecond_t{1450}, units::microsecond_t{1000});

    m_controller.SetIZone(TurretConstants::kIZone);
    m_controller.SetTolerance(TurretConstants::kTolerancePos.value(), TurretConstants::kToleranceVel.value());
    // Start m_Turret in neutral position
    // m_TurretState = TurretConstants::TRACKING;
    m_TurretState = TurretConstants::HOMING; // Setting initial state to HOMING first, should transition into tracking automagically
    wpi::log::DataLog &log = frc::DataLogManager::GetLog();
    m_AngleLog = wpi::log::DoubleLogEntry(log, "/Turret/Angle");
    m_SetPointLog = wpi::log::DoubleLogEntry(log, "/Turret/Setpoint");
    m_StateLog = wpi::log::IntegerLogEntry(log, "/Turret/State");
    m_MotorCurrentLog = wpi::log::DoubleLogEntry(log, "/Turret/MotorCurrent");
    m_MotorVoltageLog = wpi::log::DoubleLogEntry(log, "/Turret/MotorVoltage");
    m_PoseStaleLog = wpi::log::BooleanLogEntry(log, "/Turret/PoseStale");

    frc::SmartDashboard::PutBoolean("/Turret/Shooter/Allow Shooting", false);

    frc::SmartDashboard::PutBoolean("/Turret/Hood/Angle Manual Override", false);
    frc::SmartDashboard::PutNumber("/Turret/Hood/Angle Manual Set", 0.0);
    frc::SmartDashboard::SetDefaultNumber("/Turret/Comp/TurretAngleAmpDeg", 4.0);
    frc::SmartDashboard::SetDefaultNumber("/Turret/Comp/ShooterSpeedAmpRPS", 0.0);
    frc::SmartDashboard::SetPersistent("/Turret/Comp/TurretAngleAmpDeg");
    frc::SmartDashboard::SetPersistent("/Turret/Comp/ShooterSpeedAmpRPS");
    frc::SmartDashboard::PutNumber(
        "/Turret/Comp/TurretAngleAmpDeg",
        frc::SmartDashboard::GetNumber("/Turret/Comp/TurretAngleAmpDeg", 4.0));
    frc::SmartDashboard::PutNumber(
        "/Turret/Comp/ShooterSpeedAmpRPS",
        frc::SmartDashboard::GetNumber("/Turret/Comp/ShooterSpeedAmpRPS", 0.0));
    networkTableInst = nt::NetworkTableInstance::GetDefault();
    auto poseTable = networkTableInst.GetTable("ROS2Bridge");
    baseLinkSubscriber = poseTable->GetDoubleArrayTopic(robotPoseLink).Subscribe({}, {.periodic = 0.02, .sendAll = true});

    // Initialize goal topic - publish default and subscribe for updates
    auto turretTable = networkTableInst.GetTable("Turret");
    goalPublisher = turretTable->GetDoubleArrayTopic("goal").Publish({.periodic = 0.01, .sendAll = true});
    std::vector<double> defaultGoal = {4.5, 4.0, 1.829};
    goalSubscriber = turretTable->GetDoubleArrayTopic("goal").Subscribe(defaultGoal, {.periodic = 0.02, .sendAll = true});
    // Publish initial default goal
    goalPublisher.Set(defaultGoal);

    m_turretObject = m_turretField.GetObject("Turret");
    frc::SmartDashboard::PutData("Turret Field", &m_turretField);
    m_motor.Configure(config, rev::spark::SparkMax::ResetMode::kResetSafeParameters, rev::spark::SparkMax::PersistMode::kPersistParameters);
    m_encoder.SetPosition(0.0);
    SetAngle(TurretConstants::kmidPoint);

    if constexpr (frc::RobotBase::IsSimulation())
    {
        m_magSwitchSim.SetValue(true);
    }

}

void Turret::SimulationPeriodic()
{
    m_TurretSim.Update(10_ms);
    frc::SmartDashboard::PutNumber("/Turret/Aim/Motor Current Draw", m_TurretSim.GetCurrentDraw().value());
    units::radian_t epsilon = 0.001_rad;
    m_magSwitchSim.SetValue(m_TurretSim.GetAngle() > TurretConstants::kminAngle + epsilon && m_TurretSim.GetAngle() < TurretConstants::kmaxAngle - epsilon);
}

void Turret::AllowShooting()
{
    allowShooting = true;
    frc::SmartDashboard::PutBoolean("/Turret/Shooter/Allow Shooting", true);
}

void Turret::PauseShooting()
{
    allowShooting = false;
    frc::SmartDashboard::PutBoolean("/Turret/Shooter/Allow Shooting", false);
}

units::degree_t Turret::GetMeasurement()
{ // original get measurement function
    if constexpr (frc::RobotBase::IsSimulation())
    {
        return m_TurretSim.GetAngle();
    }

    return units::degree_t{(m_encoder.GetPosition())};
}

std::pair<units::degree_t, units::degrees_per_second_t> Turret::findTrackingAngle()
{
    // Use GetAtomic to get both value and timestamp
    auto poseResult = baseLinkSubscriber.GetAtomic();
    std::vector<double> baseLinkPose = poseResult.value;
    int64_t poseTimestamp = poseResult.time;

    // Check if we have valid pose data
    auto baseLink = DoubleArrayToPose2d(baseLinkPose);
    if (!baseLink.has_value())
    {
        frc::SmartDashboard::PutString("/Turret/Pose/Pose Status", "No Pose Data");
        frc::SmartDashboard::PutNumber("/Turret/Pose/Pose Data Size", baseLinkPose.size());
        m_PoseStaleLog.Append(true);
        return {GetMeasurement(), 0_deg_per_s};
    }

    // Check for stale data (timestamp hasn't changed)
    bool poseIsStale = false;
    if (m_lastPoseUpdateTime != 0 && poseTimestamp - m_lastPoseUpdateTime > 1e5)
    {
        // Timestamp hasn't changed - pose is stale
        poseIsStale = true;
        frc::SmartDashboard::PutString("/Turret/Pose/Pose Status", "STALE - Timestamp Frozen");
        // frc::SmartDashboard::PutNumber("/Turrent/Pose/Diff Time", poseTimestamp - m_lastPoseUpdateTime);
    }
    else if (m_lastValidPose.has_value())
    {
        // Check if pose values are identical (another indicator of staleness)
        auto lastPose = m_lastValidPose.value();
        double poseDiff = std::sqrt(
            std::pow((baseLink->X() - lastPose.X()).value(), 2) +
            std::pow((baseLink->Y() - lastPose.Y()).value(), 2));
        double angleDiff = std::abs((baseLink->Rotation().Radians() - lastPose.Rotation().Radians()).value());

        UpdateTurretGoal(*baseLink); // UPDATES THE CURRENT GOAL POSITION

        // If robot hasn't moved at all in multiple cycles, might be stale
        // (though it could also just be stationary)
        if (poseDiff < 0.001 && angleDiff < 0.001)
        {
            frc::SmartDashboard::PutString("/Turret/Pose/Pose Status", "WARNING - Pose Unchanged");
        }
        else
        {
            frc::SmartDashboard::PutString("/Turret/Pose/Pose Status", "OK");
        }
    }
    else
    {
        frc::SmartDashboard::PutString("/Turret/Pose/Pose Status", "First Update");
    }

    // Log staleness state
    m_PoseStaleLog.Append(poseIsStale);
    frc::SmartDashboard::PutBoolean("/Turret/Pose/Pose Stale", poseIsStale);
    frc::SmartDashboard::PutNumber("/Turret/Pose/Pose Timestamp", poseTimestamp / 1e6);         // Convert to seconds
    frc::SmartDashboard::PutNumber("/Turret/Pose/Pose Age", (nt::Now() - poseTimestamp) / 1e6); // Age in seconds

    // Update tracking variables
    m_lastPoseUpdateTime = poseTimestamp;
    m_lastValidPose = *baseLink;

    // If pose is stale, hold current position
    if (poseIsStale)
    {
        return {GetMeasurement(), 0_deg_per_s};
    }

    // frc::Transform3d world2robot = frc::Transform3d(baseLink->X(), baseLink->Y(), 0_m, frc::Rotation3d(0_rad, 0_rad, baseLink->Rotation().Radians()));

    // Read goal from NetworkTables
    std::vector<double> defaultGoal = {4.0, 4.0, 0.0};
    std::vector<double> goalArray = goalSubscriber.Get(defaultGoal);
    if (goalArray.size() >= 3)
    {
        goal = frc::Transform3d(
            units::meter_t{goalArray[0]},
            units::meter_t{goalArray[1]},
            units::meter_t{goalArray[2]},
            frc::Rotation3d());
    }

    // calculate the solution for the current robot position and orientation
    units::meters_per_second_t launch_speed;
    units::radian_t launch_angle;
    units::radian_t turret_angle;

    CalculateTargetingSolution(m_lastValidPose.value(), units::second_t{0}, true,
                               launch_speed, launch_angle, turret_angle);

    // calculate the solution for the projected robot position and orientation
    // at a small time step

    units::second_t dt = units::second_t{0.02}; // 50 ms in the future
    units::meters_per_second_t future_launch_speed;
    units::radian_t future_launch_angle;
    units::radian_t future_turret_angle;

    CalculateTargetingSolution(m_lastValidPose.value(), dt, false,
                               future_launch_speed, future_launch_angle, future_turret_angle);

    // compute the rate of change for each parameter
    m_LaunchSpeedAcceleration = (future_launch_speed - launch_speed) / dt;
    m_LaunchAngleVelocity = (future_launch_angle - launch_angle) / dt;
    m_TurretAngleVelocity = (future_turret_angle - turret_angle) / dt;

    // Find smallest signed error
    units::radian_t error = frc::AngleModulus(turret_angle - GetMeasurement());

    // If the computed target exceeds the upper limit by >180°, it likely wrapped
    if (turret_angle > TurretConstants::ksoftMaxAngle)
    {
        // If we're only just beyond by less than 180°, clamp
        if (turret_angle - 360_deg >= TurretConstants::ksoftMinAngle)
            turret_angle -= 360_deg;
        else
            turret_angle = TurretConstants::ksoftMaxAngle;
    }
    else if (turret_angle < TurretConstants::ksoftMinAngle)
    {
        if (turret_angle + 360_deg <= TurretConstants::ksoftMaxAngle)
            turret_angle += 360_deg;
        else
            turret_angle = TurretConstants::ksoftMinAngle;
    }
    frc::SmartDashboard::PutNumber("/Turret/Aim/Turret Angle Error Deg", units::degree_t{error}.value());

    frc::SmartDashboard::PutNumber("/Turret/Shooter/Launch Speed Acceleration MPS", m_LaunchSpeedAcceleration.value());
    frc::SmartDashboard::PutNumber("/Turret/Shooter/Launch Angle Velocity DPS", units::degrees_per_second_t{m_LaunchAngleVelocity}.value());
    frc::SmartDashboard::PutNumber("/Turret/Aim/Turret Angle Velocity DPS", units::degrees_per_second_t{m_TurretAngleVelocity}.value());

    return {turret_angle, units::degrees_per_second_t{m_TurretAngleVelocity}};
}

void Turret::SetAngle(units::degree_t TurretAngleGoal, units::degrees_per_second_t velocityGoal)
{
    if (!(TurretAngleGoal.value() < m_goal.value() + TurretConstants::kTolerancePos.value() && TurretAngleGoal.value() > m_goal.value() - TurretConstants::kTolerancePos.value()))
    {
        // units::degrees_per_second_t robotVel = units::degrees_per_second_t{frc::SmartDashboard::GetNumber("/Turret/Aim/Angular Velocity", 0.0)};
        auto velocity = GetVelocity();
        m_goal = units::angle::degree_t(TurretAngleGoal);
        if (abs(velocity.value()) < (1_deg_per_s).value())
        {
            velocity = 1_deg_per_s * copysign(1.0, velocity.value());
        }
        m_velocityGoal = velocityGoal;
        m_controller.SetSetpoint(m_goal.value());
    }
    frc::SmartDashboard::PutNumber("/Turret/Aim/m_goal", double(m_goal));
}

void Turret::FindLimitSwitch()
{
    m_TurretState = TurretConstants::HOMING;
}

units::degrees_per_second_t Turret::GetVelocity()
{
    return units::degrees_per_second_t{m_encoder.GetVelocity()};
}

void Turret::SetHood(double extension)
{
    m_hood.Set(extension);
    m_hood2.Set(extension);
}

double Turret::getTOF(double distance)
{
    return m_launchCalculator.GetTOF(distance);
}

units::meter_t Turret::getDistanceFromTOF(double TOF)
{
    return m_launchCalculator.GetDistanceFromTOF(TOF);
}

void Turret::ChangeHoodAngle(units::meter_t distance)
{
    double ballLaunchAngleDegrees = m_launchCalculator.GetHoodAngleDegrees(distance.value());
    frc::SmartDashboard::PutNumber("/Turret/Hood/Launch Angle", ballLaunchAngleDegrees);

    double servoExtention = (-(2.94699 * std::pow(10, -7)) * std::pow(ballLaunchAngleDegrees, 4) +
                             (5.89093 * std::pow(10, -5)) * std::pow(ballLaunchAngleDegrees, 3) -
                             (4.41946 * std::pow(10, -3)) * std::pow(ballLaunchAngleDegrees, 2) +
                             (0.124056) * ballLaunchAngleDegrees - 0.227319);

    frc::SmartDashboard::PutNumber("/Turret/Hood/Servo Extension", servoExtention);
    if (servoExtention > 0.8)
    {
        servoExtention = 0.8;
    }
    else if (servoExtention < 0.08)
    {
        servoExtention = 0.08;
    }
    m_hood.Set(servoExtention);
    m_hood2.Set(servoExtention);
}

double Turret::GetHoodAngle()
{
    if constexpr (frc::RobotBase::IsSimulation())
    {
        return frc::SmartDashboard::GetNumber("/Turret/Hood/Launch Angle", 0.0);
    }
    return 0.0;
}

void Turret::ChangeHoodAngle(double ballLaunchAngleDegrees)
{

    double servoExtention = (-(2.94699 * std::pow(10, -7)) * std::pow(ballLaunchAngleDegrees, 4) +
                             (5.89093 * std::pow(10, -5)) * std::pow(ballLaunchAngleDegrees, 3) -
                             (4.41946 * std::pow(10, -3)) * std::pow(ballLaunchAngleDegrees, 2) +
                             (0.124056) * ballLaunchAngleDegrees - 0.227319);

    frc::SmartDashboard::PutNumber("/Turret/Hood/Servo Extension", servoExtention);
    if (servoExtention > 0.8)
    {
        servoExtention = 0.8;
    }
    else if (servoExtention < 0.05)
    {
        servoExtention = 0.05;
    }
    frc::SmartDashboard::PutNumber("/Turret/Hood/Launch Angle", ballLaunchAngleDegrees);
    m_hood.Set(servoExtention);
    m_hood2.Set(servoExtention);
}

void Turret::ChangeHoodAngle(units::angle::radian_t ballLaunchAngle, units::meter_t distance)
{
    double hoodOffset = 0; // no offset
    double distVal = distance.value();

    if (!kHoodOffsetMap.empty())
    {
        auto itHigh = kHoodOffsetMap.lower_bound(distVal);

        if (itHigh == kHoodOffsetMap.begin())
        {
            // Distance is smaller than our first entry
            hoodOffset = itHigh->second;
        }
        else if (itHigh == kHoodOffsetMap.end())
        {
            // Distance is larger than our last entry
            hoodOffset = std::prev(itHigh)->second;
        }
        else
        {
            // Interpolate between prev and itHigh
            auto itLow = std::prev(itHigh);
            double d1 = itLow->first;
            double g1 = itLow->second;
            double d2 = itHigh->first;
            double g2 = itHigh->second;

            double t = (distVal - d1) / (d2 - d1);
            hoodOffset = g1 + t * (g2 - g1);
        }
    }

    ballLaunchAngle -= units::degree_t{hoodOffset};

    double ballLaunchAngleDegrees = double((ballLaunchAngle * 180) / TurretConstants::kPI);

    double servoExtention = (-(2.94699 * std::pow(10, -7)) * std::pow(ballLaunchAngleDegrees, 4) +
                             (5.89093 * std::pow(10, -5)) * std::pow(ballLaunchAngleDegrees, 3) -
                             (4.41946 * std::pow(10, -3)) * std::pow(ballLaunchAngleDegrees, 2) +
                             (0.124056) * ballLaunchAngleDegrees - 0.227319);

    frc::SmartDashboard::PutNumber("/Turret/Hood/Servo Extension", servoExtention);
    if (servoExtention > 0.8)
    {
        servoExtention = 0.8;
    }
    else if (servoExtention < 0.05)
    {
        servoExtention = 0.05;
    }
    frc::SmartDashboard::PutNumber("/Turret/Hood/Launch Angle", ballLaunchAngleDegrees);
    m_hood.Set(servoExtention);
    m_hood2.Set(servoExtention);
}

void Turret::ChangeHoodMapValue(double newOffsetValue)
{
    double distVal = frc::SmartDashboard::GetNumber("/Turret/Ballistics/Target Distance", 0);



    m_launchCalculator.AdjustHoodAngleAtDistance(distVal, newOffsetValue);
}

void Turret::ChangeLaunchSpeed(units::meters_per_second_t speed, units::meter_t distance)
{
    m_turret_shooter.SetSpeed(speed, distance);
}

void Turret::Periodic()
{
    frc::SmartDashboard::PutNumber(
        "/Turret/Comp/TurretAngleAmpDeg",
        frc::SmartDashboard::GetNumber("/Turret/Comp/TurretAngleAmpDeg", 4.0));
    frc::SmartDashboard::PutNumber(
        "/Turret/Comp/ShooterSpeedAmpRPS",
        frc::SmartDashboard::GetNumber("/Turret/Comp/ShooterSpeedAmpRPS", 0.0));

    printLog();
    UpdateFieldVisuals();
    double fb;
    units::volt_t ff;
    units::volt_t v;
    frc::SmartDashboard::PutBoolean("Turret/Aim/Limit Switch", m_magSwitch.Get());

    switch (m_TurretState)
    {
    case TurretConstants::HOLD:
    {
        //     m_TurretState = TurretConstants::TRACKING;
        // }
        frc::SmartDashboard::PutString("/Turret/State", "HOLD");
        fb = m_controller.Calculate(GetMeasurement().value());
        ff = m_feedforward.Calculate(units::degree_t{m_controller.GetSetpoint()}, 0_deg_per_s);
        v = units::volt_t{fb} + ff;
        break;
    }
    case TurretConstants::DISABLED:
    {
        frc::SmartDashboard::PutString("/Turret/State", "DISABLED");
        v = units::voltage::volt_t(0.0);
        break;
    }
    case TurretConstants::TRACKING:
    {
        frc::SmartDashboard::PutString("/Turret/State", "TRACKING");

        // Tracking Angle
        auto [angle, velocity] = findTrackingAngle();
        const units::degree_t angleCompAmp{frc::SmartDashboard::GetNumber("/Turret/Comp/TurretAngleAmpDeg", 4.0)};
        const units::degree_t angleComp = GetTurretAngleCorrection(angle, angleCompAmp);
        const units::degree_t correctedAngle = angle + angleComp;
        frc::SmartDashboard::PutNumber("/Turret/Aim/Measurement Value", GetMeasurement().value());
        if (presetShooting)
        {
            if (presetType == "middle")
            {
                SetAngle(units::angle::degree_t{manualShootingPresetMid()[2]}, velocity);
            }
            else if (presetType == "left")
            {
                SetAngle(units::angle::degree_t{manualShootingPresetLeft()[2]}, velocity);
            }
            else if (presetType == "right")
            {
                SetAngle(units::angle::degree_t{manualShootingPresetRight()[2]}, velocity);
            }
            // SetAngle(units::angle::degree_t{manualShootingPreset1()[2]}, velocity);
        }
        else
        {

            SetAngle(correctedAngle, velocity);
        }
        fb = m_controller.Calculate(GetMeasurement().value());
        ff = m_feedforward.Calculate(correctedAngle, velocity);
        v = units::volt_t{fb} + ff;
        frc::SmartDashboard::PutNumber("/Turret/Aim/Velocity", velocity.value());
        frc::SmartDashboard::PutNumber("/Turret/Aim/Angle", angle.value());
        frc::SmartDashboard::PutNumber("/Turret/Comp/TurretAngleCorrDeg", angleComp.value());
        frc::SmartDashboard::PutNumber("/Turret/Comp/TurretAngleCorrectedDeg", correctedAngle.value());
        frc::SmartDashboard::PutNumber("/Turret/Aim/Feedforward", ff.value());

        if (GetMeasurement() < TurretConstants::ksoftMinAngle && v.value() < 0)
        {
            frc::SmartDashboard::PutString("/Turret/Aim/Stopped", "soft_min");
            v = units::volt_t(0);
        }
        else if (GetMeasurement() > TurretConstants::ksoftMaxAngle && v.value() > 0)
        {
            frc::SmartDashboard::PutString("/Turret/Aim/Stopped", "soft_max");
            v = units::volt_t(0);
        }
       
        else
        {
            frc::SmartDashboard::PutString("/Turret/Aim/Stopped", "not");
        }

       
        frc::SmartDashboard::PutNumber("/Turret/Aim/Feedback", double(fb));
        frc::SmartDashboard::PutNumber("/Turret/Aim/Voltage", double(v));

        // Baseline vs lookahead-compensated launch properties for visualization.
        const units::meter_t originalDistance = m_BallisticDistance;
        const double originalTofSec = getTOF(originalDistance.value());
        const double originalHoodDeg = m_launchCalculator.GetHoodAngleDegrees(originalDistance.value());
        const double originalFlywheelRps = m_launchCalculator.GetFlywheelSpeed(originalDistance).value();
        bool hasUpdatedLaunchSolution = false;
        LaunchCalculator::LaunchingParameters updatedLaunchParams{
            false, frc::Rotation2d{}, 0.0, 0.0, 0.0, 0.0, 0.0_m, 0.0};

        if (allowShooting && !presetShooting)
        {
            const auto poseResult = baseLinkSubscriber.GetAtomic();
            const auto baseLink = DoubleArrayToPose2d(poseResult.value);
            if (baseLink.has_value())
            {
                const units::meters_per_second_t robotVx{frc::SmartDashboard::GetNumber("drive/vx", 0.0)};
                const units::meters_per_second_t robotVy{frc::SmartDashboard::GetNumber("drive/vy", 0.0)};
                const units::radians_per_second_t robotOmega{frc::SmartDashboard::GetNumber("drive/omega", 0.0)};
                updatedLaunchParams = m_launchCalculator.CalculateParameters(
                    *baseLink,
                    frc::Translation2d{goal.X(), goal.Y()},
                    robotVx,
                    robotVy,
                    robotOmega);
                hasUpdatedLaunchSolution = true;
            }
        }

        frc::SmartDashboard::PutBoolean("/Turret/LaunchCalc/HasUpdatedSolution", hasUpdatedLaunchSolution);
        frc::SmartDashboard::PutBoolean("/Turret/LaunchCalc/UpdatedIsValid", updatedLaunchParams.isValid);
        frc::SmartDashboard::PutNumber("/Turret/LaunchCalc/Original/DistanceM", originalDistance.value());
        frc::SmartDashboard::PutNumber("/Turret/LaunchCalc/Original/TofSec", originalTofSec);
        frc::SmartDashboard::PutNumber("/Turret/LaunchCalc/Original/HoodDeg", originalHoodDeg);
        frc::SmartDashboard::PutNumber("/Turret/LaunchCalc/Original/FlywheelRps", originalFlywheelRps);
        frc::SmartDashboard::PutNumber("/Turret/LaunchCalc/Updated/DistanceM", updatedLaunchParams.lookaheadDistance.value());
        frc::SmartDashboard::PutNumber("/Turret/LaunchCalc/Updated/TofSec", updatedLaunchParams.timeOfFlightSec);
        frc::SmartDashboard::PutNumber("/Turret/LaunchCalc/Updated/HoodDeg", updatedLaunchParams.hoodAngleDeg);
        frc::SmartDashboard::PutNumber("/Turret/LaunchCalc/Updated/FlywheelRps", updatedLaunchParams.flywheelRps);
        frc::SmartDashboard::PutNumber(
            "/Turret/LaunchCalc/Delta/DistanceM",
            updatedLaunchParams.lookaheadDistance.value() - originalDistance.value());
        frc::SmartDashboard::PutNumber(
            "/Turret/LaunchCalc/Delta/TofSec",
            updatedLaunchParams.timeOfFlightSec - originalTofSec);
        frc::SmartDashboard::PutNumber(
            "/Turret/LaunchCalc/Delta/HoodDeg",
            updatedLaunchParams.hoodAngleDeg - originalHoodDeg);
        frc::SmartDashboard::PutNumber(
            "/Turret/LaunchCalc/Delta/FlywheelRps",
            updatedLaunchParams.flywheelRps - originalFlywheelRps);

        // Hood/Launch Angle
        if (Flatten)
        {
            SetHood(0.3);
            frc::SmartDashboard::PutNumber("/Turret/Hood/Launch Angle", 92.44086);
        }
        else if (allowShooting)
        {
            if (presetShooting)
            {
                if (presetType == "middle")
                {
                    ChangeHoodAngle(manualShootingPresetMid()[1]);
                }
                else if (presetType == "left")
                {
                    ChangeHoodAngle(manualShootingPresetLeft()[1]);
                }
                else if (presetType == "right")
                {
                    ChangeHoodAngle(manualShootingPresetRight()[1]);
                }
            }
            else
            {
                if (hasUpdatedLaunchSolution && updatedLaunchParams.isValid)
                {
                    ChangeHoodAngle(updatedLaunchParams.lookaheadDistance);
                }
                else
                {
                    ChangeHoodAngle(originalDistance);
                }
            }
        }
        else
        {
            SetHood(0.3);
            frc::SmartDashboard::PutNumber("/Turret/Hood/Launch Angle", 92.44086);
        }

        bool override = frc::SmartDashboard::GetBoolean("/Turret/Hood/Angle Manual Override", false);
        if (override)
        {
            // ChangeHoodAngle(frc::SmartDashboard::GetNumber("/Turret/Hood/Angle Manual Set", 0.0));
        }
        // Launch Speed
        if (allowShooting)
        {
            // ChangeLaunchSpeed(m_BallisticLaunchSpeed, m_BallisticDistance);
            // units::turns_per_second_t commandMotorSpeed =
            frc::SmartDashboard::PutNumber("/Turret/Shooter/Set Speed MPS", m_BallisticLaunchSpeed.value());
            // units::turns_per_second_t commandedMotorSpeed = m_turret_shooter.ConvertBallSpeed2Motor(m_BallisticLaunchSpeed,m_BallisticDistance);
            units::turns_per_second_t commandedMotorSpeed = 0_tps;
            if (presetShooting)
            {
                if (presetType == "middle")
                {
                    commandedMotorSpeed = units::turns_per_second_t{manualShootingPresetMid()[0]};
                }
                else if (presetType == "left")
                {
                    commandedMotorSpeed = units::turns_per_second_t{manualShootingPresetLeft()[0]};
                }
                else if (presetType == "right")
                {
                    commandedMotorSpeed = units::turns_per_second_t{manualShootingPresetRight()[0]};
                }
                // commandedMotorSpeed = units::turns_per_second_t{manualShootingPreset1()[0]};
            }
            else
            {
                if (hasUpdatedLaunchSolution && updatedLaunchParams.isValid)
                {
                    commandedMotorSpeed = units::turns_per_second_t{updatedLaunchParams.flywheelRps};
                }
                else
                {
                    commandedMotorSpeed = units::turns_per_second_t{originalFlywheelRps};
                }
            }
            const units::turns_per_second_t speedCompAmp{frc::SmartDashboard::GetNumber("/Turret/Comp/ShooterSpeedAmpRPS", 0.0)};
            const units::turns_per_second_t speedComp = GetShooterSpeedCorrection(m_goal, speedCompAmp);
            commandedMotorSpeed += speedComp;
            frc::SmartDashboard::PutNumber("/Turret/Comp/ShooterSpeedCorrRPS", speedComp.value());
            frc::SmartDashboard::PutNumber("/Turret/Comp/ShooterSpeedCorrectedRPS", commandedMotorSpeed.value());
            m_turret_shooter.SetMotorSpeed(commandedMotorSpeed);
            if (m_turret_shooter.GetActualMotorSpeed() >= commandedMotorSpeed)
            {
                m_turret_shooter.RunSpindexerIndexer(m_BallisticDistance);
            }
        }
        else if (!allowShooting)
        {
            // ChangeLaunchSpeed(units::meters_per_second_t{0.0}, 0.0_m);
            m_turret_shooter.TurnOffMotors();
            m_turret_shooter.StopSpindexerIndexer();
        }
        break;
    }
    case TurretConstants::HOMING:
    {
        frc::SmartDashboard::PutString("/Turret/State", "HOMING");
        v = units::voltage::volt_t(-1.5); // TODO: Set a proper value in the constants for constant slow movement in HOMING
        if (!m_magSwitch.Get())
        {
            m_encoder.SetPosition(units::angle::degree_t{TurretConstants::kminAngle}.value());
            // m_TurretState = TurretConstants::HOLD;
            // m_goal = units::angle::degree_t(0);
            // v = units::voltage::volt_t(0.0);
            m_TurretState = TurretConstants::TRACKING;
        }
        break;
    }
    default:
    {
        frc::SmartDashboard::PutString("/Turret/State", "default");
        break;
    }
    }
    v = std::clamp(v, -TurretConstants::kMaxVoltage, TurretConstants::kMaxVoltage);
    if constexpr (frc::RobotBase::IsSimulation())
    {
        m_TurretSim.SetInputVoltage(v);
        SimulationPeriodic();
    }
    frc::SmartDashboard::PutNumber("/Turret/Aim/Voltage", double(v));
    m_motor.SetVoltage(v);

    std::string GameData;
    GameData = frc::DriverStation::GetGameSpecificMessage();
    if (GameData.length() > 0)
    {
        switch (GameData[0])
        {
        case 'B':
            // blue case code
            break;

        case 'R':
            // red case code
            break;
        default:
            // this is corrupt data
            break;
        }
    }
    else
    {
        // code for no data recieved yet
    }
}

TurretConstants::TurretState Turret::GetState()
{
    return m_TurretState;
}

void Turret::printLog()
{
    frc::SmartDashboard::PutNumber("/Turret/Aim/Actual Angle", GetMeasurement().value());
    frc::SmartDashboard::PutNumber("/Turret/Aim/Goal Angle", m_controller.GetSetpoint());
    frc::SmartDashboard::PutNumber("/Turret/Aim/Setpoint",
                                   m_controller.GetSetpoint());
    frc::SmartDashboard::PutNumber("/Turret/Aim/Velocity", double(GetVelocity()));
    m_AngleLog.Append(GetMeasurement().value());
    m_SetPointLog.Append(m_controller.GetSetpoint());
    m_StateLog.Append(m_TurretState);
    m_MotorCurrentLog.Append(m_motor.GetOutputCurrent());
    m_MotorVoltageLog.Append(m_motor.GetAppliedOutput());

    // Turret Shooter values output in shooter class
}

void Turret::Disable()
{
    m_motor.StopMotor();
}

void Turret::HoldPosition()
{
    // if (m_TurretState != TurretConstants::TurretState::HOLD)
    {
        m_controller.Reset();
        m_controller.SetSetpoint(GetMeasurement().value());
        m_goal = GetMeasurement();
        m_velocityGoal = 0_deg_per_s;
        m_TurretState = TurretConstants::TurretState::HOLD;
    }
}

std::map<double, double> Turret::GetCurrentMapState()
{
    return m_launchCalculator.GetHoodAngleMap();
}

std::vector<double> Turret::manualShootingPresetMid()
{
    return std::vector<double>{25.5, 58.5, 180.0};
}

std::vector<double> Turret::manualShootingPresetLeft()
{
    return std::vector<double>{40.0, 55.0, 272.0};
}

std::vector<double> Turret::manualShootingPresetRight()
{
    return std::vector<double>{40.0, 52.0, 90.0};
}

void Turret::PresetShooting(bool temp, std::string preset)
{
    if (temp)
    {
        presetShooting = true;
    }
    else
    {
        presetShooting = false;
    }
    presetType = preset; // Sets which preset we use
}

void Turret::SetCurrentMapState(std::map<double, double> inputCurrentState)
{
    m_launchCalculator.SetHoodAngleMap(inputCurrentState);
}

void Turret::UpdateFieldVisuals()
{
    m_turretField.GetObject("Goal")->SetPose(frc::Pose3d(goal.ToMatrix()).ToPose2d());
    if (m_turretObject == nullptr)
    {
        return;
    }

    auto baseLinkPose = DoubleArrayToPose2d(baseLinkSubscriber.Get({}));
    if (baseLinkPose.has_value())
    {
        m_turretField.SetRobotPose(*baseLinkPose);
        auto turretPose = CalculateTurretPose(*baseLinkPose);
        m_turretObject->SetPose(turretPose);
        m_lastRobotPose = *baseLinkPose;
        m_lastTurretPose = turretPose;
        return;
    }

    if (m_lastRobotPose.has_value())
    {
        m_turretField.SetRobotPose(*m_lastRobotPose);
    }

    if (m_lastTurretPose.has_value())
    {
        m_turretObject->SetPose(*m_lastTurretPose);
    }
    else
    {
        // Default to robot origin so the turret object stays visible even before NT data arrives.
        auto defaultPose = CalculateTurretPose(frc::Pose2d{});
        m_turretObject->SetPose(defaultPose);
        m_lastTurretPose = defaultPose;
    }
}

void Turret::UpdateTurretGoal(const frc::Pose2d &robotPose)
{
    frc::Pose2d turretPose = CalculateTurretPose(robotPose);
    units::length::meter_t TurretX = turretPose.X();
    units::length::meter_t TurretY = turretPose.Y();
    frc::DriverStation::Alliance AllianceColor;

    if constexpr (frc::RobotBase::IsSimulation())
    {
        AllianceColor = frc::DriverStation::Alliance::kBlue;
    }
    else
    {
        AllianceColor = frc::DriverStation::GetAlliance().value();
    }

    if (TurretX < TurretConstants::BlueAllianceZoneX && AllianceColor == frc::DriverStation::Alliance::kBlue)
    {
        TurretGoal = TurretConstants::BlueHubCoords;
    }
    else if (TurretX >= TurretConstants::BlueAllianceZoneX && TurretY >= TurretConstants::MidFieldLine && AllianceColor == frc::DriverStation::Alliance::kBlue)
    {
        TurretGoal = TurretConstants::TopBlueCoords;
    }
    else if (TurretX >= TurretConstants::BlueAllianceZoneX && TurretY < TurretConstants::MidFieldLine && AllianceColor == frc::DriverStation::Alliance::kBlue)
    {
        TurretGoal = TurretConstants::BottomBlueCoords;
    }
    else if (TurretX > TurretConstants::RedAllianceZoneX && AllianceColor == frc::DriverStation::Alliance::kRed)
    {
        TurretGoal = TurretConstants::RedHubCoords;
    }
    else if (TurretX <= TurretConstants::RedAllianceZoneX && TurretY >= TurretConstants::MidFieldLine && AllianceColor == frc::DriverStation::Alliance::kRed)
    {
        TurretGoal = TurretConstants::TopRedCoords;
    }
    else if (TurretX <= TurretConstants::RedAllianceZoneX && TurretY < TurretConstants::MidFieldLine && AllianceColor == frc::DriverStation::Alliance::kRed)
    {
        TurretGoal = TurretConstants::BottomRedCoords;
    }
    goalPublisher.Set(TurretGoal);

    
}

frc::Pose2d Turret::CalculateTurretPose(const frc::Pose2d &robotPose)
{
    // The turret's pose relative to the robot center
    frc::Transform2d turretTransform{
        frc::Translation2d{
            units::meter_t{TurretConstants::kXOffset},
            units::meter_t{TurretConstants::kYOffset}},
        frc::Rotation2d{GetMeasurement()}};

    // Apply that transform in the robot's frame to get field-relative turret pose
    return robotPose.TransformBy(turretTransform);
}

void Turret::GetBallisticSolution(TurretConstants::BallisticSolutionType solution_type,
                                  units::meters_per_second_t turret_vx,
                                  units::meters_per_second_t turret_vy,
                                  units::meter_t target_distance,
                                  units::meters_per_second_t &launch_speed,
                                  units::radian_t &launch_angle,
                                  units::radian_t &lead_angle,
                                  bool &valid)
{
    double rel_vx, rel_vy, rel_vz;
    frc::SmartDashboard::PutNumber("/Turret/Pose/Target Distance", target_distance.value());

    switch (solution_type)
    {
    case TurretConstants::BallisticSolutionType::HUB:
        m_ballistics_hub_interpolator.interpolate(turret_vx.value(),
                                                  std::abs(turret_vy.value()),
                                                  target_distance.value(),
                                                  rel_vx, rel_vy, rel_vz);
        break;
    case TurretConstants::BallisticSolutionType::GROUND:
        m_ballistics_gnd_interpolator.interpolate(turret_vx.value(),
                                                  std::abs(turret_vy.value()),
                                                  target_distance.value(),
                                                  rel_vx, rel_vy, rel_vz);
        break;
    default:
        valid = false;
        launch_speed = units::meters_per_second_t{0.0};
        lead_angle = units::radian_t{0.0};
        launch_angle = units::radian_t{0.0};
        return;
    }
    if (std::isnan(rel_vx) || std::isnan(rel_vy) || std::isnan(rel_vz))
    {
        valid = false;
        launch_speed = units::meters_per_second_t{0.0};
        lead_angle = units::radian_t{0.0};
        launch_angle = units::radian_t{0.0};
        return;
    }
    valid = true;

    if (turret_vy.value() < 0.0)
    {
        // solution space is symmetrical, but rel_vy is inverted
        rel_vy = -rel_vy;
    }

    // Compute the launch speed (magnitude of velocity vector)
    double v_mag = std::sqrt(rel_vx * rel_vx + rel_vy * rel_vy + rel_vz * rel_vz);

    launch_speed = units::meters_per_second_t{v_mag};

    // Compute the horizontal aim angle
    lead_angle = units::radian_t{std::atan2(rel_vy, rel_vx)};

    // Horizontal velocity component
    double v_horizontal = std::sqrt(rel_vx * rel_vx + rel_vy * rel_vy);

    // Compute the launch angle
    launch_angle = units::radian_t{std::acos(v_horizontal / v_mag)};
}

void Turret::CalculateTargetingSolution(const frc::Pose2d &robotPose, units::second_t dt, bool update,
                                        units::meters_per_second_t &launch_speed, units::radian_t &launch_angle, units::radian_t &turret_angle)
{
    // initial value for robot position and angle in field reference frame
    units::meter_t robotx = robotPose.X();
    units::meter_t roboty = robotPose.Y();

    // robot's initial angle in the field reference frame
    units::radian_t robotAngle = robotPose.Rotation().Radians();

    // Get the robot's linear and angular velocity from the swervedrive.
    // The linear velocities are oriented relative to the robot, not the field.
    auto robotVx = units::meters_per_second_t{frc::SmartDashboard::GetNumber("drive/vx", 0.0)};
    auto robotVy = units::meters_per_second_t{frc::SmartDashboard::GetNumber("drive/vy", 0.0)};
    auto robotOmega = units::radians_per_second_t{frc::SmartDashboard::GetNumber("drive/omega", 0.0)};
    // auto robotVx = units::meters_per_second_t{0.0};
    // auto robotVy = units::meters_per_second_t{0.0};
    // auto robotOmega = 0.0_rad_per_s;

    // Reorient robotVx, robotVy to the world reference frame (field)
    units::meters_per_second_t robotWorldVx = robotVx * std::cos(robotAngle.value()) -
                                              robotVy * std::sin(robotAngle.value());
    units::meters_per_second_t robotWorldVy = robotVx * std::sin(robotAngle.value()) +
                                              robotVy * std::cos(robotAngle.value());

    // estimate the robot's future position using current linear velocity
    robotx += robotWorldVx * dt;
    roboty += robotWorldVy * dt;

    // estimate the robot's future orientation using current angular velocity
    robotAngle += robotOmega * dt;

    // calculate the turret position in the field frame
    units::meter_t turretx = robotx + units::meter_t{TurretConstants::kXOffset} * std::cos(robotAngle.value()) -
                             units::meter_t{TurretConstants::kYOffset} * std::sin(robotAngle.value());
    units::meter_t turrety = roboty + units::meter_t{TurretConstants::kXOffset} * std::sin(robotAngle.value()) +
                             units::meter_t{TurretConstants::kYOffset} * std::cos(robotAngle.value());

    // calculate the distance and angle to the goal in the field coordinate reference frame
    units::meter_t dx = goal.X() - turretx;
    units::meter_t dy = goal.Y() - turrety;
    units::meter_t dist = units::meter_t{std::sqrt(dx.value() * dx.value() + dy.value() * dy.value())};
    units::radian_t angleToGoal = units::radian_t{std::atan2(dy.value(), dx.value())};

    // Get the position of the turret relative to the robot in field orientation
    units::meter_t tdx = turretx - robotx;
    units::meter_t tdy = turrety - roboty;

    // Calculate the turret linear velocity in the world (field) coordinate frame.
    // This is a combination of the robot's linear velocity and the applied
    // angular velocity of the robot on the turret.
    units::meters_per_second_t turretVx = robotWorldVx - robotOmega * tdy / units::radian_t{1};
    units::meters_per_second_t turretVy = robotWorldVy + robotOmega * tdx / units::radian_t{1};

    // Calculate turret velocity in a rotated coordinate frame where the radial
    // direction is towards target.
    units::meters_per_second_t turretVrad = turretVx * std::cos(-angleToGoal.value()) -
                                            turretVy * std::sin(-angleToGoal.value());
    units::meters_per_second_t turretVtan = turretVx * std::sin(-angleToGoal.value()) +
                                            turretVy * std::cos(-angleToGoal.value());

    // Select which ballistic solution to use.  If the height of the goal is
    // zero select the ground solution, otherwise select the hub solution.
    // See ballistics_rv_hub.h and ballistics_rv_gnd.h for details on each.
    TurretConstants::BallisticSolutionType solutionType;
    if (goal.Z().value() > 0.0)
    {
        // assume hub
        solutionType = TurretConstants::BallisticSolutionType::HUB;
    }
    else
    {
        // assume ground
        solutionType = TurretConstants::BallisticSolutionType::GROUND;
    }

    units::meters_per_second_t sol_launch_speed;
    units::radian_t sol_launch_angle;
    units::radian_t sol_lead_angle;
    bool sol_valid;

    // Interpolate the launch angle (hood), launch speed (flywheel), and lead angle from
    // ballistic solution grids using the turret's linear velocity and distance to the
    // target.
    GetBallisticSolution(solutionType, turretVrad, turretVtan, dist,
                         sol_launch_speed, sol_launch_angle, sol_lead_angle, sol_valid);

    if (!sol_valid)
    {
        // use previous solution
        sol_launch_speed = m_BallisticLaunchSpeed;
        sol_launch_angle = m_BallisticLaunchAngle;
        sol_lead_angle = m_BallisticLeadAngle;
    }

    // Compute desired yaw in field frame
    sol_lead_angle = std::clamp(sol_lead_angle, -TurretConstants::kLeadAngleClamp, TurretConstants::kLeadAngleClamp);
    units::radian_t turret_yaw = angleToGoal + sol_lead_angle;
    // units::radian_t turret_yaw = angleToGoal;

    // turret angle is initialized during homing to align with robot frame x-direction
    // subtract the robot angle to get the desired turret angle
    turret_angle = turret_yaw - robotPose.Rotation().Radians() - 0_deg;

    // Normalize the turret angle into the turret's physical range.
    double angle_val = turret_angle.convert<units::deg>().value();
    double min_angle = TurretConstants::kminAngle.convert<units::deg>().value();
    angle_val = angle_val - 360.0 * std::floor((angle_val - min_angle) / 360.0);
    turret_angle = units::degree_t{angle_val};

    launch_speed = sol_launch_speed;
    launch_angle = sol_launch_angle;

    if (update)
    {
        m_BallisticSolutionValid = sol_valid;
        m_BallisticLaunchSpeed = sol_launch_speed;
        m_BallisticDistance = dist;
        m_BallisticLaunchAngle = sol_launch_angle;
        m_BallisticLeadAngle = sol_lead_angle;
        frc::SmartDashboard::PutNumber("/Turret/Ballistics/Velocity X MPS", turretVx.value());
        frc::SmartDashboard::PutNumber("/Turret/Ballistics/Velocity Y MPS", turretVy.value());
        frc::SmartDashboard::PutNumber("/Turret/Ballistics/Velocity Radial MPS", turretVrad.value());
        frc::SmartDashboard::PutNumber("/Turret/Ballistics/Velocity Tangential MPS", turretVtan.value());
        frc::SmartDashboard::PutNumber("/Turret/Ballistics/Target Distance", dist.value());
        switch (solutionType)
        {
        case TurretConstants::BallisticSolutionType::HUB:
            frc::SmartDashboard::PutString("/Turret/Ballistics/Ballistic Solution Type", "HUB");
            break;
        case TurretConstants::BallisticSolutionType::GROUND:
            frc::SmartDashboard::PutString("/Turret/Ballistics/Ballistic Solution Type", "GROUND");
            break;
        }
        frc::SmartDashboard::PutNumber("/Turret/Ballistics/Ballistic Launch Speed MPS", m_BallisticLaunchSpeed.value());
        frc::SmartDashboard::PutNumber("/Turret/Ballistics/Ballistic Launch Angle Deg", units::degree_t{m_BallisticLaunchAngle}.value());
        frc::SmartDashboard::PutNumber("/Turret/Ballistics/Ballistic Lead Angle Deg", units::degree_t{m_BallisticLeadAngle}.value());
        frc::SmartDashboard::PutNumber("/Turret/Ballistics/Ballistic Turret Angle Deg", units::degree_t{turret_angle}.value());
        // Flag indicates if we have a valid solution.  We may or may not want to pause
        // shooting.  This typically occurs when driving the robot toward the hub at
        // high velocity, which should be a short-term temporary condition.  The hood's
        // angle limit would be exceeded here because the robot's radial velocity
        // must be offset, resulting in a higher launch angle.
        frc::SmartDashboard::PutBoolean("/Turret/Ballistics/Ballistic Solution Valid", m_BallisticSolutionValid);
    }
}
