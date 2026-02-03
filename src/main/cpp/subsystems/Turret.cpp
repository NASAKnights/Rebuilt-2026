// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
#include "subsystems/Turret.h"

#include <frc/geometry/Transform3d.h>
#include <frc/geometry/Translation2d.h>
#include <cmath>

// using State = frc::TrapezoidProfile<units::degrees>::State;
using degrees_per_second_squared_t =
    units::unit_t<units::compound_unit<units::angular_velocity::degrees_per_second,
                                       units::inverse<units::time::seconds>>>;

Turret::Turret() : m_controller(
                       TurretConstants::kAngleP, TurretConstants::kAngleI, TurretConstants::kAngleD),

                   m_motor(TurretConstants::kAngleMotorId), m_feedforward(TurretConstants::kFFks, TurretConstants::kFFkg, TurretConstants::kFFkV,
                                                                          TurretConstants::kFFkA),

                   m_TurretSim(TurretConstants::kSimMotor, TurretConstants::kGearRatio, TurretConstants::kmoi,
                               TurretConstants::kTurretRadius, TurretConstants::kminAngle, TurretConstants::kmaxAngle,
                               TurretConstants::kGravity, TurretConstants::kTurretStartAngle, TurretConstants::kSimNoise)
{
    // m_motor.SetInverted(true);
    ctre::phoenix6::configs::TalonFXConfiguration config;
    config.MotorOutput.Inverted = ctre::phoenix6::signals::InvertedValue::Clockwise_Positive;
    m_motor.GetConfigurator().Apply(config);
    m_controller.SetIZone(TurretConstants::kIZone);

    m_controller.SetTolerance(TurretConstants::kTolerancePos.value(), TurretConstants::kToleranceVel.value());
    // Start m_Turret in neutral position
    m_TurretState = TurretConstants::TRACKING;
    wpi::log::DataLog &log = frc::DataLogManager::GetLog();
    m_AngleLog = wpi::log::DoubleLogEntry(log, "/Turret/Angle");
    m_SetPointLog = wpi::log::DoubleLogEntry(log, "/Turret/Setpoint");
    m_StateLog = wpi::log::IntegerLogEntry(log, "/Turret/State");
    m_MotorCurrentLog = wpi::log::DoubleLogEntry(log, "/Turret/MotorCurrent");
    m_MotorVoltageLog = wpi::log::DoubleLogEntry(log, "/Turret/MotorVoltage");

    networkTableInst = nt::NetworkTableInstance::GetDefault();
    auto poseTable = networkTableInst.GetTable("ROS2Bridge");
    baseLinkSubscriber = poseTable->GetDoubleArrayTopic(robotPoseLink).Subscribe({}, {.periodic = 0.02, .sendAll = true});
    
    // Initialize goal topic - publish default and subscribe for updates
    auto turretTable = networkTableInst.GetTable("Turret");
    goalPublisher = turretTable->GetDoubleArrayTopic("goal").Publish();
    std::vector<double> defaultGoal = {4.5, 4.0, 0.0};
    goalSubscriber = turretTable->GetDoubleArrayTopic("goal").Subscribe(defaultGoal, {.periodic = 0.02, .sendAll = true});
    // Publish initial default goal
    goalPublisher.Set(defaultGoal);

    m_turretObject = m_turretField.GetObject("Turret");
    frc::SmartDashboard::PutData("Turret Field", &m_turretField);
    m_motor.SetPosition(0.0_rad);
    SetAngle(0.0_deg);

    // if constexpr(frc::RobotBase::IsSimulation())
    // {
    //     m_simTimer.Start();
    // }
    // const frc::DCMotor, const double, const units::moment_of_inertia::kilogram_square_meter_t, const units::length::meter_t,
    // const units::angle::radian_t, const units::angle::radian_t, const bool, const units::angle::radian_t, const std::array<double, 1U>)
}

void Turret::SimulationPeriodic()
{
    m_TurretSim.Update(10_ms);
    frc::SmartDashboard::PutNumber("Motor current draw", m_TurretSim.GetCurrentDraw().value());
}

units::degree_t Turret::GetMeasurement()
{ // original get measurement function
    if constexpr (frc::RobotBase::IsSimulation())
    {
        return m_TurretSim.GetAngle();
    }

    return units::turn_t{(m_motor.GetPosition().GetValue() / TurretConstants::kGearRatio)};
}

std::pair<units::degree_t, units::degrees_per_second_t> Turret::findTrackingAngle()
{
    std::vector<double> baseLinkPose = baseLinkSubscriber.Get({});
    auto baseLink = DoubleArrayToPose2d(baseLinkPose);
    if (!baseLink.has_value())
    {
        return {GetMeasurement(), 0_deg_per_s};
    }

    frc::Transform3d world2robot = frc::Transform3d(baseLink->X(), baseLink->Y(), 0_m, frc::Rotation3d(0_rad, 0_rad, baseLink->Rotation().Radians()));

    // Read goal from NetworkTables
    std::vector<double> defaultGoal = {4.0, 4.0, 0.0};
    std::vector<double> goalArray = goalSubscriber.Get(defaultGoal);
    if (goalArray.size() >= 3) {
        goal = frc::Transform3d(
            units::meter_t{goalArray[0]},
            units::meter_t{goalArray[1]},
            units::meter_t{goalArray[2]},
            frc::Rotation3d()
        );
    }
    
    frc::Transform3d world2goal = goal;

    // world2turret rotation matrix
    frc::Transform3d world2turret =
        frc::Transform3d(baseLink->X() + units::length::meter_t{TurretConstants::kXOffset},
                         baseLink->Y() + units::length::meter_t{TurretConstants::kYOffset},
                         units::length::meter_t{TurretConstants::kZOffset},
                         frc::Rotation3d(0.0_rad, 0.0_rad,
                                         units::angle::radian_t{GetMeasurement().convert<units::angle::radians>()} + baseLink->Rotation().Radians()));

    // grab world2robot, world2goal, robot2turret transforms

    // get robot2turret transform from world2robot * (world2turret)^-1

    frc::Transform3d robot2turret = frc::Transform3d(world2robot.ToMatrix().inverse() * world2turret.ToMatrix());
    // get turret2goal transform from (world2turret)^-1 * world2goal

    // Compute world-space vector from turret to goal
    Eigen::Vector3d tgVector = (world2goal.Translation() - world2turret.Translation()).ToVector();

    // Compute desired yaw in world frame
    units::radian_t targetYaw = units::radian_t{std::atan2(tgVector.y(), tgVector.x())};

    // Get current turret yaw in world frame
    units::radian_t turretYaw = world2turret.Rotation().ToRotation2d().Radians();

    // Find smallest signed error
    units::radian_t error = frc::AngleModulus(targetYaw - turretYaw);

    // Add to current turret angle
    units::degree_t newTarget = GetMeasurement() + units::degree_t{error};

    // If the computed target exceeds the upper limit by >180°, it likely wrapped
    if (newTarget > TurretConstants::kmaxAngle)
    {
        // If we’re only just beyond by less than 180°, clamp
        if (newTarget - 360_deg >= TurretConstants::kminAngle)
            newTarget -= 360_deg;
        else
            newTarget = TurretConstants::kmaxAngle;
    }
    else if (newTarget < TurretConstants::kminAngle)
    {
        if (newTarget + 360_deg <= TurretConstants::kmaxAngle)
            newTarget += 360_deg;
        else
            newTarget = TurretConstants::kminAngle;
    }
    frc::SmartDashboard::PutNumber("/Turret/newTarget", double(newTarget));
    
    // Calculate tangential velocity feedforward
    auto robotVx = units::meters_per_second_t{frc::SmartDashboard::GetNumber("drive/vx", 0.0)};
    auto robotVy = units::meters_per_second_t{frc::SmartDashboard::GetNumber("drive/vy", 0.0)};
    auto robotOmega = units::radians_per_second_t{frc::SmartDashboard::GetNumber("drive/omega", 0.0)};

    // Robot velocity vector in field frame (approximate, since we don't have full odometry velocity here easily)
    // Actually, drive/vx and vy are usually robot-relative or field-relative depending on how they are pushed. 
    // In SwerveDrive.cpp, they are pushed as the commanded chassis speeds (Robot Relative? No, SwerveDrive::Drive usually takes field relative if driven that way, but let's check).
    // SwerveDrive::Drive takes whatever is passed. In Robot.cpp, it seems field relative is used.
    // However, the dashboard values come from SwerveDrive::Drive, which receives the result of WeightedDriving.
    
    // Let's assume field relative for now as that's typical for swerve.
    
    // Relative velocity of goal wrt robot: v_g_r = v_g - v_r
    // v_g = 0 (static goal)
    // v_r = v_robot_trans + omega_robot x r_turret
    // But simply: we need the tangential component of the robot's velocity relative to the goal.

    // Tangential velocity = v_perp / distance
    // v_perp is the component of robot velocity perpendicular to the turret-to-goal vector.
    
    // Vector from Turret to Goal
    units::meter_t dx = world2goal.X() - world2turret.X();
    units::meter_t dy = world2goal.Y() - world2turret.Y();
    units::meter_t dist = units::meter_t{std::sqrt(dx.value()*dx.value() + dy.value()*dy.value())};

    // Angle to goal
    auto angleToGoal = units::radian_t{std::atan2(dy.value(), dx.value())};
    
    // Rotate robot velocity into frame aligned with goal vector
    // tangential component is -sin(theta)*vx + cos(theta)*vy
    // Be careful with frames. If vx, vy are field relative:
    auto v_tangential = -units::math::sin(angleToGoal) * robotVx + units::math::cos(angleToGoal) * robotVy;

    // Angular velocity contribution from translation: omega = v_tan / r
    auto omega_trans = v_tangential * (1.0_rad / dist);

    // Total required turret velocity = - (omega_trans) - omega_robot
    // The turret needs to counter-rotate against the robot's rotation AND track the goal translation.
    // If robot rotates CCW, turret must rotate CW (negative) to stay fixed.
    // If robot moves tangentially such that goal moves "left" in view, turret must rotate "left".
    
    auto feedforwardVel = -omega_trans - robotOmega;
    
    return {newTarget, units::degrees_per_second_t{feedforwardVel}};
}

void Turret::SetAngle(units::degree_t TurretAngleGoal, units::degrees_per_second_t velocityGoal)
{
    if (!(TurretAngleGoal.value() < m_goal.value() + TurretConstants::kTolerancePos.value() && TurretAngleGoal.value() > m_goal.value() - TurretConstants::kTolerancePos.value()))
    {
        if ((TurretAngleGoal <= TurretConstants::kmaxAngle) &&
            (TurretAngleGoal >= TurretConstants::kminAngle))
        {
            units::degrees_per_second_t robotVel = units::degrees_per_second_t{frc::SmartDashboard::GetNumber("Angular velocity", 0.0)};
            auto velocity = GetVelocity();
            m_goal = units::angle::degree_t(TurretAngleGoal);
            if (abs(velocity.value()) < (1_deg_per_s).value())
            {
                velocity = 1_deg_per_s * copysign(1.0, velocity.value());
            }
            m_velocityGoal = velocityGoal;
            m_controller.SetSetpoint(m_goal.value());
        }
    }
    frc::SmartDashboard::PutNumber("/Turret/m_goal", double(m_goal));
}

units::degrees_per_second_t Turret::GetVelocity()
{
    return (m_motor.GetVelocity().GetValue() / TurretConstants::kGearRatio);
}

void Turret::Periodic()
{

    printLog();
    UpdateFieldVisuals();
    double fb;
    units::volt_t ff;
    units::volt_t v;
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
        auto [angle, velocity] = findTrackingAngle();
        SetAngle(angle, velocity);
        fb = m_controller.Calculate(GetMeasurement().value());
        ff = m_feedforward.Calculate(angle, velocity);
        v = units::volt_t{fb} + ff;
        if(v > units::volt_t{2.0}){
            v = 2.0_V;
        }
        else if(v < -2.0_V){
            v = -2.0_V;
        }
        // units::degrees_per_second_t robotVel = units::degrees_per_second_t{frc::SmartDashboard::GetNumber("Angular velocity", 0.0)};
        // auto turretVel = GetVelocity();

        frc::SmartDashboard::PutNumber("/Turret/Voltage", double(v));
        break;
    }
    default:
    {
        frc::SmartDashboard::PutString("/Turret/State", "default");
        break;
    }
    }
    if constexpr (frc::RobotBase::IsSimulation())
    {
        m_TurretSim.SetInputVoltage(v);
        SimulationPeriodic();
    }
    m_motor.SetVoltage(v);
}

TurretConstants::TurretState Turret::GetState()
{
    return m_TurretState;
}

void Turret::printLog()
{
    frc::SmartDashboard::PutNumber("/Turret/Actual Angle", GetMeasurement().value());
    frc::SmartDashboard::PutNumber("/Turret/Goal Angle", m_controller.GetSetpoint());
    frc::SmartDashboard::PutNumber("/Turret/setpoint",
                                   m_controller.GetSetpoint());
    frc::SmartDashboard::PutNumber("/Turret/velocity", double(GetVelocity()));
    m_AngleLog.Append(GetMeasurement().value());
    m_SetPointLog.Append(m_controller.GetSetpoint());
    m_StateLog.Append(m_TurretState);
    // m_MotorCurrentLog.Append(m_motor.GetOutputCurrent());
    // m_MotorVoltageLog.Append(m_motor.GetAppliedOutput());
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

frc::Pose2d Turret::CalculateTurretPose(const frc::Pose2d &robotPose)
{
    // The turret's pose relative to the robot center
    frc::Transform2d turretTransform{
        frc::Translation2d{
            units::meter_t{TurretConstants::kXOffset},
            units::meter_t{TurretConstants::kYOffset}},
        frc::Rotation2d{GetMeasurement() + TurretConstants::kAngleOffset}};

    // Apply that transform in the robot's frame to get field-relative turret pose
    return robotPose.TransformBy(turretTransform);
}
