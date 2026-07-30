// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
#include "subsystems/Wrist.h"

using State = frc::TrapezoidProfile<units::degrees>::State;
using degrees_per_second_squared_t =
    units::unit_t<units::compound_unit<units::angular_velocity::degrees_per_second,
                                       units::inverse<units::time::seconds>>>;

Wrist::Wrist() : m_controller(
                     WristConstants::kAngleP, WristConstants::kAngleI, WristConstants::kAngleD,
                     frc::TrapezoidProfile<units::degrees>::Constraints(WristConstants::kArmVelLimit, WristConstants::kArmAccelLimit), 5_ms),
                 m_motor(WristConstants::kAngleMotorId, rev::spark::SparkLowLevel::MotorType::kBrushless), m_feedforward(WristConstants::kFFks, WristConstants::kFFkg, WristConstants::kFFkV,
                                                                                                                         WristConstants::kFFkA),
                //  m_encoder{m_motor.GetEncoder()},
                 m_encoder{m_motor.GetAbsoluteEncoder()},

                 m_WristSim(WristConstants::kSimMotor, WristConstants::kGearRatio, WristConstants::kmoi,
                            WristConstants::kWristLength, WristConstants::kminAngle, WristConstants::kmaxAngle,
                            WristConstants::kGravity, WristConstants::kWristStartAngle)
{
    m_controller.SetIZone(WristConstants::kIZone);
    rev::spark::SparkBaseConfig config;
    // config.Inverted(true);
    // config.limitSwitch.ForwardLimitSwitchTriggerBehavior(rev::spark::LimitSwitchConfig::Behavior::kStopMovingMotor);
    config.SetIdleMode(rev::spark::SparkBaseConfig::IdleMode::kBrake);
    // config.encoder.PositionConversionFactor(360 / 81.0);
    config.absoluteEncoder.PositionConversionFactor(360.0);
    config.absoluteEncoder.ZeroOffset(WristConstants::kAbsoluteOffset);
    config.absoluteEncoder.Inverted(true);
    config.absoluteEncoder.ZeroCentered(true);
    config.SmartCurrentLimit(30, 0, 20000);
    

    m_motor.Configure(config, rev::spark::SparkMax::ResetMode::kResetSafeParameters, rev::spark::SparkMax::PersistMode::kPersistParameters);

    m_controller.SetTolerance(WristConstants::kTolerancePos, WristConstants::kToleranceVel);
    // Start m_arm in neutral position
    m_WristState = WristConstants::DISABLED;

    wpi::log::DataLog &log = frc::DataLogManager::GetLog();
    m_AngleLog = wpi::log::DoubleLogEntry(log, "/Wrist/Angle");
    m_SetPointLog = wpi::log::DoubleLogEntry(log, "/Wrist/Setpoint");
    m_StateLog = wpi::log::IntegerLogEntry(log, "/Wrist/State");
    m_MotorCurrentLog = wpi::log::DoubleLogEntry(log, "/Wrist/MotorCurrent");
    m_MotorVoltageLog = wpi::log::DoubleLogEntry(log, "/Wrist/MotorVoltage");

   
}

void Wrist::SimulationPeriodic()
{
    m_WristSim.Update(10_ms);
    frc::SmartDashboard::PutNumber("/Wrist/Motor current draw", m_WristSim.GetCurrentDraw().value());
}

units::degree_t Wrist::GetMeasurement()
{ // original get measurement function
    frc::SmartDashboard::PutNumber("/Wrist/intakeWristAngle", m_encoder.GetPosition());
    if constexpr (frc::RobotBase::IsSimulation())
    {
        return m_WristSim.GetAngle();
        frc::SmartDashboard::PutNumber("/Wrist/SIMintakeWristAngle", double(m_WristSim.GetAngle()*180.0_rad/3.14159));
    }

    return units::degree_t{m_encoder.GetPosition()};
}

void Wrist::SetAngle(double wristAngleGoal)
{
    if (m_WristState != WristConstants::ZEROING)
    {
        // m_WristState = WristConstants::MOVE;
        m_WristState = WristConstants::START;
        m_goal = units::angle::degree_t(wristAngleGoal);
        // m_controller.Reset(GetMeasurement());
        // m_controller.SetGoal(m_goal);
    }
}

void Wrist::Periodic()
{

    printLog();
    double fb;
    units::volt_t ff;
    units::volt_t v;

    switch (m_WristState)
    {
    case WristConstants::START:
    {
        frc::SmartDashboard::PutString("/Wrist/State", "START");
        m_controller.Reset(GetMeasurement());
        m_controller.SetGoal(m_goal);
        m_WristState = WristConstants::MOVE;
    }
    case WristConstants::ZEROING:
    {
        frc::SmartDashboard::PutString("/Wrist/State", "ZEROING");
        // m_motor.Set(0.1);
        break;
    }
    case WristConstants::MOVE:
    {

        frc::SmartDashboard::PutString("/Wrist/State", "MOVING");
        if (m_controller.AtGoal())
        {
            m_WristState = WristConstants::HOLD;
        }
        else
        {
            fb = m_controller.Calculate(GetMeasurement());
            ff = m_feedforward.Calculate(units::radian_t{m_controller.GetSetpoint().position}, units::radians_per_second_t{m_controller.GetSetpoint().velocity}, units::radians_per_second_squared_t{m_controller.GetSetpoint().velocity / 1_s});
            v = units::volt_t{fb} + ff;
            if constexpr (frc::RobotBase::IsSimulation())
            {
                m_WristSim.SetInputVoltage(v);
            }
            m_motor.SetVoltage(v);
        }
        break;
    }
    case WristConstants::HOLD:
    {
        frc::SmartDashboard::PutString("/Wrist/State", "HOLD");
        double fb = m_controller.Calculate(GetMeasurement());
        units::volt_t ff = m_feedforward.Calculate(units::radian_t{m_controller.GetSetpoint().position}, units::radians_per_second_t{m_controller.GetSetpoint().velocity}, units::radians_per_second_squared_t{m_controller.GetSetpoint().velocity / 1_s});
        units::volt_t v = units::volt_t{fb} + ff;
        if constexpr (frc::RobotBase::IsSimulation())
        {
            m_WristSim.SetInputVoltage(v);
        }
        m_motor.SetVoltage(v);
        break;
    }
    case WristConstants::DISABLED:
        frc::SmartDashboard::PutString("/Wrist/State", "DISABLED");
        break;
    default:
    {
        frc::SmartDashboard::PutString("/Wrist/State", "default");
        break;
    }
    }
    
}

WristConstants::WristState Wrist::GetState()
{
    return m_WristState;
}

void Wrist::Zero()
{
    // m_WristState = WristConstants::ZEROING;
}

void Wrist::printLog()
{
    frc::SmartDashboard::PutNumber("/Wrist/Actual Angle", GetMeasurement().value());
    frc::SmartDashboard::PutNumber("/Wrist/Goal Angle", m_controller.GetGoal().position.value());
    frc::SmartDashboard::PutNumber("/Wrist/Setpoint",
                                   m_controller.GetSetpoint().position.value());
    frc::SmartDashboard::PutBoolean("/Wrist/Limit Switch", m_motor.GetReverseLimitSwitch().Get());
    m_AngleLog.Append(GetMeasurement().value());
    m_SetPointLog.Append(m_controller.GetSetpoint().position.value());
    m_StateLog.Append(m_WristState);
    m_MotorCurrentLog.Append(m_motor.GetOutputCurrent());
    m_MotorVoltageLog.Append(m_motor.GetAppliedOutput());
}

void Wrist::Disable()
{
    m_motor.StopMotor();
}

void Wrist::HoldPosition()
{
    // if (m_WristState != WristConstants::WristState::HOLD)
    {
        m_controller.Reset(GetMeasurement());
        m_controller.SetGoal(GetMeasurement());
        m_goal = GetMeasurement();
        m_WristState = WristConstants::WristState::HOLD;
    }
}