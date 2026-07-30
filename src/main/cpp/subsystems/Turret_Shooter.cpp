// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Turret_Shooter.h"
#include <algorithm>

Turret_Shooter::Turret_Shooter()
{
    wpi::log::DataLog& log = frc::DataLogManager::GetLog();
    m_LeftMotorCurrentLog = wpi::log::DoubleLogEntry(log, "/Turret/Shooter/LeftMotorCurrent");
    m_LeftMotorVoltageLog = wpi::log::DoubleLogEntry(log, "/Turret/Shooter/LeftMotorVoltage");
    m_RightMotorCurrentLog = wpi::log::DoubleLogEntry(log, "/Turret/Shooter/RightMotorCurrent");
    m_RightMotorVoltageLog = wpi::log::DoubleLogEntry(log, "/Turret/Shooter/RightMotorVoltage");
    m_SpindexerMotorCurrentLog = wpi::log::DoubleLogEntry(log, "/Turret/Spindexer/MotorCurrent");
    m_SpindexerMotorVoltageLog = wpi::log::DoubleLogEntry(log, "/Turret/Spindexer/MotorVoltage");
    m_IndexerMotorCurrentLog = wpi::log::DoubleLogEntry(log, "/Turret/Indexer/MotorCurrent");
    m_IndexerMotorVoltageLog = wpi::log::DoubleLogEntry(log, "/Turret/Indexer/MotorVoltage");

    ctre::phoenix6::configs::TalonFXConfiguration leftMotorConfig{};
    ctre::phoenix6::configs::TalonFXConfiguration rightMotorConfig{};
    ctre::phoenix6::controls::Follower LeftFollower{m_rightMotor.GetDeviceID(), true};
    // leftMotorConfig.Commutation.WithMotorArrangement(ctre::phoenix6::signals::MotorArrangementValue::Minion_JST);
    // rightMotorConfig.Commutation.WithMotorArrangement(ctre::phoenix6::signals::MotorArrangementValue::Minion_JST);
    m_leftMotor.SetControl(LeftFollower);
    ctre::phoenix6::configs::Slot0Configs motorSlot0Configs{};
    motorSlot0Configs.kP = kP;
    motorSlot0Configs.kI = kI;
    motorSlot0Configs.kD = kD;
    motorSlot0Configs.kS = kS;
    motorSlot0Configs.kA = kA;
    motorSlot0Configs.kV = kV;
    leftMotorConfig.Slot0 = motorSlot0Configs;
    rightMotorConfig.Slot0 = motorSlot0Configs;
    m_leftMotor.SetNeutralMode(ctre::phoenix6::signals::NeutralModeValue::Coast);
    m_rightMotor.SetNeutralMode(ctre::phoenix6::signals::NeutralModeValue::Coast);
    ctre::phoenix6::configs::CurrentLimitsConfigs currentConfig{};
    currentConfig.SupplyCurrentLimitEnable = kEnableCurrentLimit;
    currentConfig.SupplyCurrentLimit = kPeakCurrentLimit;
    currentConfig.SupplyCurrentLowerLimit = kContinousCurrentLimit;
    currentConfig.SupplyCurrentLowerTime = kPeakCurrentDuration;
    leftMotorConfig.CurrentLimits = currentConfig;
    rightMotorConfig.CurrentLimits = currentConfig;

    leftMotorConfig.MotorOutput.Inverted = true;
    rightMotorConfig.MotorOutput.Inverted = false;

    ctre::phoenix::StatusCode leftStatus = m_leftMotor.GetConfigurator().Apply(leftMotorConfig);
    ctre::phoenix::StatusCode rightStatus = m_rightMotor.GetConfigurator().Apply(rightMotorConfig);

    frc::SmartDashboard::PutBoolean("/Turret/Shooter/Left Motor Status", false);
    frc::SmartDashboard::PutBoolean("/Turret/Shooter/Right Motor Status", false);


    frc::SmartDashboard::PutBoolean("/Turret/Shooter/Left Motor Status", leftStatus.IsOK());
    frc::SmartDashboard::PutBoolean("/Turret/Shooter/Right Motor Status", rightStatus.IsOK());

    frc::SmartDashboard::PutNumber("/Turret/Shooter/Commanded Ball Speed MPS", 0.0);
    frc::SmartDashboard::PutNumber("/Turret/Shooter/Commanded Motor RPM", 0.0);

    frc::SmartDashboard::PutBoolean("/Turret/Shooter/Ball Speed Manual Override", false);
    frc::SmartDashboard::PutNumber("/Turret/Shooter/Ball Speed Manual Set MPS", 0.0);

    frc::SmartDashboard::PutNumber("/Turret/Shooter/Actual Motor RPM", 0.0);
    frc::SmartDashboard::PutNumber("/Turret/Shooter/Actual Ball Speed MPS", 0.0);

    frc::SmartDashboard::PutNumber("/Turret/Shooter/Left Motor Voltage", 0.0);
    frc::SmartDashboard::PutNumber("/Turret/Shooter/Right Motor Voltage", 0.0);

    rev::spark::SparkFlexConfig indexerConfig{};
    indexerConfig.closedLoop.Pid(Turret_ShooterConstants::kIndexerP, Turret_ShooterConstants::kIndexerI, Turret_ShooterConstants::kIndexerD);
    indexerConfig.closedLoop.feedForward.kV(Turret_ShooterConstants::kIndexerkV);
    // indexerConfig.SmartCurrentLimit(30);
    m_indexerMotor.Configure(indexerConfig, rev::spark::SparkBase::ResetMode::kResetSafeParameters, rev::spark::SparkBase::PersistMode::kPersistParameters);

    ctre::phoenix6::configs::TalonFXConfiguration spindexerConfig{};
    spindexerConfig.Slot0.kP = Turret_ShooterConstants::kSpindexerP;
    spindexerConfig.Slot0.kI = Turret_ShooterConstants::kSpindexerI;
    spindexerConfig.Slot0.kD = Turret_ShooterConstants::kSpindexerD;
    spindexerConfig.Slot0.kS = Turret_ShooterConstants::kSpindexerS;
    spindexerConfig.Slot0.kV = Turret_ShooterConstants::kSpindexerV;
    m_spindexerMotor.GetConfigurator().Apply(spindexerConfig);
    
    frc::SmartDashboard::PutBoolean("/Turret/Spindexer Indexer/Running", false);

    if constexpr (frc::RobotBase::IsSimulation()) {
        m_simTimer.Start();
    }
}

void Turret_Shooter::SetMotorSpeed(units::turns_per_second_t motorSpeed) {
    auto motorRequest = ctre::phoenix6::controls::VelocityVoltage{motorSpeed};
    ctre::phoenix::StatusCode leftStatus = m_leftMotor.SetControl(motorRequest.WithVelocity(motorSpeed).WithSlot(0));
    ctre::phoenix::StatusCode rightStatus = m_rightMotor.SetControl(motorRequest.WithVelocity(motorSpeed).WithSlot(0));
    frc::SmartDashboard::PutNumber("/Turret/Shooter/Commanded Motor RPS", motorSpeed.value());
    // frc::SmartDashboard::PutNumber("/Turret/Shooter/Commanded Motor RPM", motorSpeed.value() * 60.0);
}

void Turret_Shooter::SetSpeed(units::meters_per_second_t ballSpeed, units::meter_t distance) {
    double distVal = distance.value();
    double kFlyWheelVelocityGain = 1.95; // Default fallback

    if (!kFlyWheelGainMap.empty()) {
        auto itHigh = kFlyWheelGainMap.lower_bound(distVal);
        
        if (itHigh == kFlyWheelGainMap.begin()) {
            // Distance is smaller than our first entry
            kFlyWheelVelocityGain = itHigh->second;
        } else if (itHigh == kFlyWheelGainMap.end()) {
            // Distance is larger than our last entry
            kFlyWheelVelocityGain = std::prev(itHigh)->second;
        } else {
            // Interpolate between prev and itHigh
            auto itLow = std::prev(itHigh);
            double d1 = itLow->first;
            double g1 = itLow->second;
            double d2 = itHigh->first;
            double g2 = itHigh->second;

            double t = (distVal - d1) / (d2 - d1);
            kFlyWheelVelocityGain = g1 + t * (g2 - g1);
        }
    }

    frc::SmartDashboard::PutNumber("/Turret/Shooter/Current Velocity Gain", kFlyWheelVelocityGain);

    units::turns_per_second_t motorSpeed = (kFlyWheelVelocityGain * ballSpeed * units::radian_t{1} * 4.0) / (kFlywheelDiameter * kGearRatio);

    //Spin motor 10% faster than needed to account for loss of speed when shooting rapidly
    // motorSpeed += motorSpeed * Turret_ShooterConstants::kPercentBoost;
    
    auto motorRequest = ctre::phoenix6::controls::VelocityVoltage{motorSpeed};
    ctre::phoenix::StatusCode leftStatus = m_leftMotor.SetControl(motorRequest.WithVelocity(motorSpeed).WithSlot(0));
    ctre::phoenix::StatusCode rightStatus = m_rightMotor.SetControl(motorRequest.WithVelocity(motorSpeed).WithSlot(0));
    frc::SmartDashboard::PutBoolean("/Turret/Shooter/Left Motor Status", leftStatus.IsOK());
    frc::SmartDashboard::PutBoolean("/Turret/Shooter/Right Motor Status", rightStatus.IsOK());
    frc::SmartDashboard::PutNumber("/Turret/Shooter/Commanded Ball Speed MPS", ballSpeed.value());
    frc::SmartDashboard::PutNumber("/Turret/Shooter/Commanded Motor RPM", motorSpeed.value() * 60.0);
}

void Turret_Shooter::RunSpindexerIndexer(units::meter_t distance) 
{
        m_indexerPID.SetSetpoint(Turret_ShooterConstants::kIndexerShootVelocityRPM, rev::spark::SparkBase::ControlType::kVelocity);
        auto spindexerRequest = ctre::phoenix6::controls::VelocityVoltage{Turret_ShooterConstants::kSpindexerShootVelocity}.WithSlot(0);
        m_spindexerMotor.SetControl(spindexerRequest);
        m_spindexerTargetVelocity = Turret_ShooterConstants::kSpindexerShootVelocity;
        m_indexerTargetVelocityRPM = Turret_ShooterConstants::kIndexerShootVelocityRPM;

    frc::SmartDashboard::PutBoolean("/Turret/Spindexer Indexer/Running", true);
 }

void Turret_Shooter::StopSpindexerIndexer()
{
    m_indexerMotor.Set(0.0);
    auto spindexerRequest = ctre::phoenix6::controls::VoltageOut{0_V};
    m_spindexerMotor.SetControl(spindexerRequest);
    m_spindexerTargetVelocity = 0_tps;
    m_indexerTargetVelocityRPM = 0.0;

    frc::SmartDashboard::PutBoolean("/Turret/Spindexer Indexer/Running", false);
}

void Turret_Shooter::ChangeSpeedMapValue(double deltaValue){
    double distVal = frc::SmartDashboard::GetNumber("/Turret/Ballistics/Target Distance", 0);

    if (!kFlywheelSpeedMap.empty()){
        auto itHigh = kFlywheelSpeedMap.lower_bound(distVal);
        //TODO: Make kMaxShooterGain and kMinShooterGain
        if(!(itHigh->second + deltaValue > 125) || !(itHigh->second + deltaValue < 10)){
            itHigh->second += deltaValue;
        } 
    }
}

void Turret_Shooter::StopMotors()
{
    auto motorRequest = ctre::phoenix6::controls::VelocityVoltage{0_tps};
    auto motorVoltageRequest = ctre::phoenix6::controls::VoltageOut{0_V};
    auto motorSpeed = units::radians_per_second_t{0.0};
    m_leftMotor.SetControl(motorVoltageRequest.WithOutput(0_V));
    m_rightMotor.SetControl(motorVoltageRequest.WithOutput(0_V));
    frc::SmartDashboard::PutNumber("/Turret/Shooter/Commanded Ball Speed MPS", 0.0);
    // frc::SmartDashboard::PutNumber("/Turret/Shooter/Commanded Motor RPM", 0.0);
}

void Turret_Shooter::RunAll()
{
    SetSpeed(units::meters_per_second_t{18.0}, 2.0_m);
    m_indexerPID.SetSetpoint(Turret_ShooterConstants::kIndexerShootVelocityRPM, rev::spark::SparkBase::ControlType::kVelocity);
    auto spindexerRequest = ctre::phoenix6::controls::VelocityVoltage{Turret_ShooterConstants::kSpindexerShootVelocity}.WithSlot(0);
    m_spindexerMotor.SetControl(spindexerRequest);
    m_spindexerTargetVelocity = Turret_ShooterConstants::kSpindexerShootVelocity;
    m_indexerTargetVelocityRPM = Turret_ShooterConstants::kIndexerShootVelocityRPM;
}

void Turret_Shooter::StopAll()
{
    SetSpeed(units::meters_per_second_t{0.0}, 0.0_m);
    m_indexerMotor.Set(0.0);
    auto spindexerRequest = ctre::phoenix6::controls::VoltageOut{0_V};
    m_spindexerMotor.SetControl(spindexerRequest);
    m_spindexerTargetVelocity = 0_tps;
    m_indexerTargetVelocityRPM = 0.0;
}

std::map<double, double> Turret_Shooter::GetCurrentMapState() {
    return kFlywheelSpeedMap;
}

void Turret_Shooter::SetCurrentMapState(std::map<double, double> inputCurrentState) {
    kFlyWheelGainMap = inputCurrentState;
}

units::meters_per_second_t Turret_Shooter::GetActualBallSpeed()
{
    // Note: This returns a theoretical ball speed based on the first gain entry as a baseline.
    units::turns_per_second_t motorSpeed = m_leftMotor.GetVelocity().GetValue();
    double baselineGain = kFlyWheelGainMap.empty() ? 1.95 : kFlyWheelGainMap.begin()->second;
    units::meters_per_second_t ballSpeed = motorSpeed * (kFlywheelDiameter * kGearRatio) / (baselineGain * units::radian_t{1} * 4.0); 
    return ballSpeed;
}

units::turns_per_second_t Turret_Shooter::GetActualMotorSpeed()
{
    return m_leftMotor.GetVelocity().GetValue();
}

units::turns_per_second_t Turret_Shooter::ConvertBallSpeed2Motor(units::meters_per_second_t ballSpeed, units::meter_t distance)
{
    // Using first map entry as a safe status reference
    double baselineGain = kFlyWheelGainMap.empty() ? 1.95 : kFlyWheelGainMap.begin()->second;
    double distVal = distance.value();
    if (!kFlyWheelGainMap.empty()) {
        auto itHigh = kFlyWheelGainMap.lower_bound(distVal);
        
        if (itHigh == kFlyWheelGainMap.begin()) {
            // Distance is smaller than our first entry
            baselineGain = itHigh->second;
        } else if (itHigh == kFlyWheelGainMap.end()) {
            // Distance is larger than our last entry
            baselineGain = std::prev(itHigh)->second;
        } else {
            // Interpolate between prev and itHigh
            auto itLow = std::prev(itHigh);
            double d1 = itLow->first;
            double g1 = itLow->second;
            double d2 = itHigh->first;
            double g2 = itHigh->second;

            double t = (distVal - d1) / (d2 - d1);
            baselineGain = g1 + t * (g2 - g1);
        }
    }

    return (baselineGain * ballSpeed * units::radian_t{1} * 4.0) / (kFlywheelDiameter * kGearRatio);
}

units::turns_per_second_t Turret_Shooter::GetMotorSpeedFromMap(units::meter_t distance)
{
    // Using first map entry as a safe status reference

    double motorSpeed = kFlywheelSpeedMap.empty() ? Turret_ShooterConstants::kMinLaunchRPS : kFlywheelSpeedMap.begin()->second;
    double distVal = distance.value();
    if (!kFlywheelSpeedMap.empty()) {
        auto itHigh = kFlywheelSpeedMap.lower_bound(distVal);
        
        if (itHigh == kFlywheelSpeedMap.begin()) {
            // Distance is smaller than our first entry
            motorSpeed = itHigh->second;
        } else if (itHigh == kFlywheelSpeedMap.end()) {
            // Distance is larger than our last entry
            motorSpeed = std::prev(itHigh)->second;
        } else {
            // Interpolate between prev and itHigh
            auto itLow = std::prev(itHigh);
            double d1 = itLow->first;
            double g1 = itLow->second;
            double d2 = itHigh->first;
            double g2 = itHigh->second;

            double t = (distVal - d1) / (d2 - d1);
            motorSpeed = g1 + t * (g2 - g1);
        }
    }
    return units::turns_per_second_t{motorSpeed};
}

void Turret_Shooter::Periodic()
{
    frc::SmartDashboard::PutNumber("/Turret/Shooter/Left Motor Voltage", m_leftMotor.GetMotorVoltage().GetValue().value());
    frc::SmartDashboard::PutNumber("/Turret/Shooter/Right Motor Voltage", m_rightMotor.GetMotorVoltage().GetValue().value());
    frc::SmartDashboard::PutNumber("/Turret/Shooter/Right Motor Temperature C", m_rightMotor.GetDeviceTemp().GetValueAsDouble());
    frc::SmartDashboard::PutNumber("/Turret/Shooter/Left Motor Temperature C", m_rightMotor.GetDeviceTemp().GetValueAsDouble());
    m_LeftMotorCurrentLog.Append(m_leftMotor.GetSupplyCurrent().GetValue().value());
    m_LeftMotorVoltageLog.Append(m_leftMotor.GetMotorVoltage().GetValue().value());
    m_RightMotorCurrentLog.Append(m_rightMotor.GetSupplyCurrent().GetValue().value());
    m_RightMotorVoltageLog.Append(m_rightMotor.GetMotorVoltage().GetValue().value());

    // left and right motors are same speed
    units::turns_per_second_t motorSpeed = m_leftMotor.GetVelocity().GetValue();
    // Using baseline gain for "Actual Ball Speed" calculation on dashboard
    double baselineGain = kFlyWheelGainMap.empty() ? 1.95 : kFlyWheelGainMap.begin()->second;
    units::meters_per_second_t ballSpeed = units::radians_per_second_t{motorSpeed} * (kFlywheelDiameter * kGearRatio) / (baselineGain * units::radian_t{1} * 4.0); 

    frc::SmartDashboard::PutNumber("/Turret/Shooter/Actual Motor RPS", motorSpeed.value());
    // frc::SmartDashboard::PutNumber("/Turret/Shooter/Actual Motor RPM", motorSpeed.value() * 60.0);
    frc::SmartDashboard::PutNumber("/Turret/Shooter/Actual Ball Speed MPS", ballSpeed.value()); 

    frc::SmartDashboard::PutNumber("/Turret/Spindexer/Actual Motor RPS", m_spindexerMotor.GetVelocity().GetValue().value());
    frc::SmartDashboard::PutNumber("/Turret/Spindexer/Commanded Motor RPS", Turret_ShooterConstants::kSpindexerShootVelocity.value());
    frc::SmartDashboard::PutNumber("/Turret/Indexer/Actual Motor RPM", m_indexerMotor.GetEncoder().GetVelocity());
    frc::SmartDashboard::PutNumber("/Turret/Indexer/Commanded Motor RPM", Turret_ShooterConstants::kIndexerShootVelocityRPM);
    m_SpindexerMotorCurrentLog.Append(m_spindexerMotor.GetSupplyCurrent().GetValue().value());
    m_SpindexerMotorVoltageLog.Append(m_spindexerMotor.GetMotorVoltage().GetValue().value());
    m_IndexerMotorCurrentLog.Append(m_indexerMotor.GetOutputCurrent());
    m_IndexerMotorVoltageLog.Append(m_indexerMotor.GetAppliedOutput() * m_indexerMotor.GetBusVoltage());

    bool override = frc::SmartDashboard::GetBoolean("/Turret/Shooter/Ball Speed Manual Override", false);
    if (override) {
        SetSpeed(units::meters_per_second_t{frc::SmartDashboard::GetNumber("/Turret/Shooter/Ball Speed Manual Set MPS", 0.0)}, 1.5_m);
    }

    if constexpr (frc::RobotBase::IsSimulation()) {
        SimulationPeriodic();
    }
}

void Turret_Shooter::SimulationPeriodic()
{
    units::second_t dt = m_simTimer.Get();
    m_simTimer.Reset();

    if (dt <= 0_s) {
        return;
    }

    const double alpha = std::clamp(dt.value() * 8.0, 0.0, 1.0);

    m_spindexerSimVelocity += (m_spindexerTargetVelocity - m_spindexerSimVelocity) * alpha;
    m_spindexerSimPosition += m_spindexerSimVelocity * dt;

    auto& spindexerSim = m_spindexerMotor.GetSimState();
    spindexerSim.SetSupplyVoltage(12_V);
    spindexerSim.SetRotorVelocity(m_spindexerSimVelocity);
    spindexerSim.SetRawRotorPosition(m_spindexerSimPosition);

    m_indexerSimVelocityRPM += (m_indexerTargetVelocityRPM - m_indexerSimVelocityRPM) * alpha;
    m_indexerSimPosition += (m_indexerSimVelocityRPM / 60.0) * dt.value();

    m_indexerSparkSim.SetBusVoltage(12.0);
    m_indexerSparkSim.SetVelocity(m_indexerSimVelocityRPM);
    m_indexerSparkSim.SetPosition(m_indexerSimPosition);
    m_indexerEncoderSim.SetVelocity(m_indexerSimVelocityRPM);
    m_indexerEncoderSim.SetPosition(m_indexerSimPosition);
}
