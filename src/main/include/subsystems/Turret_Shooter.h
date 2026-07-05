// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <rev/SparkFlex.h>
#include <rev/config/SparkFlexConfig.h>
#include <rev/SparkMax.h>
// #include <ctre/phoenix6/TalonFX.hpp>
#include <frc/motorcontrol/PWMMotorController.h>
#include <ctre/phoenix6/TalonFXS.hpp>
#include <ctre/phoenix6/TalonFX.hpp>
#include <ctre/phoenix6/controls/Follower.hpp>
#include <ctre/phoenix6/sim/TalonFXSimState.hpp>
#include <units/angle.h>
#include <units/velocity.h>
#include <units/acceleration.h>
#include <units/angular_velocity.h>
#include <units/angular_acceleration.h>
#include <units/voltage.h>
#include <units/moment_of_inertia.h>
#include <map>

#include <frc/DriverStation.h>

#include <rev/SparkBase.h>
#include <frc2/command/PIDCommand.h>
#include <frc2/command/PIDSubsystem.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/controller/SimpleMotorFeedforward.h>
#include <frc/system/plant/DCMotor.h>
#include <frc/Timer.h>
#include <frc/RobotBase.h>
#include <rev/SparkSim.h>
#include <frc/DataLogManager.h>
#include <wpi/DataLog.h>


namespace Turret_ShooterConstants {

  static const int kMotorIdLeft = 4;
  static const int kMotorIdRight = 3;

  static const int kSpindexerMotorId = 5;
  static const int kIndexerMotorId = 6;

  static const double spindexerSpeed = 0.2;
  const double kPercentBoost = 0.0;

  const double kMinLaunchRPS = 2000;

  static constexpr double kSpindexerP = 0.5;
  static constexpr double kSpindexerI = 0.0;
  static constexpr double kSpindexerD = 0.0;
  static constexpr double kSpindexerS = 0.1;
  static constexpr double kSpindexerV = 0.2;

  static constexpr double kIndexerP = 0.0001;
  static constexpr double kIndexerI = 0.0;
  static constexpr double kIndexerD = 0.0;
  static constexpr double kIndexerkV = 0.00015;

  // Spindexer operates on a 5:1 gearbox. 
  // Native RPS is measured at the motor.
  static constexpr units::turns_per_second_t kSpindexerShootVelocity = -27_tps;
  static constexpr double kIndexerShootVelocityRPM = -6000;
}

class Turret_Shooter : public frc2::SubsystemBase
{
public:
  Turret_Shooter();

  void Periodic() override;
  void SimulationPeriodic() override;

  void StopMotors();
  void SetSpeed(units::meters_per_second_t speed, units::meter_t distance); // speed of the ball leaving the shooter
  units::meters_per_second_t GetActualBallSpeed();
  units::turns_per_second_t GetActualMotorSpeed();
  units::turns_per_second_t ConvertBallSpeed2Motor(units::meters_per_second_t ballSpeed, units::meter_t distance);
  units::turns_per_second_t GetMotorSpeedFromMap(units::meter_t distance);
  void SetMotorSpeed(units::turns_per_second_t speed);

  void RunSpindexerIndexer(units::meter_t distance);
  void StopSpindexerIndexer();
  void ChangeSpeedMapValue(double newOffsetValue);
  void TurnOffMotors()
  {
    auto request = ctre::phoenix6::controls::VoltageOut{0.0_V};
    m_leftMotor.SetControl(request);
    m_rightMotor.SetControl(request);
  };
  
  std::map<double, double> GetCurrentMapState();
  void SetCurrentMapState(std::map<double, double> inputCurrentState); // Should only be used when saving!! Please do not write to the maps unless you know for certain this is what you want to do!

  void RunAll();
  void StopAll();

private:
  ctre::phoenix6::hardware::TalonFX m_leftMotor{Turret_ShooterConstants::kMotorIdLeft};
  ctre::phoenix6::hardware::TalonFX m_rightMotor{Turret_ShooterConstants::kMotorIdRight};

  ctre::phoenix6::hardware::TalonFX m_spindexerMotor{Turret_ShooterConstants::kSpindexerMotorId};
  rev::spark::SparkFlex m_indexerMotor{Turret_ShooterConstants::kIndexerMotorId, rev::spark::SparkFlex::MotorType::kBrushless};
  rev::spark::SparkClosedLoopController m_indexerPID{m_indexerMotor.GetClosedLoopController()};

  static constexpr units::inch_t kFlywheelDiameter = units::inch_t{2.625};
  static constexpr int kGearRatio = 2;
  static constexpr units::inch_t kBallDiameter = units::inch_t{5.91};

  double kP = 0.37;
  double kI = 0.0;
  double kD = 0.0;
  double kS = 0.6;
  double kA = 0.005;
  double kV = 0.2;

  // determines how much faster the flywheel needs to spin
  // so that the exit velocity meets the specified speed 
  // Map of Distance (meters) to Multiplier Gain
  std::map<double, double> kFlyWheelGainMap = {
      {1.5, 1.4},
      {2.0, 1.75},
      {2.5, 2.25}, // 
      {3.0, 2.5}, // 
      {4.0, 2.5}, // up
      {5.0, 2.8},
      {6.0, 3},
      {7.0, 3.2}
  };

  std::map<double, double> kFlywheelSpeedMap = {
      {1.0, 35.},
      {1.5, 42.},
      {2.0, 45.},
      {2.5, 50.},
      {3.0, 60.},
      {3.5, 78.}, //8 deg extra
      {4.0, 83.},
      {4.5, 88.}, //10 deg extra
      {5.0, 90},
      {5.5, 95.},
      {6.0, 100.},
      {6.5, 105.},
      {7.0, 110.}
  };
  // std::map<double, double> kFlywheelSpeedMap;

  


  bool kEnableCurrentLimit = true;
  units::ampere_t kPeakCurrentLimit = units::ampere_t{40};
  units::ampere_t kContinousCurrentLimit = units::ampere_t{35};
  units::second_t kPeakCurrentDuration = units::second_t{0.1};

  frc::Timer m_simTimer;
  units::turn_t m_spindexerSimPosition{0_tr};
  units::turns_per_second_t m_spindexerSimVelocity{0_tps};
  units::turns_per_second_t m_spindexerTargetVelocity{0_tps};

  wpi::log::DoubleLogEntry m_LeftMotorCurrentLog;
  wpi::log::DoubleLogEntry m_LeftMotorVoltageLog;
  wpi::log::DoubleLogEntry m_RightMotorCurrentLog;
  wpi::log::DoubleLogEntry m_RightMotorVoltageLog;
  wpi::log::DoubleLogEntry m_SpindexerMotorCurrentLog;
  wpi::log::DoubleLogEntry m_SpindexerMotorVoltageLog;
  wpi::log::DoubleLogEntry m_IndexerMotorCurrentLog;
  wpi::log::DoubleLogEntry m_IndexerMotorVoltageLog;

  frc::DCMotor m_indexerSimMotor = frc::DCMotor::NEO(1);
  rev::spark::SparkSim m_indexerSparkSim{&m_indexerMotor, &m_indexerSimMotor};
  rev::spark::SparkRelativeEncoderSim m_indexerEncoderSim{m_indexerSparkSim.GetRelativeEncoderSim()};
  double m_indexerSimPosition{0.0};
  double m_indexerSimVelocityRPM{0.0};
  double m_indexerTargetVelocityRPM{0.0};
};
