#pragma once

#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/geometry/Transform3d.h>
#include <frc/geometry/Translation2d.h>
#include <frc/geometry/Twist2d.h>
#include <array>
#include <deque>
#include <units/length.h>
#include <units/angular_velocity.h>
#include <units/time.h>
#include <units/velocity.h>

#include <map>
#include <optional>

#include "utils/NetworkTableMap.h"

class LaunchCalculator {
 public:
  struct LaunchingParameters {
    bool isValid;
    frc::Rotation2d turretAngle;
    double turretVelocityRadPerSec;
    double hoodAngleDeg;
    double hoodVelocityDegPerSec;
    double flywheelRps;
    units::meter_t lookaheadDistance;
    double timeOfFlightSec;
  };

  struct ShotPoint {
    double hoodAngleDeg;
    double flywheelRps;
    double tofSec;
  };

  LaunchCalculator();

  LaunchingParameters CalculateParameters(
      const frc::Pose2d& robotPose,
      const frc::Translation2d& target,
      units::meters_per_second_t robotVxRobotFrame,
      units::meters_per_second_t robotVyRobotFrame,
      units::radians_per_second_t robotOmega,
      units::second_t loopPeriod = 20_ms);

  ShotPoint GetShotForDistance(units::meter_t distance) const;
  double GetTOF(double distanceMeters) const;
  units::meter_t GetDistanceFromTOF(double tofSec) const;
  double GetHoodAngleDegrees(double distanceMeters) const;
  units::turns_per_second_t GetFlywheelSpeed(units::meter_t distance) const;

  // Newton update for robot-motion compensation of flight time.
  double CompensateTOFForRobotMotion(double initialTof,
                                     const frc::Pose2d& robotPose,
                                     const frc::Transform3d& goal,
                                     units::meters_per_second_t robotVxRobotFrame,
                                     units::meters_per_second_t robotVyRobotFrame,
                                     int iterations = 5) const;

  std::map<double, double> GetHoodAngleMap() const;
  void SetHoodAngleMap(const std::map<double, double>& hoodAngleMapDeg);
  void AdjustHoodAngleAtDistance(double distanceMeters, double deltaDeg);

  void UpdateFromNetworkTables() const;
  void PublishCurrentTable() const;
  void SaveToFile() const;

 private:
  static double InterpolateFromMap(const std::map<double, double>& map, double key);

  double m_minDistanceMeters = 1.0;
  double m_maxDistanceMeters = 7.0;
  double m_phaseDelaySec = 0.03;
  double m_robotToTurretX = -0.181;
  double m_robotToTurretY = 0.18;

  std::optional<frc::Rotation2d> m_lastTurretAngle;
  std::optional<double> m_lastHoodAngleDeg;
  std::deque<double> m_turretVelocityWindow;
  std::deque<double> m_hoodVelocityWindow;

  NetworkTableMap<double, double, double, double> m_ntMap;
};
