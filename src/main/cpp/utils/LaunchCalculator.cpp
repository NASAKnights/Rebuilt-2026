#include "utils/LaunchCalculator.h"
#include "utils/DeployFileUtils.h"

#include <cmath>
#include <algorithm>

LaunchCalculator::LaunchCalculator()
    : m_ntMap("LaunchCalculator/Points",
              "Distance",
              std::array<std::string, 3>{"HoodAngle", "FlywheelRps", "TOF"},
              utils::DeployFileUtils::ResolveDeployFilePathFromNtPath("LaunchCalculator/Points").string()) {}

LaunchCalculator::LaunchingParameters LaunchCalculator::CalculateParameters(
    const frc::Pose2d& robotPose,
    const frc::Translation2d& target,
    units::meters_per_second_t robotVxRobotFrame,
    units::meters_per_second_t robotVyRobotFrame,
    units::radians_per_second_t robotOmega,
    units::second_t loopPeriod) {
  frc::Pose2d estimatedPose = robotPose.Exp(frc::Twist2d{
      units::meter_t{robotVxRobotFrame.value() * m_phaseDelaySec},
      units::meter_t{robotVyRobotFrame.value() * m_phaseDelaySec},
      units::radian_t{robotOmega.value() * m_phaseDelaySec}});

  frc::Pose2d turretPose = estimatedPose.TransformBy(
      frc::Transform2d{frc::Translation2d{units::meter_t{m_robotToTurretX}, units::meter_t{m_robotToTurretY}},
                       frc::Rotation2d{}});

  const double robotAngle = estimatedPose.Rotation().Radians().value();
  const double fieldVx = robotVxRobotFrame.value() * std::cos(robotAngle) -
                         robotVyRobotFrame.value() * std::sin(robotAngle);
  const double fieldVy = robotVxRobotFrame.value() * std::sin(robotAngle) +
                         robotVyRobotFrame.value() * std::cos(robotAngle);

  const double turretVelocityX = fieldVx + robotOmega.value() *
                                               (m_robotToTurretY * std::cos(robotAngle) -
                                                m_robotToTurretX * std::sin(robotAngle));
  const double turretVelocityY = fieldVy + robotOmega.value() *
                                               (m_robotToTurretX * std::cos(robotAngle) -
                                                m_robotToTurretY * std::sin(robotAngle));

  frc::Pose2d lookaheadPose = turretPose;
  units::meter_t lookaheadDistance = target.Distance(turretPose.Translation());
  double tofSec = GetTOF(lookaheadDistance.value());

  for (int i = 0; i < 20; ++i) {
    tofSec = GetTOF(lookaheadDistance.value());
    const units::meter_t offsetX{turretVelocityX * tofSec};
    const units::meter_t offsetY{turretVelocityY * tofSec};
    lookaheadPose = frc::Pose2d{turretPose.Translation() + frc::Translation2d{offsetX, offsetY},
                                turretPose.Rotation()};
    lookaheadDistance = target.Distance(lookaheadPose.Translation());
  }

  frc::Rotation2d turretAngle = (target - lookaheadPose.Translation()).Angle();
  const bool hasLaunchMap = !m_ntMap.GetMap().empty();
  const auto shot = GetShotForDistance(lookaheadDistance);
  const double hoodAngleDeg = shot.hoodAngleDeg;

  if (!m_lastTurretAngle.has_value()) {
    m_lastTurretAngle = turretAngle;
  }
  if (!m_lastHoodAngleDeg.has_value()) {
    m_lastHoodAngleDeg = hoodAngleDeg;
  }

  const double rawTurretVelocity = (turretAngle - *m_lastTurretAngle).Radians().value() /
                                   std::max(1e-6, loopPeriod.value());
  const double rawHoodVelocity = (hoodAngleDeg - *m_lastHoodAngleDeg) /
                                 std::max(1e-6, loopPeriod.value());

  constexpr size_t kWindowSize = 5;
  m_turretVelocityWindow.push_back(rawTurretVelocity);
  if (m_turretVelocityWindow.size() > kWindowSize) m_turretVelocityWindow.pop_front();
  m_hoodVelocityWindow.push_back(rawHoodVelocity);
  if (m_hoodVelocityWindow.size() > kWindowSize) m_hoodVelocityWindow.pop_front();

  auto mean = [](const std::deque<double>& values) {
    if (values.empty()) return 0.0;
    double sum = 0.0;
    for (double v : values) sum += v;
    return sum / static_cast<double>(values.size());
  };

  m_lastTurretAngle = turretAngle;
  m_lastHoodAngleDeg = hoodAngleDeg;

  return LaunchingParameters{
      .isValid = hasLaunchMap &&
                 lookaheadDistance.value() >= m_minDistanceMeters &&
                 lookaheadDistance.value() <= m_maxDistanceMeters,
      .turretAngle = turretAngle,
      .turretVelocityRadPerSec = mean(m_turretVelocityWindow),
      .hoodAngleDeg = hoodAngleDeg,
      .hoodVelocityDegPerSec = mean(m_hoodVelocityWindow),
      .flywheelRps = shot.flywheelRps,
      .lookaheadDistance = lookaheadDistance,
      .timeOfFlightSec = tofSec};
}

double LaunchCalculator::InterpolateFromMap(const std::map<double, double>& map, double key) {
  if (map.empty()) {
    return 0.0;
  }

  auto itHigh = map.lower_bound(key);
  if (itHigh == map.begin()) {
    return itHigh->second;
  }
  if (itHigh == map.end()) {
    return std::prev(itHigh)->second;
  }

  auto itLow = std::prev(itHigh);
  const double x1 = itLow->first;
  const double y1 = itLow->second;
  const double x2 = itHigh->first;
  const double y2 = itHigh->second;

  const double t = (key - x1) / (x2 - x1);
  return y1 + t * (y2 - y1);
}

LaunchCalculator::ShotPoint LaunchCalculator::GetShotForDistance(units::meter_t distance) const {
  std::map<double, double> hood;
  std::map<double, double> flywheel;
  std::map<double, double> tof;

  auto currentMap = m_ntMap.GetMap();
  for (const auto& [d, values] : currentMap) {
    hood[d] = std::get<0>(values);
    flywheel[d] = std::get<1>(values);
    tof[d] = std::get<2>(values);
  }

  const double distanceMeters = distance.value();
  return ShotPoint{
      .hoodAngleDeg = InterpolateFromMap(hood, distanceMeters),
      .flywheelRps = InterpolateFromMap(flywheel, distanceMeters),
      .tofSec = InterpolateFromMap(tof, distanceMeters),
  };
}

double LaunchCalculator::GetTOF(double distanceMeters) const {
  return GetShotForDistance(units::meter_t{distanceMeters}).tofSec;
}

units::meter_t LaunchCalculator::GetDistanceFromTOF(double tofSec) const {
  std::map<double, double> tofToDistance;
  auto currentMap = m_ntMap.GetMap();
  for (const auto& [distance, values] : currentMap) {
    tofToDistance[std::get<2>(values)] = distance;
  }
  return units::meter_t{InterpolateFromMap(tofToDistance, tofSec)};
}

double LaunchCalculator::GetHoodAngleDegrees(double distanceMeters) const {
  return GetShotForDistance(units::meter_t{distanceMeters}).hoodAngleDeg;
}

units::turns_per_second_t LaunchCalculator::GetFlywheelSpeed(units::meter_t distance) const {
  return units::turns_per_second_t{GetShotForDistance(distance).flywheelRps};
}

double LaunchCalculator::CompensateTOFForRobotMotion(
    double initialTof,
    const frc::Pose2d& robotPose,
    const frc::Transform3d& goal,
    units::meters_per_second_t robotVxRobotFrame,
    units::meters_per_second_t robotVyRobotFrame,
    int iterations) const {
  double tn = initialTof;

  const units::radian_t robotAngle = robotPose.Rotation().Radians();
  const double robotVx = robotVxRobotFrame.value();
  const double robotVy = robotVyRobotFrame.value();

  const double robotWorldVx = robotVx * std::cos(robotAngle.value()) - robotVy * std::sin(robotAngle.value());
  const double robotWorldVy = robotVx * std::sin(robotAngle.value()) + robotVy * std::cos(robotAngle.value());

  for (int i = 0; i < iterations; ++i) {
    const double dx = robotPose.X().value() - goal.X().value();
    const double dy = robotPose.Y().value() - goal.Y().value();

    const double distanceToTarget = GetDistanceFromTOF(tn).value();
    const double vp = (std::abs(tn) > 1e-6) ? (distanceToTarget / tn) : 0.0;

    const double E = tn - GetTOF(distanceToTarget);
    const double dE = 1.0 + ((dx * robotWorldVx + dy * robotWorldVy) /
                              (std::max(1e-6, vp * std::max(1e-6, distanceToTarget))));

    tn -= (E / dE);
    if (std::abs(E) < 1e-3) {
      break;
    }
  }

  return tn;
}

std::map<double, double> LaunchCalculator::GetHoodAngleMap() const {
  std::map<double, double> hood;
  auto currentMap = m_ntMap.GetMap();
  for (const auto& [distance, values] : currentMap) {
    hood[distance] = std::get<0>(values);
  }
  return hood;
}

void LaunchCalculator::SetHoodAngleMap(const std::map<double, double>& hoodAngleMapDeg) {
  auto currentMap = m_ntMap.GetMap();
  for (const auto& [distance, angleDeg] : hoodAngleMapDeg) {
    auto it = currentMap.find(distance);
    if (it != currentMap.end()) {
      auto values = it->second;
      std::get<0>(values) = angleDeg;
      m_ntMap.Set(distance, values);
    }
  }
}

void LaunchCalculator::AdjustHoodAngleAtDistance(double distanceMeters, double deltaDeg) {
  auto currentMap = m_ntMap.GetMap();
  if (currentMap.empty()) {
    return;
  }

  auto itHigh = currentMap.lower_bound(distanceMeters);
  if (itHigh == currentMap.end()) {
    itHigh = std::prev(currentMap.end());
  }

  if (itHigh != currentMap.end()) {
    auto values = itHigh->second;
    std::get<0>(values) += deltaDeg;
    m_ntMap.Set(itHigh->first, values);
  }
}

void LaunchCalculator::UpdateFromNetworkTables() const {
  m_ntMap.UpdateFromNetworkTables();
}

void LaunchCalculator::PublishCurrentTable() const {
  m_ntMap.PublishCurrentTable();
}

void LaunchCalculator::SaveToFile() const {
  m_ntMap.SaveToFile();
}
