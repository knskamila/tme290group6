/*
 * Copyright (C) 2018 Ola Benderius
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef BEHAVIOR
#define BEHAVIOR

#include <mutex>
#include <list>
#include <random>
#include "opendlv-standard-message-set.hpp"

#define PI 3.14159

class Behavior {
 private:
  Behavior(Behavior const &) = delete;
  Behavior(Behavior &&) = delete;
  Behavior &operator=(Behavior const &) = delete;
  Behavior &operator=(Behavior &&) = delete;

 public:
  Behavior() noexcept;
  ~Behavior() = default;

 public:
  opendlv::proxy::GroundSteeringRequest getGroundSteeringAngle() noexcept;
  opendlv::proxy::PedalPositionRequest getPedalPositionRequest() noexcept;
  void setFrontUltrasonic(opendlv::proxy::DistanceReading const &) noexcept;
  void setRearUltrasonic(opendlv::proxy::DistanceReading const &) noexcept;
  void setLeftIr(opendlv::proxy::VoltageReading const &) noexcept;
  void setRightIr(opendlv::proxy::VoltageReading const &) noexcept;
  void setPos(opendlv::sim::Frame const &) noexcept;
  void step() noexcept;
  void setGoal(std::list<std::pair<float,float>>, double, double, double) noexcept;
  bool reached(double, double, double, double) noexcept;
  double randomNoise(double, double) noexcept;
  double averageValue(double, double, double) noexcept;

 private:
  double convertIrVoltageToDistance(float) const noexcept;

 private:
  opendlv::proxy::DistanceReading m_frontUltrasonicReading;
  opendlv::proxy::DistanceReading m_rearUltrasonicReading;
  opendlv::proxy::VoltageReading m_leftIrReading;
  opendlv::proxy::VoltageReading m_rightIrReading;
  opendlv::proxy::GroundSteeringRequest m_groundSteeringAngleRequest;
  opendlv::proxy::PedalPositionRequest m_pedalPositionRequest;
  opendlv::sim::Frame m_position = opendlv::sim::Frame();
  std::mutex m_frontUltrasonicReadingMutex;
  std::mutex m_rearUltrasonicReadingMutex;
  std::mutex m_leftIrReadingMutex;
  std::mutex m_rightIrReadingMutex;
  std::mutex m_groundSteeringAngleRequestMutex;
  std::mutex m_pedalPositionRequestMutex;
  std::mutex m_positionMutex;
  std::list<std::pair<float,float>> path = std::list<std::pair<float,float>>();
  const float DEFAULT_SPEED = 0.2f;
  float pedalPosition = 0.0f;
  float groundSteeringAngle = 0.0f;
  double xp = 0;
  double yp = 0;
  double heading = 0;
};

#endif
