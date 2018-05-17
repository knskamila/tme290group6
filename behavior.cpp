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

#include "behavior.hpp"
#include <time.h>
#include <math.h>
#include <cmath>
#include <iostream>

Behavior::Behavior() noexcept:
  m_frontUltrasonicReading{},
  m_rearUltrasonicReading{},
  m_leftIrReading{},
  m_rightIrReading{},
  m_groundSteeringAngleRequest{},
  m_pedalPositionRequest{},
  m_frontUltrasonicReadingMutex{},
  m_rearUltrasonicReadingMutex{},
  m_leftIrReadingMutex{},
  m_rightIrReadingMutex{},
  m_groundSteeringAngleRequestMutex{},
  m_pedalPositionRequestMutex{},
  m_positionMutex{}
{
}

opendlv::proxy::GroundSteeringRequest Behavior::getGroundSteeringAngle() noexcept
{
  std::lock_guard<std::mutex> lock(m_groundSteeringAngleRequestMutex);
  return m_groundSteeringAngleRequest;
}

opendlv::proxy::PedalPositionRequest Behavior::getPedalPositionRequest() noexcept
{
  std::lock_guard<std::mutex> lock(m_pedalPositionRequestMutex);
  return m_pedalPositionRequest;
}

void Behavior::setFrontUltrasonic(opendlv::proxy::DistanceReading const &frontUltrasonicReading) noexcept
{
  std::lock_guard<std::mutex> lock(m_frontUltrasonicReadingMutex);
  m_frontUltrasonicReading = frontUltrasonicReading;
}

void Behavior::setRearUltrasonic(opendlv::proxy::DistanceReading const &rearUltrasonicReading) noexcept
{
  std::lock_guard<std::mutex> lock(m_rearUltrasonicReadingMutex);
  m_rearUltrasonicReading = rearUltrasonicReading;
}

void Behavior::setLeftIr(opendlv::proxy::VoltageReading const &leftIrReading) noexcept
{
  std::lock_guard<std::mutex> lock(m_leftIrReadingMutex);
  m_leftIrReading = leftIrReading;
}

void Behavior::setRightIr(opendlv::proxy::VoltageReading const &rightIrReading) noexcept
{
  std::lock_guard<std::mutex> lock(m_rightIrReadingMutex);
  m_rightIrReading = rightIrReading;
}

void Behavior::setPos(opendlv::sim::Frame const &posReading) noexcept
{
  std::lock_guard<std::mutex> lock(m_positionMutex);
  m_position = posReading;
}


void Behavior::step() noexcept
{
  opendlv::proxy::DistanceReading frontUltrasonicReading;
  opendlv::proxy::DistanceReading rearUltrasonicReading;
  opendlv::proxy::VoltageReading leftIrReading;
  opendlv::proxy::VoltageReading rightIrReading;
  opendlv::sim::Frame posReading;

  {
    std::lock_guard<std::mutex> lock1(m_frontUltrasonicReadingMutex);
    std::lock_guard<std::mutex> lock2(m_rearUltrasonicReadingMutex);
    std::lock_guard<std::mutex> lock3(m_leftIrReadingMutex);
    std::lock_guard<std::mutex> lock4(m_rightIrReadingMutex);
    std::lock_guard<std::mutex> lock5(m_positionMutex);

    frontUltrasonicReading = m_frontUltrasonicReading;
    rearUltrasonicReading = m_rearUltrasonicReading;
    leftIrReading = m_leftIrReading;
    rightIrReading = m_rightIrReading;
    posReading = m_position;
  }

  float frontDistance = frontUltrasonicReading.distance();
  float rearDistance = rearUltrasonicReading.distance();
  double leftDistance = convertIrVoltageToDistance(leftIrReading.voltage());
  double rightDistance = convertIrVoltageToDistance(rightIrReading.voltage());

  double xp_prev = xp;
  double yp_prev = yp;
  double heading_prev = heading;

  //-----------------------------ADDING NOISE-------------
  xp = posReading.x();
  yp = posReading.y();
  heading = posReading.yaw();

  xp = randomNoise(xp, 0.05);
  yp = randomNoise(yp, 0.05);
  heading = randomNoise(heading, 0.05);

  xp = averageValue(xp, xp_prev, 0.7);
  yp = averageValue(yp, yp_prev, 0.7);
  heading = averageValue(heading, heading_prev, 0.7);

  //------------------------------------------------------

  double xGoal = path.front().first;
  double yGoal = path.front().second;

  double desiredHeading = std::atan2(yGoal - yp, xGoal - xp);

  if(heading < 0 && desiredHeading > 0 && (abs(heading) + abs(desiredHeading)) > PI) groundSteeringAngle = 0.9f*(float)(desiredHeading + heading);
  else if(heading > 0 && desiredHeading < 0 && (abs(heading) + abs(desiredHeading)) > PI) groundSteeringAngle = 0.9f*(float)(desiredHeading + heading);
  else groundSteeringAngle = 0.9f*(float)(desiredHeading - heading);

  pedalPosition = DEFAULT_SPEED;

  if(reached(xp, yp, xGoal, yGoal) && path.size() > 0)
  {
      std::cout << "point passed" << std::endl;
      path.pop_front();
      if(path.size() > 0)
      {
          xGoal = path.front().first;
          yGoal = path.front().second;
          groundSteeringAngle = 0.0f;
      }
      else
      {
          std::cout << "path finished" << std::endl;
          xGoal = 10000;
          yGoal = 10000;
          pedalPosition = 0.0;
          groundSteeringAngle = 0.0f;
      }
  }

  if(path.size() == 0)
  {
      pedalPosition = 0.0;
      groundSteeringAngle = 0.0f;
  }

  if (frontDistance < 0.15f || rearDistance < 0.1f || rightDistance < 0.1f || leftDistance < 0.1f)
  {
      pedalPosition = 0.0f;
      groundSteeringAngle = 0.0f;
  }

 
  {
      std::lock_guard<std::mutex> lock1(m_groundSteeringAngleRequestMutex);
      std::lock_guard<std::mutex> lock2(m_pedalPositionRequestMutex);

      opendlv::proxy::GroundSteeringRequest groundSteeringAngleRequest;
      groundSteeringAngleRequest.groundSteering(groundSteeringAngle);
      m_groundSteeringAngleRequest = groundSteeringAngleRequest;

      opendlv::proxy::PedalPositionRequest pedalPositionRequest;
      pedalPositionRequest.position(pedalPosition);
      m_pedalPositionRequest = pedalPositionRequest;
  }
}

double Behavior::convertIrVoltageToDistance(float voltage) const noexcept
{
    double voltageDividerR1 = 1000.0;
    double voltageDividerR2 = 1000.0;

    double sensorVoltage = (voltageDividerR1 + voltageDividerR2) / voltageDividerR2 * voltage;
    double distance = (2.5 - sensorVoltage) / 0.07;
    return distance;
}

void Behavior::setGoal(std::list<std::pair<float,float>> p, double x, double y, double h) noexcept
{
    path = p;
    xp = x;
    yp = y;
    heading = h;
}

bool Behavior::reached(double xp, double yp, double xGoal, double yGoal) noexcept
{
    if(abs(xp - xGoal) < 0.15 && abs(yp - yGoal) < 0.15) return 1;
    else return 0;
}

double Behavior::randomNoise(double value, double range) noexcept
{
    std::normal_distribution<double> g(value,range);
    std::default_random_engine re(time(0));
    return g(re);
}

double Behavior::averageValue(double value, double previous, double gain) noexcept
{
    return gain*value + (1-gain)*previous;
}
