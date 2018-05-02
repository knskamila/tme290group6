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
  m_pedalPositionRequestMutex{}
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


void Behavior::step() noexcept
{
  opendlv::proxy::DistanceReading frontUltrasonicReading;
  opendlv::proxy::DistanceReading rearUltrasonicReading;
  opendlv::proxy::VoltageReading leftIrReading;
  opendlv::proxy::VoltageReading rightIrReading;
  {
    std::lock_guard<std::mutex> lock1(m_frontUltrasonicReadingMutex);
    std::lock_guard<std::mutex> lock2(m_rearUltrasonicReadingMutex);
    std::lock_guard<std::mutex> lock3(m_leftIrReadingMutex);
    std::lock_guard<std::mutex> lock4(m_rightIrReadingMutex);

    frontUltrasonicReading = m_frontUltrasonicReading;
    rearUltrasonicReading = m_rearUltrasonicReading;
    leftIrReading = m_leftIrReading;
    rightIrReading = m_rightIrReading;
  }

  float frontDistance = frontUltrasonicReading.distance();
  float rearDistance = rearUltrasonicReading.distance();
  double leftDistance = convertIrVoltageToDistance(leftIrReading.voltage());
  double rightDistance = convertIrVoltageToDistance(rightIrReading.voltage());

  speedUp(); //default speed adjustment

  if (frontDistance < 0.2f)
  {
    pedalPosition = -0.2f;
    groundSteeringAngle = 0.2f;
  }
  else if (rearDistance < 0.3f)
  {
    speedUp();
  }
  else if (frontDistance < 0.35f || rightDistance < 0.25f || leftDistance < 0.25f)
  {
     turn(0.4f, rightDistance, leftDistance);
  }
  else //if no walls are nearby, perform random behavior
  {
     randomTurn();
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

// TODO: This is a rough estimate, improve by looking into the sensor specifications.
double Behavior::convertIrVoltageToDistance(float voltage) const noexcept
{
  double voltageDividerR1 = 1000.0;
  double voltageDividerR2 = 1000.0;

  double sensorVoltage = (voltageDividerR1 + voltageDividerR2) / voltageDividerR2 * voltage;
  //double distance = (2.5 - sensorVoltage) / 0.07;
  double distance = 0.00372 * sensorVoltage * sensorVoltage - 0.21730 * sensorVoltage + 3.34464;
  return distance;



}

void Behavior::turn(float value, double right, double left) noexcept
{
  
  if (left < 0.3f)
  {
     groundSteeringAngle = -value;
  }
  else if (right < 0.3f) //redundant, but for now...
  {
     groundSteeringAngle = value;
  }
  else
  {
     groundSteeringAngle = value;
  }
}

void Behavior::randomTurn() noexcept
{
  //either continues to turn, stops turning or checks if it should turn randomly
  int r = rand();
  if(turning)
  {
    if(turningDirection) groundSteeringAngle = 0.3f;
    else groundSteeringAngle = -0.3f;
  }
  else
  {
    groundSteeringAngle = 0.0f;
  }
  turningCount++;
  if(turningCount > 40)
  {
    turning = false;
    turningCount = 0;
    groundSteeringAngle = 0.0f;
    if(r % 2 == 0) turningDirection = 1;
    else turningDirection = 0;
  }
  if(r % 10 == 0)
  {
    turning = true;
  }
}

void Behavior::speedUp() noexcept
{
  if (pedalPosition < DEFAULT_SPEED)
  {
     if (pedalPosition < DEFAULT_SPEED - STEP)
     {
	pedalPosition += STEP;
     }
     else
     {
        pedalPosition = DEFAULT_SPEED;
     }
  }

}
