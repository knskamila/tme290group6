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
#include <math.h>

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

  DEFAULT_SPEED = GIVEN_SPEED;
  if(frontDistance < SLOW_DISTANCE) DEFAULT_SPEED = GIVEN_SPEED - SPEED_DIFFERENCE;

  speedUp(); //default speed adjustment
  randomTurn(rightDistance, leftDistance); //decides random direction if area is free

  if (frontDistance < 0.17f)
  {
    backing = true;
  }
  else if (rearDistance < 0.15f)
  {
    speedUp();
  }
  else if (frontDistance < 0.4f || rightDistance < 0.4f || leftDistance < 0.4f)
  {
    avoiding = true;
    turningCount = 0;
  }

  backOut(); //if needed, the direction and angle are reversed; has a priority

  //if (frontDistance < 0.03f)
  //{
  //   pedalPosition = 0.0f;
  //   groundSteeringAngle = 0.0f;
  //}
 
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

  double distance = -0.0256*pow(voltage, 5) + 0.2504*pow(voltage, 4) - 0.9545*pow(voltage, 3) + 1.7879*pow(voltage, 2) - 1.7121*voltage + 0.7788;
  //double distance = 0.00372 * sensorVoltage * sensorVoltage - 0.21730 * sensorVoltage + 3.34464;
  return distance;

}

void Behavior::randomTurn(double right, double left) noexcept
{
  //either continues to turn, stops turning or checks if it should turn randomly
  if(avoiding && turningCount == 0)
  {
    if(left < right) turningDirection = RIGHT;
    else turningDirection = LEFT;
  }
  if(avoiding && turningCount < 20)
  {
    if (turningDirection) groundSteeringAngle = 0.45f;
    else groundSteeringAngle = -0.45f;
    turningCount++;
  }
  else if(avoiding) avoiding = false;
  else
  {
    srand(time(0));
    int r = rand();
    if (turning) {
      if (turningDirection) groundSteeringAngle = 0.3f;
      else groundSteeringAngle = -0.3f;
    } else {
      groundSteeringAngle = 0.0f;
    }
    turningCount++;
    if (turningCount > 40) {
      turning = false;
      turningCount = 0;
      groundSteeringAngle = 0.0f;
      if (r % 2 == 0) turningDirection = LEFT;
      else turningDirection = RIGHT;
    }
    if (r % 10 == 0) turning = true;
  }
}

void Behavior::backOut() noexcept
{
  //either continues to back out or stops
  if(backing)
  {
    pedalPosition = DEFAULT_BACKWARDS;
    if (turningDirection) groundSteeringAngle = -0.45f;
    else groundSteeringAngle = 0.45f;
  }
  if(backingCount > 10)
  {
    backing = false;
    backingCount = 0;
  }
  backingCount++;
}

void Behavior::speedUp() noexcept
{
  if(pedalPosition < DEFAULT_SPEED)
  {
     if(pedalPosition < DEFAULT_SPEED - STEP)
     {
	    pedalPosition += STEP;
     }
     else
     {
        pedalPosition = DEFAULT_SPEED;
     }
  }

  else pedalPosition = DEFAULT_SPEED;

}

void Behavior::setSpeed(float forward, float backwards, float difference, float slow) noexcept
{
   GIVEN_SPEED = forward;
   DEFAULT_BACKWARDS = backwards;
   SPEED_DIFFERENCE = difference;
   SLOW_DISTANCE = slow;
}
