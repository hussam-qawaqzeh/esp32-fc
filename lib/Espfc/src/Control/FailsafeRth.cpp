#include "Control/FailsafeRth.h"
#include "Utils/Math.hpp"
#include <Arduino.h>
#include <algorithm>
#include <cmath>

namespace Espfc {
namespace Control {

namespace {

float wrapAngle180(float angle)
{
  while(angle > 180.f) angle -= 360.f;
  while(angle < -180.f) angle += 360.f;
  return angle;
}

float wrapAngle360(float angle)
{
  while(angle >= 360.f) angle -= 360.f;
  while(angle < 0.f) angle += 360.f;
  return angle;
}

float blendAngle(float current, float target, float alpha)
{
  return wrapAngle360(current + wrapAngle180(target - current) * Utils::clamp(alpha, 0.f, 1.f));
}

float normalizedToPwm(float value)
{
  return FAILSAFE_PWM_MID + Utils::clamp(value, -1.f, 1.f) * ((FAILSAFE_PWM_MAX - FAILSAFE_PWM_MIN) * 0.5f);
}

float altHoldRateToInput(float rate)
{
  float input = 0.f;
  if(rate < 0.f)
  {
    input = Utils::map(Utils::clamp(rate, -2.f, 0.f), -2.f, 0.f, -1.f, 0.f);
  }
  else if(rate > 0.f)
  {
    input = Utils::map(Utils::clamp(rate, 0.f, 4.f), 0.f, 4.f, 0.f, 1.f);
  }

  if(input > 0.f) input = Utils::clamp(input + 0.1f, 0.f, 1.f);
  if(input < 0.f) input = Utils::clamp(input - 0.1f, -1.f, 0.f);

  return input;
}

}

FailsafeRth::FailsafeRth(Model& model):
  _model(model),
  _active(false),
  _pitchFlipped(false),
  _bearing(0.0f),
  _pitchSign(-1.0f)
{
}

bool FailsafeRth::canUse() const
{
  if(_model.config.failsafe.procedure != FAILSAFE_PROCEDURE_GPS_RESCUE) return false;
  if(!_model.gpsActive() || !_model.state.gps.isHomeValid()) return false;
  if(_model.state.gps.numSats < _model.config.gps.rescueMinSats) return false;
  if(!_model.accelActive()) return false;
  if(_model.config.gps.rescueSanityChecks && _model.state.gps.distanceToHome < _model.config.gps.rescueMinDistance) return false;
  return true;
}

void FailsafeRth::reset()
{
  _active = false;
  _pitchFlipped = false;
  _bearing = 0.0f;
  _pitchSign = -1.0f;
}

FailsafeRthAction FailsafeRth::update(FailsafeCommand& command)
{
  if(_model.state.failsafe.phase != FC_FAILSAFE_GPS_RESCUE)
  {
    _model.state.failsafe.phase = FC_FAILSAFE_GPS_RESCUE;
    _model.state.failsafe.timeout = millis();
    reset();
  }

  if(_model.state.gps.distanceToHome <= std::max((int)_model.config.gps.rescueMinDistance, 3))
  {
    return FailsafeRthAction::Land;
  }

  buildCommand(command);

  const uint32_t elapsed = millis() - _model.state.failsafe.timeout;
  const uint32_t maxRescueTimeMs = (uint32_t)_model.config.failsafe.offDelay * 100UL;
  if(maxRescueTimeMs > 0 && elapsed >= maxRescueTimeMs)
  {
    return FailsafeRthAction::Land;
  }

  return FailsafeRthAction::Continue;
}

float FailsafeRth::gpsCourseDegrees() const
{
  if(_model.state.gps.velocity.raw.heading != 0)
  {
    return wrapAngle360(_model.state.gps.velocity.raw.heading * 1e-5f);
  }

  const float north = _model.state.gps.velocity.raw.north * 0.001f;
  const float east = _model.state.gps.velocity.raw.east * 0.001f;
  if(std::fabs(north) < 0.001f && std::fabs(east) < 0.001f)
  {
    return 0.f;
  }

  return wrapAngle360(Utils::toDeg(std::atan2(east, north)));
}

void FailsafeRth::buildCommand(FailsafeCommand& command)
{
  const float homeBearing = wrapAngle360((float)_model.state.gps.directionToHome);
  if(!_active)
  {
    _active = true;
    _pitchFlipped = false;
    _bearing = homeBearing;
    _pitchSign = -1.0f;
  }
  else
  {
    _bearing = blendAngle(_bearing, homeBearing, 0.2f);
  }

  const float insYaw = wrapAngle360(Utils::toDeg(-_model.state.attitude.euler[AXIS_YAW]));
  const float groundSpeed = std::max(_model.state.gps.velocity.raw.groundSpeed * 0.001f, 0.0f);
  const float course = groundSpeed > MIN_SPEED ? gpsCourseDegrees() : _bearing;
  const float courseError = wrapAngle180(_bearing - course);
  const float crabAngle = groundSpeed > MIN_SPEED
    ? Utils::clamp(courseError * 0.35f, -MAX_CRAB_ANGLE, MAX_CRAB_ANGLE)
    : 0.0f;
  const float targetBearing = wrapAngle360(_bearing + crabAngle);
  const float yawError = wrapAngle180(targetBearing - insYaw);
  const float closingSpeed = groundSpeed * std::cos(Utils::toRad(courseError));

  if(!_pitchFlipped && groundSpeed > MIN_SPEED && std::fabs(yawError) < 35.0f && closingSpeed < -0.25f)
  {
    _pitchSign = -_pitchSign;
    _pitchFlipped = true;
  }

  const float alignFactor = Utils::clamp(1.0f - std::fabs(yawError) / 90.0f, 0.0f, 1.0f);
  const float angleLimit = std::max((float)_model.config.level.angleLimit, 1.0f);
  const float maxAngle = Utils::clamp(std::min((float)_model.config.gps.rescueMaxAngle, angleLimit), 5.0f, angleLimit);
  const float targetSpeed = (float)std::max((int)_model.config.gps.rescueGroundSpeed, 1) * alignFactor;
  float pitchAngle = Utils::clamp((targetSpeed - std::max(closingSpeed, 0.0f)) * PITCH_GAIN, 0.0f, maxAngle);

  const float brakeDistance = std::max((float)_model.config.gps.rescueMinDistance * 2.0f, 1.0f);
  if(_model.state.gps.distanceToHome < brakeDistance)
  {
    pitchAngle *= Utils::clamp(_model.state.gps.distanceToHome / brakeDistance, 0.0f, 1.0f);
  }

  const float pitchCommand = _pitchSign * Utils::clamp(pitchAngle / angleLimit, 0.0f, 1.0f);
  const float yawCommand = Utils::clamp(yawError * YAW_GAIN, -1.0f, 1.0f);

  float thrustCommandUs = Utils::clamp((int)_model.config.failsafe.throttle, (int)FAILSAFE_PWM_MIN, (int)FAILSAFE_PWM_MAX);
  if(_model.baroActive())
  {
    const float targetAltitude = std::max((float)_model.config.gps.rescueAltitude, _model.state.altitude.height);
    const float altitudeError = targetAltitude - _model.state.altitude.height;
    const float verticalRate = Utils::clamp(altitudeError * 0.35f, -0.75f, 1.5f);
    thrustCommandUs = normalizedToPwm(altHoldRateToInput(verticalRate));
  }

  command.roll = FAILSAFE_PWM_MID;
  command.pitch = pitchCommand * 500.0f + FAILSAFE_PWM_MID;
  command.yaw = yawCommand * 500.0f + FAILSAFE_PWM_MID;
  command.thrust = thrustCommandUs;

  if(_model.config.debug.mode == DEBUG_RTH || _model.config.debug.mode == DEBUG_GPS_RESCUE_THROTTLE_PID)
  {
    _model.state.debug[0] = std::clamp(lrintf(_model.state.gps.distanceToHome), -32000l, 32000l);
    _model.state.debug[1] = std::clamp(lrintf(targetBearing * 10.0f), -32000l, 32000l);
    _model.state.debug[2] = std::clamp(lrintf(yawError * 10.0f), -32000l, 32000l);
    _model.state.debug[3] = std::clamp(lrintf(closingSpeed * 100.0f), -32000l, 32000l);
    _model.state.debug[4] = std::clamp(lrintf(pitchAngle * 10.0f), -32000l, 32000l);
    _model.state.debug[5] = std::clamp(lrintf((thrustCommandUs - FAILSAFE_PWM_MID) * 10.0f), -32000l, 32000l);
  }
}

}
}
