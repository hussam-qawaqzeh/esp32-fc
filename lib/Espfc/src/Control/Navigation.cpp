#include "Control/Navigation.h"
#include "Utils/Math.hpp"
#include <cmath>

namespace Espfc {
namespace Control {

Navigation::Navigation(Model& model)
  : _model(model)
  , _fusion(model)
{
}

int Navigation::begin()
{
  _fusion.begin();

  // Initialize position PIDs
  _pidPosNorth.begin();
  _pidPosEast.begin();
  _pidPosDown.begin();

  // Initialize velocity PIDs
  _pidVelNorth.begin();
  _pidVelEast.begin();
  _pidVelDown.begin();

  // Reset navigation state
  _model.state.navigation.mode = NAV_MODE_NONE;
  _model.state.navigation.phase = NAV_PHASE_IDLE;
  _model.state.navigation.navigationActive = false;

  return 1;
}

int Navigation::update()
{
  // Update fusion first
  _fusion.update();

  // Update navigation state
  updateNavigationState();

  // Check if navigation is active
  if(!_model.state.navigation.navigationActive)
  {
    return 1;
  }

  // Update based on active mode
  switch(_model.state.navigation.mode)
  {
    case NAV_MODE_GPS_HOLD:
      updateGpsHold();
      break;
    case NAV_MODE_RETURN_TO_HOME:
      updateReturnToHome();
      break;
    case NAV_MODE_WAYPOINT:
      updateWaypoint();
      break;
    case NAV_MODE_CRUISE:
      updateCruise();
      break;
    default:
      break;
  }

  return 1;
}

void Navigation::updateNavigationState()
{
  // Update position and velocity from fusion
  if(_fusion.isValid())
  {
    _model.state.navigation.position = _fusion.getPosition();
    _model.state.navigation.velocity = _fusion.getVelocity();
    _model.state.navigation.acceleration = _fusion.getAcceleration();
  }

  // Update GPS validity
  _model.state.navigation.gpsValid = isGpsValid();
  _model.state.navigation.homeValid = isHomeValid();
  _model.state.navigation.lastUpdate = micros();
}

bool Navigation::setHome()
{
  if(!isGpsValid()) return false;

  // Set home position in GPS state
  _model.state.gps.location.home.lat = _model.state.gps.location.raw.lat;
  _model.state.gps.location.home.lon = _model.state.gps.location.raw.lon;
  _model.state.gps.location.home.height = _model.state.gps.location.raw.height;
  _model.state.gps.homeSet = true;

  // Reset fusion to recalibrate NED frame
  _fusion.reset();

  return true;
}

bool Navigation::activateRTH()
{
  if(!canActivate()) return false;

  _model.state.navigation.mode = NAV_MODE_RETURN_TO_HOME;
  _model.state.navigation.phase = NAV_PHASE_RTH_CLIMB;
  _model.state.navigation.navigationActive = true;
  _model.state.navigation.phaseStartTime = micros();

  // Set target position to home
  _model.state.navigation.targetPosition.set(0.0f, 0.0f, -_model.config.navigation.rthAltitude);

  return true;
}

void Navigation::deactivate()
{
  _model.state.navigation.mode = NAV_MODE_NONE;
  _model.state.navigation.phase = NAV_PHASE_IDLE;
  _model.state.navigation.navigationActive = false;

  // Reset PID controllers
  _pidPosNorth.iTerm = 0.0f;
  _pidPosEast.iTerm = 0.0f;
  _pidPosDown.iTerm = 0.0f;
  _pidVelNorth.iTerm = 0.0f;
  _pidVelEast.iTerm = 0.0f;
  _pidVelDown.iTerm = 0.0f;
}

bool Navigation::canActivate() const
{
  // Check GPS validity
  if(!isGpsValid()) return false;

  // Check home position
  if(!isHomeValid()) return false;

  // Check if armed
  if(!_model.isModeActive(MODE_ARMED)) return false;

  // Check minimum GPS quality
  if(_model.state.gps.numSats < _model.config.navigation.minGpsQuality) return false;

  return true;
}

void Navigation::updateGpsHold()
{
  // In GPS hold mode, maintain current position
  // Target is set once when mode is activated
  calculateNavigationOutputs();
}

void Navigation::updateReturnToHome()
{
  switch(_model.state.navigation.phase)
  {
    case NAV_PHASE_RTH_CLIMB:
      rthClimb();
      break;
    case NAV_PHASE_RTH_NAVIGATE:
      rthNavigate();
      break;
    case NAV_PHASE_RTH_DESCENT:
      rthDescent();
      break;
    case NAV_PHASE_RTH_LANDING:
      rthLanding();
      break;
    default:
      break;
  }
}

void Navigation::rthClimb()
{
  // Climb to RTH altitude
  float currentAltitude = -_model.state.navigation.position.z; // NED frame, down is positive
  float targetAltitude = _model.config.navigation.rthAltitude;

  if(currentAltitude >= targetAltitude - 2.0f) // Within 2m tolerance
  {
    // Switch to navigate phase
    _model.state.navigation.phase = NAV_PHASE_RTH_NAVIGATE;
    _model.state.navigation.phaseStartTime = micros();
  }

  // Set target position to climb
  _model.state.navigation.targetPosition.set(
    _model.state.navigation.position.x,
    _model.state.navigation.position.y,
    -targetAltitude
  );

  calculateNavigationOutputs();
}

void Navigation::rthNavigate()
{
  // Navigate to home position
  _model.state.navigation.targetPosition.set(0.0f, 0.0f, -_model.config.navigation.rthAltitude);

  calculateNavigationOutputs();

  // Calculate distance to home
  float distance = calculateDistance(_model.state.navigation.position, _model.state.navigation.targetPosition);
  _model.state.navigation.distanceToTarget = distance;

  // Check if we're close to home
  if(distance < _model.config.navigation.homeTolerance)
  {
    // Switch to descent phase
    _model.state.navigation.phase = NAV_PHASE_RTH_DESCENT;
    _model.state.navigation.phaseStartTime = micros();
  }
}

void Navigation::rthDescent()
{
  // Descend to landing altitude
  _model.state.navigation.targetPosition.set(
    0.0f,
    0.0f,
    0.0f // Ground level
  );

  calculateNavigationOutputs();

  float currentAltitude = -_model.state.navigation.position.z;
  
  // Check if close to ground (within 1m)
  if(currentAltitude < 1.0f)
  {
    // Switch to landing phase
    _model.state.navigation.phase = NAV_PHASE_RTH_LANDING;
    _model.state.navigation.phaseStartTime = micros();
  }
}

void Navigation::rthLanding()
{
  // Gentle landing - reduce throttle gradually
  _model.state.navigation.desiredThrottle = 0.3f; // Low throttle for gentle touchdown
  _model.state.navigation.desiredRoll = 0.0f;
  _model.state.navigation.desiredPitch = 0.0f;

  // Check if landed (very low altitude and low vertical velocity)
  float currentAltitude = -_model.state.navigation.position.z;
  float verticalVel = std::abs(_model.state.navigation.velocity.z);

  if(currentAltitude < 0.3f && verticalVel < 0.1f)
  {
    // Landed - disarm
    _model.state.navigation.phase = NAV_PHASE_RTH_LANDED;
    _model.disarm(DISARM_REASON_GPS_RESCUE);
    deactivate();
  }
}

void Navigation::updateWaypoint()
{
  // Navigate to waypoint
  calculateNavigationOutputs();
}

void Navigation::updateCruise()
{
  // Maintain heading and altitude, allow manual roll/pitch input
  // Not fully implemented in this minimal version
}

void Navigation::calculateNavigationOutputs()
{
  // Position error
  VectorFloat posError(
    _model.state.navigation.targetPosition.x - _model.state.navigation.position.x,
    _model.state.navigation.targetPosition.y - _model.state.navigation.position.y,
    _model.state.navigation.targetPosition.z - _model.state.navigation.position.z
  );

  // Configure PIDs with navigation config
  float posP = _model.config.navigation.posP * 0.1f;
  float posI = _model.config.navigation.posI * 0.1f;
  float posD = _model.config.navigation.posD * 0.1f;

  _pidPosNorth.Kp = posP;
  _pidPosNorth.Ki = posI;
  _pidPosNorth.Kd = posD;
  _pidPosEast.Kp = posP;
  _pidPosEast.Ki = posI;
  _pidPosEast.Kd = posD;
  _pidPosDown.Kp = posP;
  _pidPosDown.Ki = posI;
  _pidPosDown.Kd = posD;

  // Position PIDs output desired velocities
  float velNorthDesired = _pidPosNorth.update(posError.x, _model.state.navigation.velocity.x);
  float velEastDesired = _pidPosEast.update(posError.y, _model.state.navigation.velocity.y);
  float velDownDesired = _pidPosDown.update(posError.z, _model.state.navigation.velocity.z);

  // Limit desired velocities
  velNorthDesired = Utils::clamp(velNorthDesired, -(float)_model.config.navigation.maxSpeed, (float)_model.config.navigation.maxSpeed);
  velEastDesired = Utils::clamp(velEastDesired, -(float)_model.config.navigation.maxSpeed, (float)_model.config.navigation.maxSpeed);
  velDownDesired = Utils::clamp(velDownDesired, -(float)_model.config.navigation.maxDescentRate, (float)_model.config.navigation.maxClimbRate);

  // Velocity error
  float velNorthError = velNorthDesired - _model.state.navigation.velocity.x;
  float velEastError = velEastDesired - _model.state.navigation.velocity.y;
  float velDownError = velDownDesired - _model.state.navigation.velocity.z;

  // Configure velocity PIDs
  float velP = _model.config.navigation.velP * 0.1f;
  float velI = _model.config.navigation.velI * 0.1f;
  float velD = _model.config.navigation.velD * 0.1f;

  _pidVelNorth.Kp = velP;
  _pidVelNorth.Ki = velI;
  _pidVelNorth.Kd = velD;
  _pidVelEast.Kp = velP;
  _pidVelEast.Ki = velI;
  _pidVelEast.Kd = velD;
  _pidVelDown.Kp = velP;
  _pidVelDown.Ki = velI;
  _pidVelDown.Kd = velD;

  // Velocity PIDs output desired accelerations
  float accelNorth = _pidVelNorth.update(velNorthError, 0.0f);
  float accelEast = _pidVelEast.update(velEastError, 0.0f);
  float accelDown = _pidVelDown.update(velDownError, 0.0f);

  // Convert desired accelerations to roll/pitch angles
  // For small angles: pitch ≈ -accelNorth/g, roll ≈ accelEast/g
  float desiredPitch = -std::atan2(accelNorth, ACCEL_G);
  float desiredRoll = std::atan2(accelEast, ACCEL_G);

  // Limit angles
  limitAngle(desiredPitch, _model.config.navigation.maxAngle);
  limitAngle(desiredRoll, _model.config.navigation.maxAngle);

  // Calculate throttle adjustment for vertical velocity
  float throttleAdjust = accelDown / ACCEL_G;
  float desiredThrottle = 0.5f + throttleAdjust; // Base throttle at hover + adjustment

  // Store outputs
  _model.state.navigation.desiredRoll = desiredRoll;
  _model.state.navigation.desiredPitch = desiredPitch;
  _model.state.navigation.desiredYaw = _model.state.attitude.euler.z; // Maintain current heading
  _model.state.navigation.desiredThrottle = Utils::clamp(desiredThrottle, 0.3f, 0.8f);

  // Calculate bearing and distance for telemetry
  _model.state.navigation.bearingToTarget = calculateBearing(_model.state.navigation.position, _model.state.navigation.targetPosition);
  _model.state.navigation.distanceToTarget = calculateDistance(_model.state.navigation.position, _model.state.navigation.targetPosition);
  _model.state.navigation.altitudeError = posError.z;
}

float Navigation::calculateBearing(const VectorFloat& from, const VectorFloat& to) const
{
  float dNorth = to.x - from.x;
  float dEast = to.y - from.y;
  return std::atan2(dEast, dNorth);
}

float Navigation::calculateDistance(const VectorFloat& from, const VectorFloat& to) const
{
  float dNorth = to.x - from.x;
  float dEast = to.y - from.y;
  return std::sqrt(dNorth * dNorth + dEast * dEast);
}

void Navigation::limitAngle(float& angle, float maxAngle) const
{
  float maxRad = Utils::toRad(maxAngle);
  angle = Utils::clamp(angle, -maxRad, maxRad);
}

void Navigation::getOutputs(float& roll, float& pitch, float& yaw, float& throttle) const
{
  roll = _model.state.navigation.desiredRoll;
  pitch = _model.state.navigation.desiredPitch;
  yaw = _model.state.navigation.desiredYaw;
  throttle = _model.state.navigation.desiredThrottle;
}

bool Navigation::isGpsValid() const
{
  return _model.state.gps.present 
      && _model.state.gps.fix 
      && _model.state.gps.fixType >= 3
      && _model.state.gps.numSats >= _model.config.gps.minSats;
}

bool Navigation::isHomeValid() const
{
  return _model.state.gps.homeSet && isGpsValid();
}

} // namespace Control
} // namespace Espfc
