#include "Utils/HybridFusion.h"
#include "Utils/Math.hpp"
#include <cmath>

namespace Espfc {
namespace Utils {

HybridFusion::HybridFusion(Model& model)
  : _model(model)
  , _initialized(false)
  , _lastGpsUpdate(0)
  , _lastImuUpdate(0)
{
}

int HybridFusion::begin()
{
  _initialized = false;
  _lastAccel.set(0.0f, 0.0f, 0.0f);
  _lastGpsUpdate = 0;
  _lastImuUpdate = 0;
  return 1;
}

int HybridFusion::update()
{
  // Update IMU prediction at high rate
  updateIMU();

  // Update GPS measurement at low rate
  if(_model.state.gps.fix && _model.state.gps.homeSet)
  {
    updateGPS();
  }

  return 1;
}

void HybridFusion::updateIMU()
{
  uint32_t now = micros();
  if(_lastImuUpdate == 0)
  {
    _lastImuUpdate = now;
    return;
  }

  float dt = (now - _lastImuUpdate) * 1e-6f;
  _lastImuUpdate = now;

  if(dt <= 0.0f || dt > 1.0f) return; // Sanity check

  // Get acceleration in body frame from accelerometer
  VectorFloat accelBody = _model.state.accel.adc;

  // Subtract gravity
  accelBody.z -= ACCEL_G;

  // Rotate acceleration to earth frame using attitude quaternion
  VectorFloat accelEarth = accelBody;
  accelEarth.rotate(_model.state.attitude.quaternion);

  // Convert to NED (North-East-Down) frame
  // In the earth frame, Z is up, but NED uses Z down
  VectorFloat accelNED(accelEarth.x, accelEarth.y, -accelEarth.z);

  // Store for external access
  _lastAccel = accelNED;

  // Predict using Kalman filter
  _kalman.predict(accelNED, dt);
}

void HybridFusion::updateGPS()
{
  uint32_t now = micros();
  
  // Only update on new GPS data
  if(_model.state.gps.lastMsgTs == _lastGpsUpdate) return;
  _lastGpsUpdate = _model.state.gps.lastMsgTs;

  // Convert GPS position to NED
  VectorFloat gpsPos = gpsToNED(
    _model.state.gps.location.raw.lat,
    _model.state.gps.location.raw.lon,
    _model.state.gps.location.raw.height
  );

  // Convert GPS velocity to NED
  VectorFloat gpsVel = gpsVelToNED(
    _model.state.gps.velocity.raw.north,
    _model.state.gps.velocity.raw.east,
    _model.state.gps.velocity.raw.down
  );

  // Get GPS quality metric
  float quality = getGPSQuality();

  // Initialize Kalman filter on first GPS fix
  if(!_kalman.isInitialized())
  {
    _kalman.init(gpsPos, gpsVel);
    _initialized = true;
  }
  else
  {
    // Update Kalman filter with GPS measurement
    _kalman.update(gpsPos, gpsVel, quality);
  }
}

float HybridFusion::getGPSQuality() const
{
  // Calculate quality based on:
  // 1. Number of satellites
  // 2. Fix type
  // 3. Horizontal accuracy

  float quality = 0.0f;

  // Satellite count contribution (0-0.5)
  float satQuality = Utils::clamp((float)_model.state.gps.numSats / 12.0f, 0.0f, 1.0f) * 0.5f;
  quality += satQuality;

  // Fix type contribution (0-0.3)
  float fixQuality = 0.0f;
  if(_model.state.gps.fixType >= 3) // 3D fix
  {
    fixQuality = 0.3f;
  }
  else if(_model.state.gps.fixType == 2) // 2D fix
  {
    fixQuality = 0.15f;
  }
  quality += fixQuality;

  // Horizontal accuracy contribution (0-0.2)
  if(_model.state.gps.accuracy.horizontal > 0)
  {
    // Lower HDOP is better. Typical values: 1-3m is excellent, 5-10m is good
    float hdopQuality = Utils::clamp(5000.0f / (float)_model.state.gps.accuracy.horizontal, 0.0f, 1.0f) * 0.2f;
    quality += hdopQuality;
  }

  return Utils::clamp(quality, 0.0f, 1.0f);
}

VectorFloat HybridFusion::gpsToNED(int32_t lat, int32_t lon, int32_t height) const
{
  if(!_model.state.gps.homeSet)
  {
    return VectorFloat(0.0f, 0.0f, 0.0f);
  }

  // Get home position
  int32_t homeLat = _model.state.gps.location.home.lat;
  int32_t homeLon = _model.state.gps.location.home.lon;
  int32_t homeHeight = _model.state.gps.location.home.height;

  // Calculate differences
  float dLat = (lat - homeLat) * 1e-7f; // degrees
  float dLon = (lon - homeLon) * 1e-7f; // degrees
  float dHeight = (height - homeHeight) * 1e-3f; // meters

  // Convert to meters (approximate for small distances)
  // 1 degree latitude ≈ 111,320 meters
  // 1 degree longitude ≈ 111,320 * cos(latitude) meters
  float latRad = homeLat * 1e-7f * M_PI / 180.0f;
  float north = dLat * 111320.0f;
  float east = dLon * 111320.0f * cos(latRad);
  float down = -dHeight; // NED frame has down positive

  return VectorFloat(north, east, down);
}

VectorFloat HybridFusion::gpsVelToNED(int32_t north, int32_t east, int32_t down) const
{
  // Convert from mm/s to m/s
  return VectorFloat(
    north * 1e-3f,
    east * 1e-3f,
    down * 1e-3f
  );
}

VectorFloat HybridFusion::getPosition() const
{
  return _kalman.getPosition();
}

VectorFloat HybridFusion::getVelocity() const
{
  return _kalman.getVelocity();
}

VectorFloat HybridFusion::getAcceleration() const
{
  return _lastAccel;
}

bool HybridFusion::isValid() const
{
  return _initialized && _kalman.isInitialized();
}

void HybridFusion::reset()
{
  _kalman.reset();
  _initialized = false;
  _lastGpsUpdate = 0;
  _lastImuUpdate = 0;
  _lastAccel.set(0.0f, 0.0f, 0.0f);
}

} // namespace Utils
} // namespace Espfc
