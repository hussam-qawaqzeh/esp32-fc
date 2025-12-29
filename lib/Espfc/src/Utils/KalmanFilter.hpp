#pragma once

#include <cmath>
#include <cstring>
#include "helper_3dmath.h"

namespace Espfc {
namespace Utils {

/**
 * @brief Simple Kalman Filter for GPS/IMU sensor fusion
 * 
 * This implements a simplified Kalman filter for position and velocity estimation
 * by fusing GPS position measurements with IMU-based dead reckoning.
 */
class KalmanFilter
{
public:
  KalmanFilter()
    : _position(0.0f, 0.0f, 0.0f)
    , _velocity(0.0f, 0.0f, 0.0f)
    , _initialized(false)
  {
    // Process noise covariance (how much we trust the model)
    _processNoise = 0.01f;
    
    // Measurement noise covariance (how much we trust GPS)
    _measurementNoise = 1.0f;
    
    // Initialize state covariance
    for(int i = 0; i < 6; i++)
    {
      for(int j = 0; j < 6; j++)
      {
        _covariance[i][j] = (i == j) ? 1.0f : 0.0f;
      }
    }
  }

  /**
   * @brief Initialize filter with GPS position
   */
  void init(const VectorFloat& position, const VectorFloat& velocity)
  {
    _position = position;
    _velocity = velocity;
    _initialized = true;
  }

  /**
   * @brief Prediction step using IMU acceleration
   * @param acceleration Acceleration from IMU (m/s^2)
   * @param dt Time step (seconds)
   */
  void predict(const VectorFloat& acceleration, float dt)
  {
    if(!_initialized) return;

    // State prediction: x = x + v*dt + 0.5*a*dt^2
    _position.x += _velocity.x * dt + 0.5f * acceleration.x * dt * dt;
    _position.y += _velocity.y * dt + 0.5f * acceleration.y * dt * dt;
    _position.z += _velocity.z * dt + 0.5f * acceleration.z * dt * dt;

    // Velocity prediction: v = v + a*dt
    _velocity.x += acceleration.x * dt;
    _velocity.y += acceleration.y * dt;
    _velocity.z += acceleration.z * dt;

    // Update covariance (simplified)
    float q = _processNoise * dt * dt;
    for(int i = 0; i < 6; i++)
    {
      _covariance[i][i] += q;
    }
  }

  /**
   * @brief Update step using GPS measurement
   * @param gpsPosition GPS position measurement (m)
   * @param gpsVelocity GPS velocity measurement (m/s)
   * @param quality GPS quality (0-1, higher is better)
   * @return true if update was applied
   */
  bool update(const VectorFloat& gpsPosition, const VectorFloat& gpsVelocity, float quality)
  {
    if(!_initialized || quality < 0.1f) return false;

    // Adaptive measurement noise based on GPS quality
    float r = _measurementNoise * (1.0f / (quality + 0.1f));

    // Simplified Kalman gain calculation
    float k[6];
    for(int i = 0; i < 6; i++)
    {
      k[i] = _covariance[i][i] / (_covariance[i][i] + r);
    }

    // Update position
    VectorFloat posError(
      gpsPosition.x - _position.x,
      gpsPosition.y - _position.y,
      gpsPosition.z - _position.z
    );
    
    _position.x += k[0] * posError.x;
    _position.y += k[1] * posError.y;
    _position.z += k[2] * posError.z;

    // Update velocity
    VectorFloat velError(
      gpsVelocity.x - _velocity.x,
      gpsVelocity.y - _velocity.y,
      gpsVelocity.z - _velocity.z
    );
    
    _velocity.x += k[3] * velError.x;
    _velocity.y += k[4] * velError.y;
    _velocity.z += k[5] * velError.z;

    // Update covariance (simplified)
    for(int i = 0; i < 6; i++)
    {
      _covariance[i][i] *= (1.0f - k[i]);
    }

    return true;
  }

  /**
   * @brief Reset the filter
   */
  void reset()
  {
    _initialized = false;
    _position.set(0.0f, 0.0f, 0.0f);
    _velocity.set(0.0f, 0.0f, 0.0f);
  }

  // Getters
  const VectorFloat& getPosition() const { return _position; }
  const VectorFloat& getVelocity() const { return _velocity; }
  bool isInitialized() const { return _initialized; }

  // Setters for tuning
  void setProcessNoise(float noise) { _processNoise = noise; }
  void setMeasurementNoise(float noise) { _measurementNoise = noise; }

private:
  VectorFloat _position;      // Estimated position (m)
  VectorFloat _velocity;      // Estimated velocity (m/s)
  float _covariance[6][6];    // State covariance matrix
  float _processNoise;        // Process noise covariance
  float _measurementNoise;    // Measurement noise covariance
  bool _initialized;          // Filter initialization status
};

} // namespace Utils
} // namespace Espfc
