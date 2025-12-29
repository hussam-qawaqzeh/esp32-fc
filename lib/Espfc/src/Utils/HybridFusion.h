#pragma once

#include "Model.h"
#include "Utils/KalmanFilter.hpp"

namespace Espfc {
namespace Utils {

/**
 * @brief Hybrid GPS/IMU Sensor Fusion
 * 
 * Fuses GPS position data with IMU acceleration for robust position estimation
 */
class HybridFusion
{
public:
  HybridFusion(Model& model);

  /**
   * @brief Initialize the fusion system
   */
  int begin();

  /**
   * @brief Update fusion with latest sensor data
   * @return 1 on success
   */
  int update();

  /**
   * @brief Get fused position in NED frame (North, East, Down) in meters
   */
  VectorFloat getPosition() const;

  /**
   * @brief Get fused velocity in NED frame (m/s)
   */
  VectorFloat getVelocity() const;

  /**
   * @brief Get acceleration in NED frame (m/s^2)
   */
  VectorFloat getAcceleration() const;

  /**
   * @brief Check if fusion is initialized and valid
   */
  bool isValid() const;

  /**
   * @brief Reset fusion state
   */
  void reset();

  /**
   * @brief Convert GPS coordinates to local NED position relative to home
   * @param lat Latitude (degrees * 1e7)
   * @param lon Longitude (degrees * 1e7)
   * @param height Height (mm)
   * @return Position in NED frame (meters)
   */
  VectorFloat gpsToNED(int32_t lat, int32_t lon, int32_t height) const;

  /**
   * @brief Convert GPS velocity to NED frame
   * @param north North velocity (mm/s)
   * @param east East velocity (mm/s)
   * @param down Down velocity (mm/s)
   * @return Velocity in NED frame (m/s)
   */
  VectorFloat gpsVelToNED(int32_t north, int32_t east, int32_t down) const;

private:
  void updateGPS();
  void updateIMU();
  float getGPSQuality() const;

  Model& _model;
  KalmanFilter _kalman;
  VectorFloat _lastAccel;
  bool _initialized;
  uint32_t _lastGpsUpdate;
  uint32_t _lastImuUpdate;
};

} // namespace Utils
} // namespace Espfc
