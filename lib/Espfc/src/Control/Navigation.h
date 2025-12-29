#pragma once

#include "Model.h"
#include "Utils/HybridFusion.h"
#include "Control/Pid.h"

namespace Espfc {
namespace Control {

/**
 * @brief Navigation Controller for GPS-based autonomous flight
 * 
 * Provides GPS Hold, Return-to-Home, Waypoint navigation, and Cruise modes
 */
class Navigation
{
public:
  Navigation(Model& model);

  /**
   * @brief Initialize navigation system
   */
  int begin();

  /**
   * @brief Update navigation controller
   * @return 1 on success
   */
  int update();

  /**
   * @brief Set home position to current GPS location
   * @return true if successful
   */
  bool setHome();

  /**
   * @brief Activate Return-to-Home mode
   * @return true if RTH can be activated
   */
  bool activateRTH();

  /**
   * @brief Deactivate navigation
   */
  void deactivate();

  /**
   * @brief Check if navigation can be activated
   */
  bool canActivate() const;

  /**
   * @brief Get current navigation outputs
   */
  void getOutputs(float& roll, float& pitch, float& yaw, float& throttle) const;

private:
  // Mode-specific updates
  void updateGpsHold();
  void updateReturnToHome();
  void updateWaypoint();
  void updateCruise();

  // RTH phase handlers
  void rthClimb();
  void rthNavigate();
  void rthDescent();
  void rthLanding();

  // Helper functions
  void calculateNavigationOutputs();
  float calculateBearing(const VectorFloat& from, const VectorFloat& to) const;
  float calculateDistance(const VectorFloat& from, const VectorFloat& to) const;
  void limitAngle(float& angle, float maxAngle) const;

  // Update navigation state from fusion
  void updateNavigationState();

  // Check GPS validity
  bool isGpsValid() const;
  bool isHomeValid() const;

  Model& _model;
  Utils::HybridFusion _fusion;
  
  // PID controllers for navigation
  Pid _pidPosNorth;
  Pid _pidPosEast;
  Pid _pidPosDown;
  Pid _pidVelNorth;
  Pid _pidVelEast;
  Pid _pidVelDown;

  // Navigation setpoints
  VectorFloat _targetWaypoint;
};

} // namespace Control
} // namespace Espfc
