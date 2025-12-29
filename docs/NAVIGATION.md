# Hybrid Navigation System

## Overview

The hybrid navigation system provides GPS-based autonomous flight capabilities for the ESP32-FC flight controller. It intelligently fuses GPS position data with IMU acceleration measurements to provide robust position and velocity estimates for autonomous navigation modes.

## Features

### Core Capabilities
- **GPS/IMU Sensor Fusion**: Kalman filter-based fusion of GPS and IMU data
- **Multiple Navigation Modes**:
  - GPS Hold: Maintain current position
  - Return-to-Home (RTH): Navigate back to takeoff location
  - Waypoint: Navigate to specified coordinates
  - Cruise: Maintain heading and altitude
- **Failsafe RTH**: Automatic RTH activation on RC signal loss
- **Position Control**: Cascaded PID control for position and velocity
- **Safe Landing**: Automated descent and disarm sequence

### Flight Modes

#### GPS_HOLD Mode
Maintains the current GPS position. The aircraft will automatically compensate for wind and drift to stay at the same location.

**Requirements:**
- GPS fix with minimum satellites (configurable, default: 8)
- Home position set
- Armed state

#### RETURN_TO_HOME (RTH) Mode
Autonomously navigates the aircraft back to the home position (takeoff location) and lands.

**RTH Phases:**
1. **Climb**: Ascends to configured RTH altitude
2. **Navigate**: Flies to home position at RTH altitude
3. **Descent**: Descends to ground level when within home tolerance
4. **Landing**: Gentle touchdown and automatic disarm

**Requirements:**
- GPS fix with minimum satellites
- Valid home position
- Armed state

#### Failsafe RTH
Automatically activated when RC signal is lost while armed.

**Conditions:**
- RTH failsafe enabled in configuration
- GPS present and valid fix
- Home position set
- Minimum GPS satellites met

**Fallback:** If GPS is not available, the aircraft will immediately disarm (traditional failsafe behavior).

### Sensor Fusion

The system uses a simplified Kalman filter to optimally combine:
- **GPS**: Provides absolute position and velocity (low rate, ~5-10 Hz)
- **IMU**: Provides acceleration measurements (high rate, ~1000 Hz)

Benefits:
- Smooth position estimates at high update rates
- Reduced GPS noise and jitter
- Better response during GPS dropouts
- Improved position accuracy

## Configuration

### Navigation Parameters (ModelConfig.h)

```cpp
struct NavigationConfig
{
  // PID gains for navigation (scaled by 10)
  uint8_t posP = 20;      // Position P gain
  uint8_t posI = 5;       // Position I gain
  uint8_t posD = 10;      // Position D gain
  
  uint8_t velP = 15;      // Velocity P gain
  uint8_t velI = 3;       // Velocity I gain
  uint8_t velD = 5;       // Velocity D gain
  
  // Navigation limits
  uint8_t maxAngle = 30;           // Maximum navigation angle (degrees)
  uint8_t maxSpeed = 10;           // Maximum navigation speed (m/s)
  uint8_t maxClimbRate = 3;        // Maximum climb rate (m/s)
  uint8_t maxDescentRate = 2;      // Maximum descent rate (m/s)
  
  // RTH parameters
  uint16_t rthAltitude = 20;       // RTH altitude (meters)
  uint8_t rthDescentRate = 1;      // RTH descent rate (m/s)
  uint16_t homeTolerance = 2;      // Home position tolerance (meters)
  uint8_t landingDescentRate = 1;  // Landing descent rate (m/s)
  uint8_t landingThrottle = 30;    // Landing throttle percent (0-100)
  
  // Failsafe RTH
  uint8_t enableRthFailsafe = 1;   // Enable RTH on failsafe
  uint8_t minGpsQuality = 5;       // Minimum GPS quality for RTH (0-10)
};
```

### PID Tuning

The navigation system uses cascaded PID controllers:

1. **Position PIDs** (Outer Loop)
   - Input: Position error (target - current)
   - Output: Desired velocity
   - Tuning: Start with low P gain (1-2), increase until stable

2. **Velocity PIDs** (Inner Loop)
   - Input: Velocity error (desired - current)
   - Output: Desired acceleration
   - Tuning: Higher P gain (1.5-2.5), minimal I and D

### GPS Configuration

```cpp
struct GpsConfig
{
  uint8_t minSats = 8;      // Minimum satellites for GPS fix
  uint8_t setHomeOnce = 1;  // Set home only once on first arm
};
```

## Implementation Details

### File Structure

```
lib/Espfc/src/
├── Control/
│   ├── Navigation.h         # Navigation controller header
│   └── Navigation.cpp       # Navigation controller implementation
├── Utils/
│   ├── KalmanFilter.hpp     # Kalman filter for sensor fusion
│   ├── HybridFusion.h       # GPS/IMU fusion header
│   └── HybridFusion.cpp     # GPS/IMU fusion implementation
├── ModelConfig.h            # Updated with NavigationConfig
└── ModelState.h             # Updated with NavigationState
```

### Integration Points

1. **Espfc.h/cpp**: Navigation controller instantiation and update loop
2. **Controller.cpp**: Navigation output integration in control loop
3. **Actuator.cpp**: Navigation mode activation and validation
4. **Input.cpp**: Failsafe RTH trigger logic

### Coordinate Frames

The system uses the North-East-Down (NED) coordinate frame:
- **North**: Positive towards geographic north
- **East**: Positive towards geographic east
- **Down**: Positive towards ground (negative altitude)

GPS coordinates are converted to local NED frame relative to home position for navigation calculations.

## Usage

### Setting Home Position
Home position is automatically set when the aircraft is armed for the first time (if `setHomeOnce = 1`). It can also be set manually via CLI.

### Activating Navigation Modes
Navigation modes are activated through AUX channel switches configured in the actuator conditions (similar to other flight modes).

### RTH Activation
RTH can be activated:
1. Manually via AUX switch
2. Automatically on RC signal loss (if failsafe RTH enabled)

### Landing Sequence
When RTH reaches home position:
1. Descends to ground level at configured descent rate
2. Reduces throttle to landing throttle when close to ground
3. Automatically disarms when touchdown detected

## Safety Features

- **GPS Validity Checks**: Navigation modes require valid GPS fix
- **Home Position Validation**: RTH requires valid home position
- **Altitude Limits**: Configurable climb/descent rate limits
- **Speed Limits**: Maximum navigation speed enforcement
- **Angle Limits**: Maximum pitch/roll angles for navigation
- **Automatic Deactivation**: Navigation disabled when disarmed
- **Failsafe Fallback**: Immediate disarm if RTH unavailable

## Performance Considerations

- **Update Rate**: Navigation controller runs at main loop rate (~1kHz)
- **GPS Rate**: Typically 5-10 Hz, fusion provides smooth estimates
- **CPU Usage**: Minimal overhead, Kalman filter uses simplified model
- **Memory**: ~200 bytes for navigation state

## Limitations

- GPS position accuracy depends on satellite visibility and quality
- Coordinate conversion assumes flat earth (accurate for distances < 10km)
- No obstacle avoidance
- Requires good GPS signal for reliable operation
- Wind compensation limited by aircraft performance

## Future Enhancements

- Altitude hold using barometer fusion
- Magnetometer heading for improved course tracking
- Waypoint mission support
- Position hold with manual input override
- Advanced path planning and obstacle avoidance
- Geofencing capabilities

## Troubleshooting

### Navigation Not Activating
- Check GPS fix status (minimum satellites)
- Verify home position is set
- Ensure aircraft is armed
- Check mode switch configuration

### Poor Position Hold
- Verify GPS quality (satellite count, HDOP)
- Tune position PID gains
- Check for GPS antenna placement issues
- Verify magnetometer calibration

### RTH Not Working
- Check failsafe RTH enabled in config
- Verify minimum GPS quality setting
- Check home position validity
- Review GPS satellite count

## Technical References

- Kalman Filter: Simplified position-velocity model
- PID Control: Cascaded position-velocity control
- Sensor Fusion: GPS/IMU complementary fusion
- Coordinate Systems: NED (North-East-Down) frame

## Credits

Implementation based on:
- Betaflight/iNav GPS rescue concepts
- Standard Kalman filter theory
- Cascaded PID control architecture
