# Betaflight Integration Documentation

## Overview

This document outlines the compatibility status, implementation roadmap, and technical requirements for integrating Betaflight flight control capabilities with the Hybrid Navigation system on ESP32-based flight controllers.

---

## Compatibility Status

### Current State

| Component | Status | Notes |
|-----------|--------|-------|
| Protocol Support | ⚠️ Partial | MSP (Multiwii Serial Protocol) partially implemented |
| Hardware Compatibility | ✅ Supported | ESP32 meets MCU requirements |
| IMU Integration | ✅ Supported | Standard 6-axis/9-axis sensors compatible |
| Motor Control | ✅ Supported | PWM and DShot protocols available |
| PID Controller | ⚠️ In Development | Custom hybrid implementation in progress |
| Telemetry | ⚠️ Partial | Basic telemetry working, full integration pending |
| Configuration Format | ⏳ Planned | Betaflight config format conversion in backlog |

### Version Target

- **Primary Target**: Betaflight 4.4.x
- **Minimum Version**: Betaflight 4.3.x
- **Future Support**: Betaflight 4.5.x and beyond

---

## Implementation Roadmap

### Phase 1: Core Protocol Implementation (Current)
**Timeline**: Q1 2025

- [x] MSP protocol basic parsing
- [ ] Complete MSP command set implementation
- [ ] Configuration API compatibility layer
- [ ] Status message synchronization
- [ ] Error handling and recovery mechanisms

### Phase 2: Flight Dynamics Integration (Planned)
**Timeline**: Q2 2025

- [ ] Rate controller alignment
- [ ] Attitude estimator compatibility
- [ ] Gyroscope filtering pipeline
- [ ] Accelerometer calibration routines
- [ ] Magnetometer integration

### Phase 3: Advanced Features (Planned)
**Timeline**: Q3 2025

- [ ] OSD (On-Screen Display) support
- [ ] Advanced telemetry formats
- [ ] Custom tuning profiles
- [ ] Failsafe mechanisms
- [ ] GPS integration layer

### Phase 4: Testing & Optimization (Planned)
**Timeline**: Q4 2025

- [ ] Comprehensive hardware testing
- [ ] Performance benchmarking
- [ ] Edge case handling
- [ ] Documentation completion
- [ ] Community beta program

---

## Integration Requirements

### Hardware Requirements

#### Microcontroller
- **Device**: ESP32 (WROOM-32, WROVER, or equivalent)
- **Flash**: Minimum 4MB (8MB+ recommended)
- **RAM**: Minimum 520KB SRAM
- **Clock Speed**: 240 MHz (dual-core preferred)

#### Sensors
- **IMU (6-axis)**: MPU6050, MPU6500, or ICM20602
- **IMU (9-axis)**: BNO055, BMX160 with magnetometer
- **Barometer** (optional): BMP280, MS5611
- **Magnetometer** (optional): HMC5883L, QMC5883L

#### Motor & ESC Control
- **PWM Outputs**: Minimum 4 channels (up to 8 supported)
- **ESC Protocol**: PWM, OneShot, DShot (150, 300, 600)
- **Servo Outputs** (optional): Additional PWM channels

#### Radio Receiver
- **Protocols Supported**: 
  - PPM (Pulse Position Modulation)
  - SBUS (Serial Bus)
  - iBUS (Flysky protocol)
  - PWM (standard)

### Software Requirements

#### Firmware Dependencies
```
- Arduino Framework (ESP32)
- MPU6050 I2C driver
- SPI driver for sensors
- UART drivers for MSP communication
- FreeRTOS (included with Arduino-ESP32)
```

#### Build Environment
- **PlatformIO**: 5.2.x or later
- **Arduino IDE**: 1.8.x or later (alternative)
- **CMake**: 3.10+ (for advanced builds)

#### Communication Interfaces

**UART Configuration** (for Betaflight tools compatibility):
- **UART 0**: Serial monitor (115200 baud)
- **UART 1**: MSP protocol (115200 baud) - configurable
- **UART 2**: GPS/Telemetry (9600-57600 baud)

**I2C Configuration** (for sensors):
- **I2C 0**: Primary sensor bus (GPIO 21 SDA, GPIO 22 SCL)
- **I2C 1**: Secondary sensor bus (optional)

### Software Architecture

#### Core Modules

**1. MSP Protocol Handler**
```
lib/HybridNavigation/
├── msp/
│   ├── msp_protocol.cpp
│   ├── msp_protocol.h
│   ├── msp_commands.h
│   └── msp_serializer.cpp
```

**2. Betaflight Configuration Parser**
```
lib/HybridNavigation/
├── betaflight/
│   ├── bf_config.cpp
│   ├── bf_config.h
│   ├── bf_pid_controller.cpp
│   └── bf_profiles.h
```

**3. Flight Dynamics Interface**
```
lib/HybridNavigation/
├── flight_dynamics/
│   ├── attitude_estimator.cpp
│   ├── rate_controller.cpp
│   └── hybrid_controller.cpp
```

#### Data Flow Architecture

```
┌─────────────────┐
│  Radio Receiver │
│   (PPM/SBUS)    │
└────────┬────────┘
         │
         ▼
┌─────────────────┐      ┌──────────────────┐
│ Betaflight      │◄────►│  Hybrid Config   │
│ Config Parser   │      │  Manager         │
└────────┬────────┘      └──────────────────┘
         │
         ▼
┌─────────────────┐      ┌──────────────────┐
│  Rate Control   │◄────►│  PID Tuning      │
│   (Setpoint)    │      │  Parameters      │
└────────┬────────┘      └──────────────────┘
         │
         ▼
┌─────────────────┐      ┌──────────────────┐
│ Attitude        │◄────►│  Sensor Fusion   │
│ Estimator       │      │  (IMU + Mag)     │
└────────┬────────┘      └──────────────────┘
         │
         ▼
┌─────────────────┐      ┌──────────────────┐
│  Motor Mix &    │◄────►│  ESC Output      │
│  Throttle       │      │  (PWM/DShot)     │
└────────┬────────┘      └──────────────────┘
         │
         ▼
    ┌────────────┐
    │ ESCs & Motors
    └────────────┘
```

---

## API Integration

### Betaflight Configuration Compatibility

The system provides wrapper functions for Betaflight-style configuration:

```cpp
// Example: Load Betaflight profile
bf_status_t loadBetaflightProfile(const char* profile_name);

// Example: Get/Set PID values (Betaflight format)
void setBetaflightPID(uint8_t axis, float P, float I, float D);
void getBetaflightPID(uint8_t axis, float* P, float* I, float* D);

// Example: Configure receiver
bf_status_t configureBetaflightReceiver(receiver_protocol_t protocol);
```

### MSP Command Support

| Command | ID | Status | Description |
|---------|----|----|-------------|
| MSP_IDENT | 100 | ✅ | Board identification |
| MSP_STATUS | 101 | ✅ | System status |
| MSP_RAW_IMU | 102 | ✅ | Raw IMU data |
| MSP_SERVO | 103 | ✅ | Servo position data |
| MSP_MOTOR | 104 | ✅ | Motor command data |
| MSP_RC | 105 | ✅ | RC channel values |
| MSP_RAW_GPS | 106 | ⚠️ | GPS data (partial) |
| MSP_COMP_GPS | 107 | ⏳ | Computed GPS (planned) |
| MSP_ATTITUDE | 108 | ✅ | Attitude angles |
| MSP_ALTITUDE | 109 | ⚠️ | Altitude (pending baro) |
| MSP_ANALOG | 110 | ✅ | Battery voltage, signal |
| MSP_RC_TUNING | 111 | ⏳ | RC tuning (planned) |
| MSP_PID | 112 | ⚠️ | PID values (partial) |
| MSP_SELECT_SETTING | 250 | ⏳ | Settings selection (planned) |
| MSP_SET_HEAD | 211 | ⏳ | Set heading (planned) |

---

## Configuration Requirements

### Essential Configuration Items

#### 1. Receiver Configuration
```
receiver_protocol: SBUS
receiver_port: UART1
stick_min: 1000
stick_max: 2000
stick_center: 1500
```

#### 2. Motor Configuration
```
motor_output: DShot600
motor_count: 4
motor_order: X-quad (standard)
motor_poles: 12
```

#### 3. Sensor Configuration
```
imu_type: MPU6050
imu_address: 0x68
acc_calibration: [auto/manual]
gyro_calibration: [auto/manual]
mag_enable: true/false
```

#### 4. Flight Dynamics Configuration
```
pid_profile: [Betaflight, Hybrid, Custom]
level_mode_enabled: true/false
horizon_mode_enabled: true/false
acro_mode_enabled: true/false
max_angle: 50
```

### Configuration File Format

Configuration can be stored in two formats:

**Format 1: Betaflight-compatible**
```
set imu_gyro_lpf = 256HZ
set gyro_fir_enable = ON
set imu_acc_lpf = 10HZ
set p_pitch = 41
set i_pitch = 35
set d_pitch = 23
```

**Format 2: JSON (Hybrid)**
```json
{
  "imu": {
    "gyro_lpf": "256HZ",
    "acc_lpf": "10HZ"
  },
  "pid": {
    "pitch": {"p": 41, "i": 35, "d": 23},
    "roll": {"p": 41, "i": 35, "d": 23},
    "yaw": {"p": 50, "i": 40, "d": 0}
  }
}
```

---

## Testing & Validation

### Unit Tests Required
- [ ] MSP protocol encoding/decoding
- [ ] Configuration parser validation
- [ ] PID controller calculations
- [ ] Sensor data acquisition
- [ ] Motor output mapping

### Integration Tests Required
- [ ] End-to-end communication loop
- [ ] Receiver to motor response time
- [ ] Configuration persistence
- [ ] Multi-sensor fusion
- [ ] Failsafe activation

### Hardware Validation
- [ ] Bench testing with battery
- [ ] Flight testing in acro mode
- [ ] Flight testing in level/horizon modes
- [ ] Failsafe testing
- [ ] Temperature/load testing

### Compatibility Testing
- [ ] Betaflight Configurator connectivity
- [ ] Popular ESC firmware (BLHeli_32, AM32)
- [ ] Common receiver protocols (SBUS, iBUS, PPM)
- [ ] Various IMU sensors

---

## Troubleshooting Guide

### Common Issues

| Issue | Symptom | Solution |
|-------|---------|----------|
| MSP Connection Failed | Configurator can't connect | Check UART baud rate and pin configuration |
| Inverted Controls | Aircraft responds opposite to input | Invert receiver or swap motor wires |
| Oscillations in Flight | Excessive PID-induced vibrations | Reduce P/I gains or increase gyro filter cutoff |
| No IMU Data | Acro mode not working | Verify I2C bus, sensor address, and wiring |
| Motor Sync Issues | Motors don't start cleanly | Check ESC calibration and DShot protocol version |
| Failsafe Doesn't Trigger | Aircraft doesn't land on signal loss | Configure failsafe mode and threshold values |

### Debug Commands

```cpp
// Enable debug output
DEBUG_MODE = 1;

// Check MSP connection
msp_diagnose();

// Verify sensor readings
printIMUDebug();
printReceiverDebug();

// Monitor flight dynamics
printAttitudeDebug();
printMotorDebug();
```

---

## Performance Considerations

### CPU Usage
- **Baseline**: ~15-20% (attitude estimation, sensor reading)
- **With MSP**: +5-10% (protocol processing)
- **Target**: <80% sustained on dual-core ESP32

### Memory Usage
- **Flash**: ~1.2 MB for Betaflight compatibility layer
- **RAM**: ~150 KB runtime data structures
- **Recommended**: Use 8MB flash ESP32 for future expansion

### Latency Targets
- **Sensor to Motor**: <5ms
- **Receiver Input Processing**: <2ms
- **MSP Command Response**: <10ms

---

## Dependencies & Libraries

### Required Libraries
- **Wire.h**: I2C communication
- **SPIFFS/LittleFS**: Configuration storage
- **FreeRTOS**: Task scheduling
- **ESP32 HAL**: Low-level peripheral access

### Recommended Third-Party Libraries
- **MPU6050 Library**: https://github.com/ElectronicCats/mpu6050
- **Adafruit BNO055**: For 9-axis IMU support
- **SerialCommand**: For CLI support

---

## Future Enhancements

### Short-term (2025)
- [ ] Full Betaflight Configurator compatibility
- [ ] OSD support for FPV
- [ ] Advanced telemetry protocols (MAVLink, CRSF)
- [ ] SD card logging

### Medium-term (2026)
- [ ] Advanced flight modes (GPS hold, return to home)
- [ ] Autonomous mission support
- [ ] Machine learning-based PID tuning
- [ ] Multi-aircraft coordination

### Long-term (2027+)
- [ ] Hardware-in-the-loop simulation
- [ ] Advanced AI-assisted tuning
- [ ] Cloud-based configuration management
- [ ] Community plugin system

---

## References & Resources

### Official Documentation
- [Betaflight GitHub](https://github.com/betaflight/betaflight)
- [Betaflight Wiki](https://betaflight.com/docs/wiki)
- [MSP Protocol Specification](https://betaflight.com/docs/wiki/protocol/MSP-V1)

### ESP32 Resources
- [ESP32 Technical Reference](https://espressif.com/en/products/microcontrollers/esp32/resources)
- [Arduino-ESP32 GitHub](https://github.com/espressif/arduino-esp32)
- [PlatformIO Documentation](https://docs.platformio.org)

### Flight Control Theory
- [Attitude Estimation Tutorial](https://www.xsens.com/en/general/attitude)
- [PID Controller Design Guide](https://en.wikipedia.org/wiki/PID_controller)
- [Quadcopter Dynamics](http://papers.nips.cc/paper/5991-quadcopter-dynamics)

---

## Support & Contributions

### Getting Help
- **Issues**: Report bugs on GitHub issues tracker
- **Discussions**: Use GitHub discussions for questions
- **Discord**: Join community Discord server for real-time help

### Contributing
- Fork the repository
- Create feature branch from `HYBRID-NAVIGATION`
- Follow coding standards (see CONTRIBUTING.md)
- Submit pull request with test cases

### Compatibility Feedback
- Document hardware configurations
- Report sensor compatibility issues
- Share tuning profiles
- Contribute test results

---

## Document History

| Version | Date | Author | Changes |
|---------|------|--------|---------|
| 1.0 | 2025-12-29 | hussam-qawaqzeh | Initial documentation creation |
| | | | - Core compatibility status |
| | | | - Implementation roadmap |
| | | | - Integration requirements |
| | | | - API specifications |

---

**Last Updated**: 2025-12-29 17:23:15 UTC

**Maintained By**: hussam-qawaqzeh

**Status**: Active Development
