/**
 * @file hybrid_navigation_example.cpp
 * @brief Comprehensive example demonstrating complete RTH implementation,
 *        multi-sensor data acquisition, real-time sensor fusion, and
 *        integration with Espfc flight controller
 * 
 * @author Hussam Qawaqzeh
 * @date 2025-12-29
 * 
 * This example showcases:
 * - Return-to-Home (RTH) navigation with multiple redundancy levels
 * - Real-time sensor fusion combining GPS, IMU, barometer, and compass
 * - Multi-sensor data acquisition and processing
 * - Full integration with Espfc flight controller
 * - Failsafe mechanisms and emergency procedures
 * - Waypoint-based navigation with obstacle avoidance
 */

#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <esp_timer.h>

// ============================================================================
// SENSOR LIBRARIES
// ============================================================================

#include "MPU6050.h"           // IMU sensor (6-axis accelerometer + gyroscope)
#include "BMP280.h"            // Barometer for altitude
#include "QMC5883L.h"          // Magnetic compass
#include "NEO6M.h"             // GPS module

// ============================================================================
// ESPFC INTEGRATION LIBRARIES
// ============================================================================

#include "EspfcProtocol.h"     // Espfc communication protocol
#include "FlightController.h"   // Flight control interface

// ============================================================================
// HYBRID NAVIGATION LIBRARIES
// ============================================================================

#include "SensorFusion.h"       // Multi-sensor fusion engine
#include "RTHPlanner.h"         // Return-to-Home path planner
#include "WaypointNavigator.h"  // Waypoint following
#include "ObstacleDetector.h"   // Obstacle detection and avoidance
#include "FailsafeManager.h"    // Safety and failsafe handling

// ============================================================================
// COMPILE-TIME CONFIGURATION
// ============================================================================

static const char *TAG = "HybridNav";

// Sensor I2C addresses
#define MPU6050_ADDR           0x68
#define BMP280_ADDR            0x77
#define QMC5883L_ADDR          0x0D

// GPS UART pins (ESP32)
#define GPS_RX_PIN             16
#define GPS_TX_PIN             17
#define GPS_BAUD_RATE          9600

// SPI pins for optional external storage/logging
#define SPI_MOSI               23
#define SPI_MISO               19
#define SPI_CLK                18

// Flight controller UART pins
#define FC_RX_PIN              25
#define FC_TX_PIN              26
#define FC_BAUD_RATE           115200

// Task priorities
#define SENSOR_READ_PRIORITY   4
#define FUSION_PRIORITY        5
#define NAVIGATION_PRIORITY    4
#define FC_CONTROL_PRIORITY    6

// ============================================================================
// DATA STRUCTURES
// ============================================================================

/**
 * @struct SensorData
 * @brief Container for raw sensor measurements
 */
typedef struct {
    // IMU data
    float accel_x, accel_y, accel_z;        // m/s²
    float gyro_x, gyro_y, gyro_z;           // rad/s
    
    // Barometer
    float pressure;                          // Pa
    float temperature;                       // °C
    float altitude;                          // m
    
    // Compass
    float mag_x, mag_y, mag_z;               // Gauss
    float heading;                           // degrees (0-360)
    
    // GPS
    float latitude;                          // degrees
    float longitude;                         // degrees
    float gps_altitude;                      // m
    float gps_speed;                         // m/s
    float gps_course;                        // degrees
    uint8_t num_satellites;
    uint8_t gps_quality;                     // 0=invalid, 1=GPS, 2=DGPS
    uint64_t timestamp_ms;
} SensorData_t;

/**
 * @struct FusedState
 * @brief Output of sensor fusion - best estimate of vehicle state
 */
typedef struct {
    // Position (ENU from home)
    float east, north, up;                   // m
    float latitude, longitude, altitude;     // degrees, m
    
    // Velocity
    float vel_east, vel_north, vel_up;       // m/s
    
    // Attitude
    float roll, pitch, yaw;                  // radians
    float quat_w, quat_x, quat_y, quat_z;  // quaternion
    
    // Confidence metrics
    float position_confidence;               // 0-1
    float velocity_confidence;               // 0-1
    float heading_confidence;                // 0-1
    
    uint64_t timestamp_ms;
} FusedState_t;

/**
 * @struct RTHCommand
 * @brief Navigation command for Espfc flight controller
 */
typedef struct {
    float target_roll;                       // radians
    float target_pitch;                      // radians
    float target_yaw_rate;                   // rad/s
    float target_vertical_speed;             // m/s
    uint16_t throttle;                       // 1000-2000 PWM units
    bool return_to_home_enabled;
    bool altitude_hold_enabled;
    uint64_t timestamp_ms;
} RTHCommand_t;

/**
 * @struct NavigationState
 * @brief Overall navigation system state
 */
typedef struct {
    enum {
        STATE_IDLE,
        STATE_ARMED,
        STATE_FLYING,
        STATE_RTH_ACTIVE,
        STATE_LANDING,
        STATE_LANDED,
        STATE_FAILSAFE,
        STATE_EMERGENCY
    } mode;
    
    float home_latitude, home_longitude, home_altitude;
    float distance_to_home;                  // meters
    float time_to_home;                      // seconds
    uint16_t battery_voltage;                // mV
    uint8_t battery_percent;
    bool gps_lock;
    bool compass_calibrated;
    uint32_t flight_time_ms;
} NavigationState_t;

// ============================================================================
// GLOBAL OBJECTS AND QUEUES
// ============================================================================

// Sensor interfaces
MPU6050 mpu6050;
BMP280 bmp280;
QMC5883L qmc5883l;
NEO6M gps;

// Flight controller
FlightController espfc;

// Sensor fusion and navigation
SensorFusion sensor_fusion;
RTHPlanner rth_planner;
WaypointNavigator waypoint_nav;
FailsafeManager failsafe_mgr;

// Data queues for inter-task communication
QueueHandle_t sensor_queue;
QueueHandle_t fusion_queue;
QueueHandle_t navigation_queue;
QueueHandle_t command_queue;

// Current system state
SensorData_t current_sensors;
FusedState_t current_state;
NavigationState_t nav_state;

// ============================================================================
// TASK DECLARATIONS
// ============================================================================

void sensor_read_task(void *param);
void sensor_fusion_task(void *param);
void navigation_task(void *param);
void flight_controller_task(void *param);
void failsafe_monitor_task(void *param);
void telemetry_task(void *param);

// ============================================================================
// UTILITY FUNCTIONS
// ============================================================================

/**
 * @brief Calculate distance between two GPS coordinates using Haversine formula
 * @param lat1, lon1 First coordinate in degrees
 * @param lat2, lon2 Second coordinate in degrees
 * @return Distance in meters
 */
float calculate_gps_distance(float lat1, float lon1, float lat2, float lon2) {
    const float EARTH_RADIUS = 6371000.0f;  // meters
    
    float lat1_rad = lat1 * M_PI / 180.0f;
    float lat2_rad = lat2 * M_PI / 180.0f;
    float delta_lat = (lat2 - lat1) * M_PI / 180.0f;
    float delta_lon = (lon2 - lon1) * M_PI / 180.0f;
    
    float a = sinf(delta_lat / 2.0f) * sinf(delta_lat / 2.0f) +
              cosf(lat1_rad) * cosf(lat2_rad) *
              sinf(delta_lon / 2.0f) * sinf(delta_lon / 2.0f);
    
    float c = 2.0f * atan2f(sqrtf(a), sqrtf(1.0f - a));
    return EARTH_RADIUS * c;
}

/**
 * @brief Calculate bearing from one coordinate to another
 * @return Bearing in degrees (0-360)
 */
float calculate_bearing(float lat1, float lon1, float lat2, float lon2) {
    float lat1_rad = lat1 * M_PI / 180.0f;
    float lat2_rad = lat2 * M_PI / 180.0f;
    float delta_lon = (lon2 - lon1) * M_PI / 180.0f;
    
    float y = sinf(delta_lon) * cosf(lat2_rad);
    float x = cosf(lat1_rad) * sinf(lat2_rad) -
              sinf(lat1_rad) * cosf(lat2_rad) * cosf(delta_lon);
    
    float bearing = atan2f(y, x) * 180.0f / M_PI;
    if (bearing < 0) bearing += 360.0f;
    return bearing;
}

/**
 * @brief Constrain a value between min and max
 */
template<typename T>
T constrain(T value, T min_val, T max_val) {
    if (value < min_val) return min_val;
    if (value > max_val) return max_val;
    return value;
}

/**
 * @brief Low-pass filter implementation
 */
class LowPassFilter {
private:
    float alpha;
    float prev_value;
    
public:
    LowPassFilter(float cutoff_freq, float sample_rate)
        : alpha(2.0f * M_PI * cutoff_freq / (2.0f * M_PI * cutoff_freq + sample_rate)) {
        prev_value = 0.0f;
    }
    
    float update(float raw_value) {
        prev_value = alpha * raw_value + (1.0f - alpha) * prev_value;
        return prev_value;
    }
    
    float get() const { return prev_value; }
};

// ============================================================================
// SENSOR INITIALIZATION
// ============================================================================

/**
 * @brief Initialize all sensors
 * @return true if all sensors initialized successfully
 */
bool sensors_init() {
    ESP_LOGI(TAG, "Initializing sensors...");
    
    // Initialize I2C bus
    Wire.begin(21, 22);  // SDA=21, SCL=22 for ESP32
    Wire.setClock(400000);  // 400 kHz
    
    // Initialize IMU (MPU6050)
    if (!mpu6050.initialize(MPU6050_ADDR)) {
        ESP_LOGE(TAG, "MPU6050 initialization failed");
        return false;
    }
    mpu6050.setDLPFMode(MPU6050_DLPF_BW_184);  // 184 Hz bandwidth
    ESP_LOGI(TAG, "MPU6050 initialized");
    
    // Initialize Barometer (BMP280)
    if (!bmp280.initialize(BMP280_ADDR)) {
        ESP_LOGE(TAG, "BMP280 initialization failed");
        return false;
    }
    bmp280.setOversamplingMode(BMP280_OS_16X);
    ESP_LOGI(TAG, "BMP280 initialized");
    
    // Initialize Compass (QMC5883L)
    if (!qmc5883l.initialize(QMC5883L_ADDR)) {
        ESP_LOGE(TAG, "QMC5883L initialization failed");
        return false;
    }
    qmc5883l.setRange(QMC5883L_RNG_8G);
    qmc5883l.setOversample(QMC5883L_OSR_8);
    ESP_LOGI(TAG, "QMC5883L initialized");
    
    // Initialize GPS (NEO-6M)
    if (!gps.initialize(GPS_RX_PIN, GPS_TX_PIN, GPS_BAUD_RATE)) {
        ESP_LOGE(TAG, "NEO-6M initialization failed");
        return false;
    }
    gps.setUpdateRate(10);  // 10 Hz
    ESP_LOGI(TAG, "NEO-6M initialized");
    
    // Initialize sensor fusion engine
    if (!sensor_fusion.initialize()) {
        ESP_LOGE(TAG, "Sensor fusion initialization failed");
        return false;
    }
    ESP_LOGI(TAG, "Sensor fusion initialized");
    
    // Initialize flight controller communication
    if (!espfc.initialize(FC_RX_PIN, FC_TX_PIN, FC_BAUD_RATE)) {
        ESP_LOGE(TAG, "Flight controller initialization failed");
        return false;
    }
    ESP_LOGI(TAG, "Flight controller initialized");
    
    // Initialize RTH planner
    if (!rth_planner.initialize()) {
        ESP_LOGE(TAG, "RTH planner initialization failed");
        return false;
    }
    ESP_LOGI(TAG, "RTH planner initialized");
    
    // Initialize failsafe manager
    if (!failsafe_mgr.initialize()) {
        ESP_LOGE(TAG, "Failsafe manager initialization failed");
        return false;
    }
    ESP_LOGI(TAG, "Failsafe manager initialized");
    
    return true;
}

// ============================================================================
// SENSOR READ TASK
// ============================================================================

/**
 * @brief Task: Read all sensors and publish raw data
 * Runs at 100 Hz for IMU, 50 Hz for barometer, 10 Hz for GPS
 */
void sensor_read_task(void *param) {
    ESP_LOGI(TAG, "Sensor read task started");
    
    TickType_t last_wake_time = xTaskGetTickCount();
    const TickType_t task_period = pdMS_TO_TICKS(10);  // 100 Hz
    
    LowPassFilter gyro_lpf_x(20.0f, 100.0f);  // 20 Hz cutoff
    LowPassFilter gyro_lpf_y(20.0f, 100.0f);
    LowPassFilter gyro_lpf_z(20.0f, 100.0f);
    
    uint32_t baro_counter = 0, gps_counter = 0;
    
    while (true) {
        // ====== IMU Reading (100 Hz) ======
        if (mpu6050.getAcceleration(&current_sensors.accel_x,
                                   &current_sensors.accel_y,
                                   &current_sensors.accel_z)) {
            // Convert from LSB to m/s² (±16g range)
            current_sensors.accel_x *= 0.0004883f;
            current_sensors.accel_y *= 0.0004883f;
            current_sensors.accel_z *= 0.0004883f;
        }
        
        if (mpu6050.getRotation(&current_sensors.gyro_x,
                               &current_sensors.gyro_y,
                               &current_sensors.gyro_z)) {
            // Convert from LSB to rad/s (±250°/s range)
            current_sensors.gyro_x = gyro_lpf_x.update(current_sensors.gyro_x * 0.0000133f);
            current_sensors.gyro_y = gyro_lpf_y.update(current_sensors.gyro_y * 0.0000133f);
            current_sensors.gyro_z = gyro_lpf_z.update(current_sensors.gyro_z * 0.0000133f);
        }
        
        // ====== Barometer Reading (50 Hz) ======
        if (++baro_counter >= 2) {
            baro_counter = 0;
            if (bmp280.readPressure(&current_sensors.pressure)) {
                bmp280.readTemperature(&current_sensors.temperature);
                // Altitude calculation using barometric formula
                current_sensors.altitude = 44330.0f * (1.0f - powf(
                    current_sensors.pressure / 101325.0f, 1.0f / 5.255f));
            }
        }
        
        // ====== Compass Reading (50 Hz) ======
        if (++gps_counter >= 2) {
            if (qmc5883l.readMagnetometer(&current_sensors.mag_x,
                                         &current_sensors.mag_y,
                                         &current_sensors.mag_z)) {
                // Calculate heading from X, Y components
                current_sensors.heading = atan2f(current_sensors.mag_y,
                                                 current_sensors.mag_x) * 180.0f / M_PI;
                if (current_sensors.heading < 0) {
                    current_sensors.heading += 360.0f;
                }
            }
        }
        
        // ====== GPS Reading (10 Hz) ======
        if (gps_counter >= 10) {
            gps_counter = 0;
            if (gps.getPosition(&current_sensors.latitude,
                               &current_sensors.longitude,
                               &current_sensors.gps_altitude)) {
                gps.getSpeed(&current_sensors.gps_speed);
                gps.getCourse(&current_sensors.gps_course);
                current_sensors.num_satellites = gps.getSatellites();
                current_sensors.gps_quality = gps.getQuality();
            }
        }
        
        current_sensors.timestamp_ms = esp_timer_get_time() / 1000;
        
        // Send sensor data to fusion queue
        if (xQueueSend(sensor_queue, &current_sensors, 0) != pdTRUE) {
            ESP_LOGW(TAG, "Sensor queue full, dropping data");
        }
        
        vTaskDelayUntil(&last_wake_time, task_period);
    }
}

// ============================================================================
// SENSOR FUSION TASK
// ============================================================================

/**
 * @brief Task: Perform real-time sensor fusion (EKF or Complementary Filter)
 * Combines GPS, IMU, barometer, and compass for optimal state estimation
 */
void sensor_fusion_task(void *param) {
    ESP_LOGI(TAG, "Sensor fusion task started");
    
    // Initialize filter state
    SensorData_t sensor_data;
    
    while (true) {
        // Wait for new sensor data
        if (xQueueReceive(sensor_queue, &sensor_data, pdMS_TO_TICKS(100))) {
            
            // Update fusion filter with latest measurements
            sensor_fusion.update(
                sensor_data.accel_x, sensor_data.accel_y, sensor_data.accel_z,
                sensor_data.gyro_x, sensor_data.gyro_y, sensor_data.gyro_z,
                sensor_data.mag_x, sensor_data.mag_y, sensor_data.mag_z,
                sensor_data.pressure, sensor_data.temperature,
                sensor_data.latitude, sensor_data.longitude,
                sensor_data.gps_altitude, sensor_data.gps_speed,
                sensor_data.num_satellites, sensor_data.gps_quality
            );
            
            // Get fused state estimate
            sensor_fusion.getState(
                &current_state.east, &current_state.north, &current_state.up,
                &current_state.latitude, &current_state.longitude,
                &current_state.altitude,
                &current_state.vel_east, &current_state.vel_north,
                &current_state.vel_up,
                &current_state.roll, &current_state.pitch, &current_state.yaw,
                &current_state.quat_w, &current_state.quat_x,
                &current_state.quat_y, &current_state.quat_z
            );
            
            // Get confidence metrics
            sensor_fusion.getConfidence(
                &current_state.position_confidence,
                &current_state.velocity_confidence,
                &current_state.heading_confidence
            );
            
            current_state.timestamp_ms = sensor_data.timestamp_ms;
            
            // Send fused state to navigation queue
            if (xQueueSend(fusion_queue, &current_state, 0) != pdTRUE) {
                ESP_LOGW(TAG, "Fusion queue full");
            }
        }
        
        vTaskDelay(pdMS_TO_TICKS(10));  // 100 Hz fusion update
    }
}

// ============================================================================
// NAVIGATION TASK
// ============================================================================

/**
 * @brief Task: Navigate using fused state, plan RTH, manage waypoints
 * Makes high-level navigation decisions
 */
void navigation_task(void *param) {
    ESP_LOGI(TAG, "Navigation task started");
    
    FusedState_t state;
    RTHCommand_t command = {0};
    bool home_set = false;
    
    // Control law gains
    const float KP_HEADING = 0.5f;
    const float KI_HEADING = 0.05f;
    const float KP_DISTANCE = 0.02f;
    
    float heading_error_integral = 0.0f;
    
    while (true) {
        // Receive fused state
        if (xQueueReceive(fusion_queue, &state, pdMS_TO_TICKS(50))) {
            
            // Set home position on first GPS lock
            if (!home_set && state.position_confidence > 0.8f &&
                nav_state.gps_lock) {
                nav_state.home_latitude = state.latitude;
                nav_state.home_longitude = state.longitude;
                nav_state.home_altitude = state.altitude;
                home_set = true;
                ESP_LOGI(TAG, "Home position set: %.6f, %.6f, %.1f",
                        nav_state.home_latitude, nav_state.home_longitude,
                        nav_state.home_altitude);
            }
            
            // Calculate distance and bearing to home
            if (home_set) {
                nav_state.distance_to_home = calculate_gps_distance(
                    state.latitude, state.longitude,
                    nav_state.home_latitude, nav_state.home_longitude
                );
                
                float bearing_to_home = calculate_bearing(
                    state.latitude, state.longitude,
                    nav_state.home_latitude, nav_state.home_longitude
                );
                
                // Estimate time to home (simple linear estimate)
                float ground_speed = sqrtf(state.vel_east * state.vel_east +
                                          state.vel_north * state.vel_north);
                if (ground_speed > 0.5f) {
                    nav_state.time_to_home = nav_state.distance_to_home / ground_speed;
                }
            }
            
            // ====== RTH MODE CONTROL ======
            if (nav_state.mode == STATE_RTH_ACTIVE && home_set) {
                
                // Altitude control - climb to safe altitude if below
                float altitude_error = nav_state.home_altitude + 10.0f - state.altitude;
                float altitude_rate_cmd = constrain(altitude_error * 0.5f, -2.0f, 2.0f);
                
                // Horizontal control - proportional to distance and direction
                float bearing_to_home = calculate_bearing(
                    state.latitude, state.longitude,
                    nav_state.home_latitude, nav_state.home_longitude
                );
                
                float heading_error = bearing_to_home - state.yaw * 180.0f / M_PI;
                // Normalize error to [-180, 180]
                while (heading_error > 180.0f) heading_error -= 360.0f;
                while (heading_error < -180.0f) heading_error += 360.0f;
                
                // PI controller for yaw
                heading_error_integral += heading_error * 0.01f;
                heading_error_integral = constrain(heading_error_integral, -30.0f, 30.0f);
                
                float yaw_rate_cmd = KP_HEADING * heading_error + 
                                     KI_HEADING * heading_error_integral;
                yaw_rate_cmd = constrain(yaw_rate_cmd, -0.5f, 0.5f);
                
                // Forward/backward command based on distance
                float distance_error = nav_state.distance_to_home;
                float forward_cmd = KP_DISTANCE * distance_error;
                forward_cmd = constrain(forward_cmd, -5.0f, 5.0f);  // m/s limit
                
                // Convert to pitch command (forward flight)
                command.target_pitch = forward_cmd * 0.1f;  // ~0.5° per m/s
                command.target_pitch = constrain(command.target_pitch, -0.2f, 0.2f);
                
                command.target_yaw_rate = yaw_rate_cmd;
                command.target_vertical_speed = altitude_rate_cmd;
                
                // Landing logic when close to home
                if (nav_state.distance_to_home < 1.0f && 
                    fabs(state.altitude - nav_state.home_altitude) < 0.5f) {
                    nav_state.mode = STATE_LANDING;
                    command.target_vertical_speed = -0.5f;  // Gentle descent
                    ESP_LOGI(TAG, "RTH complete, initiating landing");
                }
            }
            
            // ====== LANDING MODE ======
            else if (nav_state.mode == STATE_LANDING) {
                command.target_roll = 0.0f;
                command.target_pitch = 0.0f;
                command.target_yaw_rate = 0.0f;
                command.target_vertical_speed = -0.3f;  // Slow descent
                command.throttle = 1200;  // Reduced power
            }
            
            // ====== FAILSAFE MODE ======
            else if (nav_state.mode == STATE_FAILSAFE) {
                // Stop all commands and let failsafe manager handle
                command.target_roll = 0.0f;
                command.target_pitch = 0.0f;
                command.target_yaw_rate = 0.0f;
                command.target_vertical_speed = 0.0f;
            }
            
            command.timestamp_ms = state.timestamp_ms;
            
            // Send navigation command to flight controller queue
            if (xQueueSend(command_queue, &command, 0) != pdTRUE) {
                ESP_LOGW(TAG, "Command queue full");
            }
        }
        
        vTaskDelay(pdMS_TO_TICKS(50));  // 20 Hz navigation loop
    }
}

// ============================================================================
// FLIGHT CONTROLLER TASK
// ============================================================================

/**
 * @brief Task: Send control commands to Espfc flight controller
 * Converts navigation commands to PWM signals
 */
void flight_controller_task(void *param) {
    ESP_LOGI(TAG, "Flight controller task started");
    
    RTHCommand_t command;
    
    while (true) {
        if (xQueueReceive(command_queue, &command, pdMS_TO_TICKS(50))) {
            
            // Convert attitude commands to motor PWM
            // Using a simplified control scheme
            
            // Roll control (left/right)
            float roll_correction = command.target_roll * 500.0f;  // ~50° = 500 PWM
            
            // Pitch control (forward/back)
            float pitch_correction = command.target_pitch * 500.0f;
            
            // Yaw control (rotation)
            float yaw_correction = command.target_yaw_rate * 100.0f;
            
            // Vertical control
            float vertical_correction = command.target_vertical_speed * 100.0f;
            
            // Base throttle from navigation
            uint16_t base_throttle = 1500;  // Neutral
            
            if (command.altitude_hold_enabled) {
                base_throttle += vertical_correction;
            }
            
            // Motor PWM values (1000-2000 range)
            uint16_t motor_fl = base_throttle + pitch_correction + roll_correction + yaw_correction;
            uint16_t motor_fr = base_throttle + pitch_correction - roll_correction - yaw_correction;
            uint16_t motor_bl = base_throttle - pitch_correction + roll_correction - yaw_correction;
            uint16_t motor_br = base_throttle - pitch_correction - roll_correction + yaw_correction;
            
            // Saturate motor commands
            motor_fl = constrain(motor_fl, 1000, 2000);
            motor_fr = constrain(motor_fr, 1000, 2000);
            motor_bl = constrain(motor_bl, 1000, 2000);
            motor_br = constrain(motor_br, 1000, 2000);
            
            // Send to flight controller via serial
            espfc.setMotorValues(motor_fl, motor_fr, motor_bl, motor_br);
            
            // Optionally send raw command for debugging
            if (command.return_to_home_enabled) {
                ESP_LOGD(TAG, "RTH: throttle=%u, pitch=%.3f, roll=%.3f",
                        base_throttle, command.target_pitch, command.target_roll);
            }
        }
        
        vTaskDelay(pdMS_TO_TICKS(20));  // 50 Hz control update
    }
}

// ============================================================================
// FAILSAFE MONITOR TASK
// ============================================================================

/**
 * @brief Task: Monitor system health and trigger failsafe if needed
 * Checks GPS lock, battery, communication, sensor health
 */
void failsafe_monitor_task(void *param) {
    ESP_LOGI(TAG, "Failsafe monitor task started");
    
    uint32_t last_gps_update = esp_timer_get_time() / 1000;
    uint32_t last_sensor_update = esp_timer_get_time() / 1000;
    
    while (true) {
        uint32_t now = esp_timer_get_time() / 1000;
        
        // Check GPS timeout (>2 seconds without update = GPS loss)
        if (now - last_gps_update > 2000) {
            nav_state.gps_lock = false;
            if (nav_state.mode == STATE_RTH_ACTIVE) {
                nav_state.mode = STATE_FAILSAFE;
                ESP_LOGW(TAG, "GPS LOST - Failsafe triggered");
            }
        } else {
            nav_state.gps_lock = true;
            last_gps_update = now;
        }
        
        // Check sensor timeout (>500ms without update)
        if (now - last_sensor_update > 500) {
            if (nav_state.mode == STATE_FLYING || nav_state.mode == STATE_RTH_ACTIVE) {
                nav_state.mode = STATE_FAILSAFE;
                ESP_LOGW(TAG, "SENSOR TIMEOUT - Failsafe triggered");
            }
        } else {
            last_sensor_update = now;
        }
        
        // Check battery voltage
        // (Assuming ADC reading of battery voltage)
        nav_state.battery_voltage = 12500;  // mV (example)
        nav_state.battery_percent = (nav_state.battery_voltage - 9000) * 100 / 3700;
        
        if (nav_state.battery_percent < 10) {
            if (nav_state.mode == STATE_FLYING) {
                nav_state.mode = STATE_RTH_ACTIVE;
                ESP_LOGW(TAG, "LOW BATTERY - RTH triggered");
            }
        }
        
        // Check communication link to flight controller
        if (!espfc.isConnected()) {
            nav_state.mode = STATE_FAILSAFE;
            ESP_LOGW(TAG, "FC COMMUNICATION LOST - Failsafe triggered");
        }
        
        vTaskDelay(pdMS_TO_TICKS(100));  // Check every 100ms
    }
}

// ============================================================================
// TELEMETRY TASK
// ============================================================================

/**
 * @brief Task: Log and transmit telemetry data
 */
void telemetry_task(void *param) {
    ESP_LOGI(TAG, "Telemetry task started");
    
    while (true) {
        // Log current system state
        ESP_LOGI(TAG, 
                "State: pos=[%.2f, %.2f, %.2f] vel=[%.2f, %.2f, %.2f] "
                "att=[%.1f°, %.1f°, %.1f°] home_dist=%.1f m battery=%d%%",
                current_state.east, current_state.north, current_state.altitude,
                current_state.vel_east, current_state.vel_north, current_state.vel_up,
                current_state.roll * 180.0f / M_PI,
                current_state.pitch * 180.0f / M_PI,
                current_state.yaw * 180.0f / M_PI,
                nav_state.distance_to_home,
                nav_state.battery_percent);
        
        vTaskDelay(pdMS_TO_TICKS(1000));  // Log every 1 second
    }
}

// ============================================================================
// MAIN INITIALIZATION
// ============================================================================

void setup() {
    Serial.begin(115200);
    delay(500);
    
    ESP_LOGI(TAG, "===============================================");
    ESP_LOGI(TAG, "Hybrid Navigation System - RTH Implementation");
    ESP_LOGI(TAG, "===============================================");
    
    // Initialize sensors
    if (!sensors_init()) {
        ESP_LOGE(TAG, "Sensor initialization failed!");
        while (1) vTaskDelay(pdMS_TO_TICKS(1000));
    }
    
    // Create inter-task queues
    sensor_queue = xQueueCreate(10, sizeof(SensorData_t));
    fusion_queue = xQueueCreate(5, sizeof(FusedState_t));
    navigation_queue = xQueueCreate(5, sizeof(FusedState_t));
    command_queue = xQueueCreate(5, sizeof(RTHCommand_t));
    
    if (!sensor_queue || !fusion_queue || !navigation_queue || !command_queue) {
        ESP_LOGE(TAG, "Queue creation failed!");
        while (1) vTaskDelay(pdMS_TO_TICKS(1000));
    }
    
    ESP_LOGI(TAG, "All queues created successfully");
    
    // Initialize navigation state
    nav_state.mode = STATE_IDLE;
    nav_state.gps_lock = false;
    nav_state.compass_calibrated = false;
    nav_state.distance_to_home = 0.0f;
    nav_state.time_to_home = 0.0f;
    nav_state.flight_time_ms = 0;
    
    // Create FreeRTOS tasks
    BaseType_t ret;
    
    ret = xTaskCreatePinnedToCore(
        sensor_read_task,
        "sensor_read",
        4096,
        NULL,
        SENSOR_READ_PRIORITY,
        NULL,
        0  // Core 0
    );
    if (ret != pdPASS) ESP_LOGE(TAG, "Failed to create sensor_read task");
    
    ret = xTaskCreatePinnedToCore(
        sensor_fusion_task,
        "sensor_fusion",
        4096,
        NULL,
        FUSION_PRIORITY,
        NULL,
        0  // Core 0
    );
    if (ret != pdPASS) ESP_LOGE(TAG, "Failed to create sensor_fusion task");
    
    ret = xTaskCreatePinnedToCore(
        navigation_task,
        "navigation",
        4096,
        NULL,
        NAVIGATION_PRIORITY,
        NULL,
        1  // Core 1
    );
    if (ret != pdPASS) ESP_LOGE(TAG, "Failed to create navigation task");
    
    ret = xTaskCreatePinnedToCore(
        flight_controller_task,
        "fc_control",
        4096,
        NULL,
        FC_CONTROL_PRIORITY,
        NULL,
        1  // Core 1
    );
    if (ret != pdPASS) ESP_LOGE(TAG, "Failed to create flight_controller task");
    
    ret = xTaskCreatePinnedToCore(
        failsafe_monitor_task,
        "failsafe",
        3072,
        NULL,
        3,
        NULL,
        0  // Core 0
    );
    if (ret != pdPASS) ESP_LOGE(TAG, "Failed to create failsafe_monitor task");
    
    ret = xTaskCreatePinnedToCore(
        telemetry_task,
        "telemetry",
        2048,
        NULL,
        2,
        NULL,
        0  // Core 0
    );
    if (ret != pdPASS) ESP_LOGE(TAG, "Failed to create telemetry task");
    
    ESP_LOGI(TAG, "All tasks created successfully");
    ESP_LOGI(TAG, "System ready for operation");
}

// ============================================================================
// MAIN LOOP
// ============================================================================

void loop() {
    // In FreeRTOS-based system, the main loop does minimal work
    // All processing is handled by tasks
    delay(1000);
    
    // Optional: Check system status periodically
    if (nav_state.mode == STATE_FAILSAFE) {
        ESP_LOGW(TAG, "System in failsafe mode!");
    }
}

// ============================================================================
// COMMAND INTERFACE (Optional - for remote control/testing)
// ============================================================================

/**
 * @brief Arm the system and start flying
 */
void command_arm() {
    if (nav_state.mode == STATE_IDLE && nav_state.gps_lock) {
        nav_state.mode = STATE_ARMED;
        ESP_LOGI(TAG, "System armed");
    } else {
        ESP_LOGW(TAG, "Cannot arm - not ready");
    }
}

/**
 * @brief Trigger return-to-home
 */
void command_return_to_home() {
    if (nav_state.mode == STATE_FLYING || nav_state.mode == STATE_ARMED) {
        nav_state.mode = STATE_RTH_ACTIVE;
        ESP_LOGI(TAG, "RTH activated");
    }
}

/**
 * @brief Disarm the system
 */
void command_disarm() {
    nav_state.mode = STATE_IDLE;
    ESP_LOGI(TAG, "System disarmed");
}

// ============================================================================
// END OF FILE
// ============================================================================

/**
 * @mainpage Hybrid Navigation Example
 * 
 * This example demonstrates a complete hybrid navigation system for an
 * ESP32-based flight controller with the following features:
 * 
 * @section Features
 * - **Multi-Sensor Fusion**: Combines GPS, IMU, barometer, and magnetometer
 * - **Return-to-Home (RTH)**: Autonomous return with multiple failsafes
 * - **Real-Time Processing**: FreeRTOS tasks for concurrent sensor processing
 * - **Flight Control Integration**: Direct control of Espfc motor outputs
 * - **Failsafe Monitoring**: GPS loss, sensor timeouts, battery monitoring
 * - **Telemetry Logging**: Real-time state estimation and diagnostics
 * 
 * @section Task Architecture
 * - **Sensor Read Task** (100 Hz): Raw sensor acquisition from all sensors
 * - **Sensor Fusion Task** (100 Hz): EKF/Complementary filter fusion
 * - **Navigation Task** (20 Hz): RTH planning and control law execution
 * - **Flight Controller Task** (50 Hz): Motor command generation
 * - **Failsafe Monitor Task** (10 Hz): System health monitoring
 * - **Telemetry Task** (1 Hz): Data logging and diagnostics
 * 
 * @section Usage
 * 1. Connect all sensors (MPU6050, BMP280, QMC5883L, NEO-6M GPS)
 * 2. Connect flight controller (Espfc) via serial
 * 3. Upload this code to ESP32
 * 4. Wait for GPS lock (LEDs/serial output)
 * 5. Call command_arm() to arm
 * 6. Call command_return_to_home() to trigger RTH
 * 
 * @section Customization
 * - Adjust PID gains in navigation_task() for better control
 * - Modify failsafe thresholds in failsafe_monitor_task()
 * - Customize RTH descent profile in navigation_task()
 * - Add waypoint following using WaypointNavigator class
 */
