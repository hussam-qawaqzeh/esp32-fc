#ifndef HYBRID_NAVIGATION_H
#define HYBRID_NAVIGATION_H

#include <Arduino.h>
#include <Wire.h>
#include <vector>
#include <cmath>

/**
 * @file HybridNavigation.h
 * @brief Comprehensive hybrid navigation system combining GPS and IMU data
 * @details Integrates GPS positioning with IMU (accelerometer, gyroscope, magnetometer)
 *          data to provide robust and accurate navigation for autonomous systems.
 * @version 1.0.0
 * @date 2025-12-26
 */

// ==================== Data Structures ====================

/**
 * @struct GPSData
 * @brief GPS positioning information
 */
struct GPSData {
    double latitude;           ///< Latitude in degrees (-90 to 90)
    double longitude;          ///< Longitude in degrees (-180 to 180)
    float altitude;            ///< Altitude in meters above sea level
    float hdop;                ///< Horizontal Dilution of Precision
    float speed_knots;         ///< Ground speed in knots
    float speed_ms;            ///< Ground speed in meters per second
    float track_true;          ///< True track angle in degrees (0-360)
    uint8_t satellites;        ///< Number of satellites in use
    uint8_t fix_type;          ///< Fix type (0=None, 1=GPS, 2=DGPS, 3=PPS, etc.)
    uint32_t timestamp_ms;     ///< Timestamp in milliseconds
    bool valid;                ///< Data validity flag
};

/**
 * @struct IMUData
 * @brief Inertial Measurement Unit data
 */
struct IMUData {
    // Accelerometer data (m/s²)
    float accel_x;
    float accel_y;
    float accel_z;
    
    // Gyroscope data (rad/s)
    float gyro_x;
    float gyro_y;
    float gyro_z;
    
    // Magnetometer data (µT - microTesla)
    float mag_x;
    float mag_y;
    float mag_z;
    
    // Temperature (°C)
    float temperature;
    
    // Timestamp
    uint32_t timestamp_ms;
};

/**
 * @struct Quaternion
 * @brief Quaternion representation for rotation
 */
struct Quaternion {
    float w;  ///< Scalar component
    float x;  ///< Vector component X
    float y;  ///< Vector component Y
    float z;  ///< Vector component Z
    
    Quaternion() : w(1.0f), x(0.0f), y(0.0f), z(0.0f) {}
    Quaternion(float w, float x, float y, float z) 
        : w(w), x(x), y(y), z(z) {}
};

/**
 * @struct EulerAngles
 * @brief Euler angles representation (Roll, Pitch, Yaw)
 */
struct EulerAngles {
    float roll;   ///< Roll angle in radians (-π to π)
    float pitch;  ///< Pitch angle in radians (-π/2 to π/2)
    float yaw;    ///< Yaw angle in radians (-π to π)
    
    EulerAngles() : roll(0.0f), pitch(0.0f), yaw(0.0f) {}
    EulerAngles(float r, float p, float y) 
        : roll(r), pitch(p), yaw(y) {}
};

/**
 * @struct VelocityVector
 * @brief 3D velocity vector
 */
struct VelocityVector {
    float vx;  ///< Velocity in X direction (m/s)
    float vy;  ///< Velocity in Y direction (m/s)
    float vz;  ///< Velocity in Z direction (m/s)
    
    VelocityVector() : vx(0.0f), vy(0.0f), vz(0.0f) {}
    VelocityVector(float x, float y, float z) 
        : vx(x), vy(y), vz(z) {}
};

/**
 * @struct PositionEstimate
 * @brief Fused position estimate combining GPS and IMU
 */
struct PositionEstimate {
    double latitude;           ///< Latitude estimate (degrees)
    double longitude;          ///< Longitude estimate (degrees)
    float altitude;            ///< Altitude estimate (meters)
    VelocityVector velocity;   ///< 3D velocity vector
    EulerAngles attitude;      ///< Attitude (roll, pitch, yaw)
    float position_uncertainty; ///< Position uncertainty (meters)
    float attitude_uncertainty; ///< Attitude uncertainty (radians)
    uint32_t timestamp_ms;     ///< Timestamp
    bool is_valid;             ///< Data validity flag
};

/**
 * @struct NavStatistics
 * @brief Navigation system statistics and diagnostics
 */
struct NavStatistics {
    float gps_quality_factor;  ///< GPS quality (0.0-1.0)
    float imu_confidence;      ///< IMU confidence (0.0-1.0)
    uint32_t gps_updates;      ///< Number of GPS updates received
    uint32_t imu_updates;      ///< Number of IMU updates received
    uint32_t fusion_cycles;    ///< Number of fusion cycles executed
    float process_time_ms;     ///< Time to process last fusion (ms)
    float cpu_load_percent;    ///< CPU load percentage
};

// ==================== Main HybridNavigation Class ====================

/**
 * @class HybridNavigation
 * @brief Hybrid navigation system combining GPS and IMU data fusion
 * @details Uses complementary filtering and extended Kalman filter techniques
 *          to fuse GPS and IMU data for robust navigation estimates.
 */
class HybridNavigation {
public:
    /**
     * @enum FusionMethod
     * @brief Available data fusion methods
     */
    enum class FusionMethod {
        COMPLEMENTARY_FILTER = 0,  ///< Complementary filtering
        KALMAN_FILTER = 1,          ///< Kalman filtering
        EXTENDED_KALMAN_FILTER = 2  ///< Extended Kalman filtering
    };

    // ==================== Constructor & Initialization ====================
    
    /**
     * @brief Constructor
     */
    HybridNavigation();
    
    /**
     * @brief Initialize the hybrid navigation system
     * @param fusion_method Fusion method to use
     * @param imu_sample_rate IMU sampling rate in Hz (default: 100)
     * @param gps_sample_rate GPS sampling rate in Hz (default: 10)
     * @return true if initialization successful, false otherwise
     */
    bool begin(FusionMethod fusion_method = FusionMethod::EXTENDED_KALMAN_FILTER,
               uint16_t imu_sample_rate = 100,
               uint16_t gps_sample_rate = 10);
    
    /**
     * @brief Deinitialize the system
     */
    void end();
    
    /**
     * @brief Check if system is initialized
     * @return true if initialized
     */
    bool isInitialized() const { return initialized; }

    // ==================== GPS Interface ====================
    
    /**
     * @brief Update with new GPS data
     * @param gps_data GPS data structure
     * @return true if data accepted, false otherwise
     */
    bool updateGPSData(const GPSData& gps_data);
    
    /**
     * @brief Get last GPS data received
     * @return Reference to last GPS data
     */
    const GPSData& getGPSData() const { return last_gps_data; }
    
    /**
     * @brief Check if GPS fix is valid
     * @return true if GPS fix is valid
     */
    bool isGPSValid() const;
    
    /**
     * @brief Get GPS quality factor (0.0-1.0)
     * @return Quality factor based on HDOP and satellite count
     */
    float getGPSQuality() const;

    // ==================== IMU Interface ====================
    
    /**
     * @brief Update with new IMU data
     * @param imu_data IMU data structure
     * @return true if data accepted, false otherwise
     */
    bool updateIMUData(const IMUData& imu_data);
    
    /**
     * @brief Get last IMU data received
     * @return Reference to last IMU data
     */
    const IMUData& getIMUData() const { return last_imu_data; }
    
    /**
     * @brief Calibrate IMU accelerometer
     * @param num_samples Number of samples for calibration (default: 1000)
     * @return true if calibration successful
     */
    bool calibrateAccelerometer(uint16_t num_samples = 1000);
    
    /**
     * @brief Calibrate IMU gyroscope
     * @param num_samples Number of samples for calibration (default: 1000)
     * @return true if calibration successful
     */
    bool calibrateGyroscope(uint16_t num_samples = 1000);
    
    /**
     * @brief Calibrate magnetometer
     * @param num_samples Number of samples for calibration (default: 1000)
     * @return true if calibration successful
     */
    bool calibrateMagnetometer(uint16_t num_samples = 1000);

    // ==================== Data Fusion & Processing ====================
    
    /**
     * @brief Perform data fusion and update position estimate
     * @return true if fusion successful
     */
    bool update();
    
    /**
     * @brief Set the fusion method
     * @param method Fusion method to use
     */
    void setFusionMethod(FusionMethod method) { fusion_method = method; }
    
    /**
     * @brief Get current fusion method
     * @return Current fusion method
     */
    FusionMethod getFusionMethod() const { return fusion_method; }
    
    /**
     * @brief Get the latest position estimate
     * @return Reference to position estimate
     */
    const PositionEstimate& getPositionEstimate() const { 
        return position_estimate; 
    }
    
    /**
     * @brief Get attitude (Euler angles)
     * @return Euler angles structure
     */
    EulerAngles getAttitude() const;
    
    /**
     * @brief Get velocity vector
     * @return Velocity vector structure
     */
    VelocityVector getVelocity() const;
    
    /**
     * @brief Get current orientation as quaternion
     * @return Quaternion representing current orientation
     */
    Quaternion getQuaternion() const;

    // ==================== Filter Configuration ====================
    
    /**
     * @brief Set Kalman filter process noise
     * @param q_accel Acceleration process noise
     * @param q_gyro Gyroscope process noise
     * @param q_mag Magnetometer process noise
     */
    void setProcessNoise(float q_accel, float q_gyro, float q_mag);
    
    /**
     * @brief Set Kalman filter measurement noise
     * @param r_gps GPS measurement noise
     * @param r_accel Accelerometer measurement noise
     * @param r_mag Magnetometer measurement noise
     */
    void setMeasurementNoise(float r_gps, float r_accel, float r_mag);
    
    /**
     * @brief Set complementary filter coefficients
     * @param gps_weight GPS weight (0.0-1.0)
     * @param imu_weight IMU weight (0.0-1.0)
     */
    void setComplementaryWeights(float gps_weight, float imu_weight);

    // ==================== Diagnostic & Statistics ====================
    
    /**
     * @brief Get navigation statistics
     * @return Navigation statistics structure
     */
    const NavStatistics& getStatistics() const { return statistics; }
    
    /**
     * @brief Reset statistics
     */
    void resetStatistics();
    
    /**
     * @brief Get system health status
     * @return Health status (0-100%)
     */
    uint8_t getHealthStatus() const;
    
    /**
     * @brief Print diagnostic information to serial
     */
    void printDiagnostics() const;

    // ==================== Utility Functions ====================
    
    /**
     * @brief Convert quaternion to Euler angles
     * @param q Quaternion
     * @return Euler angles
     */
    static EulerAngles quaternionToEuler(const Quaternion& q);
    
    /**
     * @brief Convert Euler angles to quaternion
     * @param euler Euler angles
     * @return Quaternion
     */
    static Quaternion eulerToQuaternion(const EulerAngles& euler);
    
    /**
     * @brief Normalize a quaternion
     * @param q Quaternion to normalize
     * @return Normalized quaternion
     */
    static Quaternion normalizeQuaternion(const Quaternion& q);
    
    /**
     * @brief Calculate distance between two GPS coordinates (Haversine)
     * @param lat1 Latitude 1 (degrees)
     * @param lon1 Longitude 1 (degrees)
     * @param lat2 Latitude 2 (degrees)
     * @param lon2 Longitude 2 (degrees)
     * @return Distance in meters
     */
    static float haversineDistance(double lat1, double lon1, 
                                   double lat2, double lon2);
    
    /**
     * @brief Calculate bearing between two GPS coordinates
     * @param lat1 Latitude 1 (degrees)
     * @param lon1 Longitude 1 (degrees)
     * @param lat2 Latitude 2 (degrees)
     * @param lon2 Longitude 2 (degrees)
     * @return Bearing in degrees (0-360)
     */
    static float calculateBearing(double lat1, double lon1,
                                  double lat2, double lon2);

    // ==================== Constants ====================
    
    static constexpr float EARTH_RADIUS_M = 6371000.0f;  ///< Earth radius in meters
    static constexpr float DEG_TO_RAD = M_PI / 180.0f;   ///< Degrees to radians
    static constexpr float RAD_TO_DEG = 180.0f / M_PI;   ///< Radians to degrees
    static constexpr float GRAVITY_MS2 = 9.80665f;       ///< Standard gravity

private:
    // ==================== Private Member Variables ====================
    
    bool initialized;
    FusionMethod fusion_method;
    
    // Data storage
    GPSData last_gps_data;
    IMUData last_imu_data;
    PositionEstimate position_estimate;
    
    // Calibration offsets
    struct {
        float accel_x, accel_y, accel_z;
        float gyro_x, gyro_y, gyro_z;
        float mag_x, mag_y, mag_z;
    } calibration_offsets;
    
    // Filter states
    Quaternion orientation_quaternion;
    VelocityVector velocity_state;
    
    // Complementary filter coefficients
    float complementary_gps_weight;
    float complementary_imu_weight;
    
    // Kalman filter parameters
    struct {
        float q_accel, q_gyro, q_mag;
        float r_gps, r_accel, r_mag;
    } filter_params;
    
    // Statistics
    NavStatistics statistics;
    
    // Timing
    uint32_t last_gps_update_time;
    uint32_t last_imu_update_time;
    uint32_t last_fusion_time;
    uint16_t imu_sample_rate_hz;
    uint16_t gps_sample_rate_hz;

    // ==================== Private Methods ====================
    
    /**
     * @brief Initialize filter parameters
     */
    void initializeFilterParameters();
    
    /**
     * @brief Complementary filter fusion
     * @return true if successful
     */
    bool fuseComplementary();
    
    /**
     * @brief Extended Kalman filter fusion
     * @return true if successful
     */
    bool fuseExtendedKalman();
    
    /**
     * @brief Update attitude from IMU gyroscope data
     * @param dt Time delta in seconds
     */
    void updateAttitudeFromGyro(float dt);
    
    /**
     * @brief Update attitude from accelerometer and magnetometer
     */
    void updateAttitudeFromAccelMag();
    
    /**
     * @brief Calculate velocity from GPS data
     */
    void calculateVelocityFromGPS();
    
    /**
     * @brief Validate sensor data
     * @return true if data is valid
     */
    bool validateSensorData();
    
    /**
     * @brief Calculate position uncertainty
     */
    void updatePositionUncertainty();
};

#endif // HYBRID_NAVIGATION_H
