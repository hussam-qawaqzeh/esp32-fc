#include "HybridNavigation.h"
#include <cmath>
#include <algorithm>

// Initialize static members
HybridNavigation* HybridNavigation::instance = nullptr;

/**
 * Constructor - Initialize hybrid navigation system
 */
HybridNavigation::HybridNavigation()
    : gpsDataValid(false), imuDataValid(false), navigationInitialized(false) {
    
    // Initialize state vectors
    initializeStateVectors();
    
    // Initialize filter parameters
    initializeFilterParameters();
    
    // Initialize sensor fusion matrices
    initializeSensorFusion();
    
    // Initialize calibration values
    initializeCalibration();
}

/**
 * Initialize state vectors for navigation
 */
void HybridNavigation::initializeStateVectors() {
    // Position state vector [lat, lon, altitude]
    positionState.latitude = 0.0f;
    positionState.longitude = 0.0f;
    positionState.altitude = 0.0f;
    
    // Velocity state vector [north, east, down]
    velocityState.north = 0.0f;
    velocityState.east = 0.0f;
    velocityState.down = 0.0f;
    
    // Attitude state vector [roll, pitch, yaw]
    attitudeState.roll = 0.0f;
    attitudeState.pitch = 0.0f;
    attitudeState.yaw = 0.0f;
    
    // DCM (Direction Cosine Matrix) initialization to identity
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            dcmMatrix[i][j] = (i == j) ? 1.0f : 0.0f;
        }
    }
    
    // Initialize quaternion to identity
    quaternion.q0 = 1.0f;
    quaternion.q1 = 0.0f;
    quaternion.q2 = 0.0f;
    quaternion.q3 = 0.0f;
    
    // Initialize gyro bias estimates
    gyroBias.x = 0.0f;
    gyroBias.y = 0.0f;
    gyroBias.z = 0.0f;
    
    // Initialize accumulator
    deltaVelocity.x = 0.0f;
    deltaVelocity.y = 0.0f;
    deltaVelocity.z = 0.0f;
    
    deltaAngle.x = 0.0f;
    deltaAngle.y = 0.0f;
    deltaAngle.z = 0.0f;
}

/**
 * Initialize filter parameters
 */
void HybridNavigation::initializeFilterParameters() {
    // Complementary filter gains
    complementaryFilterGain = 0.95f;  // Weight for gyro integration
    accelFilterGain = 0.05f;           // Weight for accelerometer correction
    
    // Extended Kalman Filter process noise covariance
    ekfProcessNoise.posX = 0.01f;
    ekfProcessNoise.posY = 0.01f;
    ekfProcessNoise.posZ = 0.01f;
    ekfProcessNoise.velX = 0.05f;
    ekfProcessNoise.velY = 0.05f;
    ekfProcessNoise.velZ = 0.05f;
    
    // Extended Kalman Filter measurement noise covariance
    ekfMeasurementNoise.gps = 5.0f;      // GPS position noise in meters
    ekfMeasurementNoise.accel = 0.1f;    // Accelerometer noise
    ekfMeasurementNoise.gyro = 0.01f;    // Gyroscope noise
    
    // Initialize Kalman gain matrix
    for (int i = 0; i < 9; i++) {
        for (int j = 0; j < 9; j++) {
            kalmanGain[i][j] = (i == j) ? 0.5f : 0.0f;
        }
    }
    
    // Initialize state covariance matrix
    for (int i = 0; i < 9; i++) {
        for (int j = 0; j < 9; j++) {
            stateCovariance[i][j] = (i == j) ? 1.0f : 0.0f;
        }
    }
}

/**
 * Initialize sensor fusion related matrices
 */
void HybridNavigation::initializeSensorFusion() {
    // Initialize measurement matrix H for EKF
    // Maps state space to measurement space
    for (int i = 0; i < 9; i++) {
        for (int j = 0; j < 9; j++) {
            measurementMatrix[i][j] = 0.0f;
        }
    }
    
    // Position measurements from GPS
    measurementMatrix[0][0] = 1.0f;  // Latitude
    measurementMatrix[1][1] = 1.0f;  // Longitude
    measurementMatrix[2][2] = 1.0f;  // Altitude
    
    // Velocity measurements from GPS
    measurementMatrix[3][3] = 1.0f;  // North velocity
    measurementMatrix[4][4] = 1.0f;  // East velocity
    measurementMatrix[5][5] = 1.0f;  // Down velocity
    
    // Attitude measurements from accelerometer
    measurementMatrix[6][6] = 1.0f;  // Roll
    measurementMatrix[7][7] = 1.0f;  // Pitch
    measurementMatrix[8][8] = 1.0f;  // Yaw
    
    // Initialize process noise matrix Q
    for (int i = 0; i < 9; i++) {
        for (int j = 0; j < 9; j++) {
            processNoiseMatrix[i][j] = (i == j) ? 0.001f : 0.0f;
        }
    }
    
    // Initialize measurement noise matrix R
    for (int i = 0; i < 9; i++) {
        for (int j = 0; j < 9; j++) {
            measurementNoiseMatrix[i][j] = 0.0f;
        }
    }
    measurementNoiseMatrix[0][0] = 25.0f;   // GPS latitude noise
    measurementNoiseMatrix[1][1] = 25.0f;   // GPS longitude noise
    measurementNoiseMatrix[2][2] = 25.0f;   // GPS altitude noise
    measurementNoiseMatrix[3][3] = 1.0f;    // Velocity north noise
    measurementNoiseMatrix[4][4] = 1.0f;    // Velocity east noise
    measurementNoiseMatrix[5][5] = 1.0f;    // Velocity down noise
    measurementNoiseMatrix[6][6] = 0.01f;   // Accelerometer noise
    measurementNoiseMatrix[7][7] = 0.01f;
    measurementNoiseMatrix[8][8] = 0.01f;
}

/**
 * Initialize sensor calibration parameters
 */
void HybridNavigation::initializeCalibration() {
    // IMU calibration parameters
    imuCalibration.accelBiasX = 0.0f;
    imuCalibration.accelBiasY = 0.0f;
    imuCalibration.accelBiasZ = 0.0f;
    
    imuCalibration.gyroBiasX = 0.0f;
    imuCalibration.gyroBiasY = 0.0f;
    imuCalibration.gyroBiasZ = 0.0f;
    
    imuCalibration.accelScaleX = 1.0f;
    imuCalibration.accelScaleY = 1.0f;
    imuCalibration.accelScaleZ = 1.0f;
    
    imuCalibration.gyroScaleX = 1.0f;
    imuCalibration.gyroScaleY = 1.0f;
    imuCalibration.gyroScaleZ = 1.0f;
    
    // Magnetometer calibration
    magCalibration.biasX = 0.0f;
    magCalibration.biasY = 0.0f;
    magCalibration.biasZ = 0.0f;
    
    magCalibration.scaleX = 1.0f;
    magCalibration.scaleY = 1.0f;
    magCalibration.scaleZ = 1.0f;
    
    // GPS receiver parameters
    gpsParameters.dilutionOfPrecision = 2.0f;
    gpsParameters.horizontalAccuracy = 5.0f;  // meters
    gpsParameters.verticalAccuracy = 10.0f;   // meters
    gpsParameters.velocityAccuracy = 0.1f;    // m/s
    
    // Navigation system parameters
    navParameters.updateRate = 50;             // Hz
    navParameters.gpsUpdateRate = 10;          // Hz
    navParameters.earthRadius = 6371000.0f;    // meters
    navParameters.flattening = 1.0f / 298.257f;
}

/**
 * Update inertial navigation system with IMU data
 */
void HybridNavigation::updateInertialNavigation(const IMUData& imuData, float deltaTime) {
    if (deltaTime <= 0.0f || deltaTime > 1.0f) {
        return;  // Invalid time delta
    }
    
    // Apply calibration to IMU data
    Vector3f calibratedAccel = applySensorCalibration(imuData.acceleration, 
                                                      imuCalibration, true);
    Vector3f calibratedGyro = applySensorCalibration(imuData.gyroscope, 
                                                     imuCalibration, false);
    
    // Update DCM matrix using gyroscope data
    updateDCMMatrix(calibratedGyro, deltaTime);
    
    // Transform accelerometer data to NED frame
    Vector3f accelNED = transformToNED(calibratedAccel);
    
    // Update velocity using accelerometer measurements
    updateVelocity(accelNED, deltaTime);
    
    // Update position using velocity
    updatePosition(deltaTime);
    
    // Accumulate delta velocity and delta angle
    deltaVelocity.x += calibratedAccel.x * deltaTime;
    deltaVelocity.y += calibratedAccel.y * deltaTime;
    deltaVelocity.z += calibratedAccel.z * deltaTime;
    
    deltaAngle.x += calibratedGyro.x * deltaTime;
    deltaAngle.y += calibratedGyro.y * deltaTime;
    deltaAngle.z += calibratedGyro.z * deltaTime;
    
    imuDataValid = true;
}

/**
 * Apply sensor calibration to IMU data
 */
Vector3f HybridNavigation::applySensorCalibration(const Vector3f& rawData, 
                                                   const IMUCalibration& calib, 
                                                   bool isAccelerometer) {
    Vector3f calibrated;
    
    if (isAccelerometer) {
        calibrated.x = (rawData.x - calib.accelBiasX) * calib.accelScaleX;
        calibrated.y = (rawData.y - calib.accelBiasY) * calib.accelScaleY;
        calibrated.z = (rawData.z - calib.accelBiasZ) * calib.accelScaleZ;
    } else {
        calibrated.x = (rawData.x - calib.gyroBiasX) * calib.gyroScaleX;
        calibrated.y = (rawData.y - calib.gyroBiasY) * calib.gyroScaleY;
        calibrated.z = (rawData.z - calib.gyroBiasZ) * calib.gyroScaleZ;
    }
    
    return calibrated;
}

/**
 * Update DCM matrix using gyroscope measurements
 * Implements skew-symmetric matrix multiplication for attitude update
 */
void HybridNavigation::updateDCMMatrix(const Vector3f& gyroRate, float deltaTime) {
    // Create skew-symmetric cross product matrix
    float wx = gyroRate.x * deltaTime;
    float wy = gyroRate.y * deltaTime;
    float wz = gyroRate.z * deltaTime;
    
    // DCM update: Cn+1 = Cn * (I + [w]×)
    // where [w]× is the skew-symmetric matrix of angular rates
    float dcmUpdate[3][3] = {
        {1.0f, -wz, wy},
        {wz, 1.0f, -wx},
        {-wy, wx, 1.0f}
    };
    
    // Multiply DCM by update matrix
    float tempDCM[3][3];
    matrixMultiply3x3(dcmMatrix, dcmUpdate, tempDCM);
    
    // Normalize DCM to maintain orthogonality
    normalizeDCMMatrix();
    
    // Copy result back
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            dcmMatrix[i][j] = tempDCM[i][j];
        }
    }
}

/**
 * Transform accelerometer data from body frame to NED frame
 */
Vector3f HybridNavigation::transformToNED(const Vector3f& bodyFrame) {
    Vector3f ned;
    
    // Apply DCM transformation: NED = DCM * body
    ned.x = dcmMatrix[0][0] * bodyFrame.x + dcmMatrix[0][1] * bodyFrame.y + dcmMatrix[0][2] * bodyFrame.z;
    ned.y = dcmMatrix[1][0] * bodyFrame.x + dcmMatrix[1][1] * bodyFrame.y + dcmMatrix[1][2] * bodyFrame.z;
    ned.z = dcmMatrix[2][0] * bodyFrame.x + dcmMatrix[2][1] * bodyFrame.y + dcmMatrix[2][2] * bodyFrame.z;
    
    return ned;
}

/**
 * Update velocity state using accelerometer measurements
 */
void HybridNavigation::updateVelocity(const Vector3f& accelNED, float deltaTime) {
    // Subtract gravity from down acceleration
    float gravityAdjustedAccel = accelNED.z + 9.81f;
    
    // Integrate acceleration to get velocity
    velocityState.north += accelNED.x * deltaTime;
    velocityState.east += accelNED.y * deltaTime;
    velocityState.down += gravityAdjustedAccel * deltaTime;
}

/**
 * Update position state using velocity
 * Implements geodetic position update
 */
void HybridNavigation::updatePosition(float deltaTime) {
    // Convert velocity from NED to geodetic rates
    float latitudeRad = degreesToRadians(positionState.latitude);
    float earthRadiusAtLat = navParameters.earthRadius / 
                             sqrt(1.0f - (2.0f * navParameters.flattening - 
                             navParameters.flattening * navParameters.flattening) * 
                             sin(latitudeRad) * sin(latitudeRad));
    
    float dLat = (velocityState.north / (earthRadiusAtLat + positionState.altitude)) * deltaTime;
    float dLon = (velocityState.east / ((earthRadiusAtLat * cos(latitudeRad)) + positionState.altitude)) * deltaTime;
    float dAlt = -velocityState.down * deltaTime;
    
    // Update position
    positionState.latitude += radiansToDegrees(dLat);
    positionState.longitude += radiansToDegrees(dLon);
    positionState.altitude += dAlt;
}

/**
 * Complementary filter for attitude estimation
 * Fuses gyroscope and accelerometer data
 */
void HybridNavigation::complementaryFilter(const IMUData& imuData, float deltaTime) {
    if (deltaTime <= 0.0f) {
        return;
    }
    
    // Calibrate sensors
    Vector3f accel = applySensorCalibration(imuData.acceleration, imuCalibration, true);
    Vector3f gyro = applySensorCalibration(imuData.gyroscope, imuCalibration, false);
    
    // Normalize accelerometer
    float accelMagnitude = sqrt(accel.x * accel.x + accel.y * accel.y + accel.z * accel.z);
    if (accelMagnitude > 0.0f) {
        accel.x /= accelMagnitude;
        accel.y /= accelMagnitude;
        accel.z /= accelMagnitude;
    }
    
    // Get acceleration-based attitude estimates
    float rollFromAccel = atan2(accel.y, accel.z);
    float pitchFromAccel = asin(-accel.x);
    
    // Get yaw from magnetometer (if available)
    float yawFromMag = 0.0f;
    if (imuData.magnetometer.x != 0.0f || imuData.magnetometer.y != 0.0f) {
        // Apply calibration to magnetometer
        Vector3f mag = imuData.magnetometer;
        mag.x = (mag.x - magCalibration.biasX) * magCalibration.scaleX;
        mag.y = (mag.y - magCalibration.biasY) * magCalibration.scaleY;
        mag.z = (mag.z - magCalibration.biasZ) * magCalibration.scaleZ;
        
        yawFromMag = atan2(mag.y, mag.x);
    }
    
    // Complementary filter equations
    // High-pass filter for gyroscope (maintains short-term accuracy)
    // Low-pass filter for accelerometer (corrects long-term drift)
    attitudeState.roll = complementaryFilterGain * (attitudeState.roll + gyro.x * deltaTime) + 
                        accelFilterGain * rollFromAccel;
    attitudeState.pitch = complementaryFilterGain * (attitudeState.pitch + gyro.y * deltaTime) + 
                         accelFilterGain * pitchFromAccel;
    attitudeState.yaw = complementaryFilterGain * (attitudeState.yaw + gyro.z * deltaTime) + 
                       accelFilterGain * yawFromMag;
    
    // Constrain angles to [-pi, pi]
    constrainAngle(attitudeState.roll);
    constrainAngle(attitudeState.pitch);
    constrainAngle(attitudeState.yaw);
}

/**
 * Extended Kalman Filter implementation for sensor fusion
 * State vector: [lat, lon, alt, v_n, v_e, v_d, roll, pitch, yaw]
 */
void HybridNavigation::extendedKalmanFilter(const IMUData& imuData, const GPSData* gpsData, 
                                            float deltaTime) {
    if (deltaTime <= 0.0f) {
        return;
    }
    
    // Predict step
    predictState(imuData, deltaTime);
    predictCovariance(deltaTime);
    
    // Update step with available measurements
    if (gpsData != nullptr && gpsDataValid) {
        updateWithGPSMeasurement(*gpsData);
    }
    
    if (imuDataValid) {
        updateWithIMUMeasurement(imuData);
    }
    
    // Correct state estimate
    correctState();
}

/**
 * EKF prediction step - propagate state forward in time
 */
void HybridNavigation::predictState(const IMUData& imuData, float deltaTime) {
    // Apply sensor calibration
    Vector3f accel = applySensorCalibration(imuData.acceleration, imuCalibration, true);
    Vector3f gyro = applySensorCalibration(imuData.gyroscope, imuCalibration, false);
    
    // Subtract estimated bias from gyro
    gyro.x -= gyroBias.x;
    gyro.y -= gyroBias.y;
    gyro.z -= gyroBias.z;
    
    // Transform acceleration to NED frame
    Vector3f accelNED = transformToNED(accel);
    
    // Position rate (velocity in geodetic frame)
    float latRad = degreesToRadians(positionState.latitude);
    float cosLat = cos(latRad);
    float sinLat = sin(latRad);
    
    float Re = navParameters.earthRadius / 
               sqrt(1.0f - (2.0f * navParameters.flattening - 
               navParameters.flattening * navParameters.flattening) * 
               sinLat * sinLat);
    
    // Update position
    positionState.latitude += (velocityState.north / (Re + positionState.altitude)) * deltaTime * 
                              (180.0f / M_PI);
    positionState.longitude += (velocityState.east / ((Re * cosLat) + positionState.altitude)) * 
                               deltaTime * (180.0f / M_PI);
    positionState.altitude += -velocityState.down * deltaTime;
    
    // Update velocity (with gravity compensation)
    velocityState.north += accelNED.x * deltaTime;
    velocityState.east += accelNED.y * deltaTime;
    velocityState.down += (accelNED.z + 9.81f) * deltaTime;
    
    // Update attitude using gyroscope
    attitudeState.roll += gyro.x * deltaTime;
    attitudeState.pitch += gyro.y * deltaTime;
    attitudeState.yaw += gyro.z * deltaTime;
}

/**
 * EKF covariance prediction step
 * P = F*P*F' + Q
 */
void HybridNavigation::predictCovariance(float deltaTime) {
    // Simplified covariance update
    // In a full implementation, compute Jacobian matrix F and perform P = F*P*F' + Q
    
    // Add process noise to covariance diagonal
    for (int i = 0; i < 9; i++) {
        stateCovariance[i][i] += processNoiseMatrix[i][i] * deltaTime;
        
        // Ensure covariance remains positive definite
        if (stateCovariance[i][i] < 0.0f) {
            stateCovariance[i][i] = 0.001f;
        }
    }
}

/**
 * EKF update step with GPS measurements
 */
void HybridNavigation::updateWithGPSMeasurement(const GPSData& gps) {
    // GPS measurement noise increases with DOP
    float gpsNoise = 5.0f * gps.dilutionOfPrecision;
    
    // Innovation (measurement residual)
    float z_lat = gps.latitude - positionState.latitude;
    float z_lon = gps.longitude - positionState.longitude;
    float z_alt = gps.altitude - positionState.altitude;
    float z_v_n = gps.velocityNorth - velocityState.north;
    float z_v_e = gps.velocityEast - velocityState.east;
    float z_v_d = gps.velocityDown - velocityState.down;
    
    // Kalman gain calculation (simplified)
    float innovationCovariance = stateCovariance[0][0] + gpsNoise * gpsNoise;
    if (innovationCovariance > 0.0f) {
        float kalmanGainGPS = stateCovariance[0][0] / innovationCovariance;
        
        // Update state with innovation
        positionState.latitude += kalmanGainGPS * z_lat;
        positionState.longitude += kalmanGainGPS * z_lon;
        positionState.altitude += kalmanGainGPS * z_alt;
        velocityState.north += kalmanGainGPS * z_v_n;
        velocityState.east += kalmanGainGPS * z_v_e;
        velocityState.down += kalmanGainGPS * z_v_d;
        
        // Update covariance
        stateCovariance[0][0] *= (1.0f - kalmanGainGPS);
    }
}

/**
 * EKF update step with IMU measurements
 */
void HybridNavigation::updateWithIMUMeasurement(const IMUData& imuData) {
    // Normalize accelerometer for attitude correction
    Vector3f accel = imuData.acceleration;
    float accelMag = sqrt(accel.x * accel.x + accel.y * accel.y + accel.z * accel.z);
    
    if (accelMag > 0.0f && fabs(accelMag - 9.81f) < 3.0f) {  // Valid acceleration
        accel.x /= accelMag;
        accel.y /= accelMag;
        accel.z /= accelMag;
        
        // Expected acceleration in body frame
        float expectedAccel[3];
        expectedAccel[0] = sin(attitudeState.pitch);
        expectedAccel[1] = -sin(attitudeState.roll) * cos(attitudeState.pitch);
        expectedAccel[2] = cos(attitudeState.roll) * cos(attitudeState.pitch);
        
        // Accelerometer measurement innovation
        float innovAccel = sqrt(pow(accel.x - expectedAccel[0], 2) +
                               pow(accel.y - expectedAccel[1], 2) +
                               pow(accel.z - expectedAccel[2], 2));
        
        // If innovation is small, update attitude covariance
        if (innovAccel < 0.5f) {
            stateCovariance[6][6] *= 0.99f;  // Increase confidence in roll
            stateCovariance[7][7] *= 0.99f;  // Increase confidence in pitch
        }
    }
}

/**
 * Correct state with Kalman gain
 */
void HybridNavigation::correctState() {
    // State correction is performed in update steps
    // This function can be used for additional corrections or constraints
    
    // Ensure altitude is reasonable
    if (positionState.altitude < -500.0f || positionState.altitude > 10000.0f) {
        positionState.altitude = 0.0f;
    }
}

/**
 * Update navigation system with GPS data
 */
void HybridNavigation::updateGPS(const GPSData& gpsData) {
    // Validate GPS data
    if (gpsData.latitude >= -90.0f && gpsData.latitude <= 90.0f &&
        gpsData.longitude >= -180.0f && gpsData.longitude <= 180.0f &&
        gpsData.horizontalAccuracy > 0.0f) {
        
        // Check if this is initial position fix
        if (!navigationInitialized) {
            positionState.latitude = gpsData.latitude;
            positionState.longitude = gpsData.longitude;
            positionState.altitude = gpsData.altitude;
            navigationInitialized = true;
        }
        
        gpsDataValid = true;
    }
}

/**
 * Normalize DCM matrix to maintain orthogonality
 * Uses Gram-Schmidt orthogonalization
 */
void HybridNavigation::normalizeDCMMatrix() {
    // Extract rows
    Vector3f row1 = {dcmMatrix[0][0], dcmMatrix[0][1], dcmMatrix[0][2]};
    Vector3f row2 = {dcmMatrix[1][0], dcmMatrix[1][1], dcmMatrix[1][2]};
    Vector3f row3 = {dcmMatrix[2][0], dcmMatrix[2][1], dcmMatrix[2][2]};
    
    // Normalize first row
    float norm1 = sqrt(row1.x * row1.x + row1.y * row1.y + row1.z * row1.z);
    if (norm1 > 0.0f) {
        row1.x /= norm1;
        row1.y /= norm1;
        row1.z /= norm1;
    }
    
    // Orthogonalize second row
    float dot12 = row2.x * row1.x + row2.y * row1.y + row2.z * row1.z;
    row2.x -= dot12 * row1.x;
    row2.y -= dot12 * row1.y;
    row2.z -= dot12 * row1.z;
    
    float norm2 = sqrt(row2.x * row2.x + row2.y * row2.y + row2.z * row2.z);
    if (norm2 > 0.0f) {
        row2.x /= norm2;
        row2.y /= norm2;
        row2.z /= norm2;
    }
    
    // Third row is cross product of first two
    row3.x = row1.y * row2.z - row1.z * row2.y;
    row3.y = row1.z * row2.x - row1.x * row2.z;
    row3.z = row1.x * row2.y - row1.y * row2.x;
    
    // Write normalized rows back
    dcmMatrix[0][0] = row1.x;
    dcmMatrix[0][1] = row1.y;
    dcmMatrix[0][2] = row1.z;
    dcmMatrix[1][0] = row2.x;
    dcmMatrix[1][1] = row2.y;
    dcmMatrix[1][2] = row2.z;
    dcmMatrix[2][0] = row3.x;
    dcmMatrix[2][1] = row3.y;
    dcmMatrix[2][2] = row3.z;
}

/**
 * Matrix multiplication: C = A * B (3x3 matrices)
 */
void HybridNavigation::matrixMultiply3x3(float A[3][3], float B[3][3], float C[3][3]) {
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            C[i][j] = 0.0f;
            for (int k = 0; k < 3; k++) {
                C[i][j] += A[i][k] * B[k][j];
            }
        }
    }
}

/**
 * Convert degrees to radians
 */
float HybridNavigation::degreesToRadians(float degrees) {
    return degrees * M_PI / 180.0f;
}

/**
 * Convert radians to degrees
 */
float HybridNavigation::radiansToDegrees(float radians) {
    return radians * 180.0f / M_PI;
}

/**
 * Constrain angle to [-pi, pi] range
 */
void HybridNavigation::constrainAngle(float& angle) {
    while (angle > M_PI) angle -= 2.0f * M_PI;
    while (angle < -M_PI) angle += 2.0f * M_PI;
}

/**
 * Get current navigation state
 */
NavigationState HybridNavigation::getNavigationState() const {
    NavigationState state;
    state.position = positionState;
    state.velocity = velocityState;
    state.attitude = attitudeState;
    state.isValid = navigationInitialized && (gpsDataValid || imuDataValid);
    return state;
}

/**
 * Update accelerometer calibration parameters
 */
void HybridNavigation::calibrateAccelerometer(const Vector3f bias, const Vector3f scale) {
    imuCalibration.accelBiasX = bias.x;
    imuCalibration.accelBiasY = bias.y;
    imuCalibration.accelBiasZ = bias.z;
    imuCalibration.accelScaleX = scale.x;
    imuCalibration.accelScaleY = scale.y;
    imuCalibration.accelScaleZ = scale.z;
}

/**
 * Update gyroscope calibration parameters
 */
void HybridNavigation::calibrateGyroscope(const Vector3f bias, const Vector3f scale) {
    imuCalibration.gyroBiasX = bias.x;
    imuCalibration.gyroBiasY = bias.y;
    imuCalibration.gyroBiasZ = bias.z;
    imuCalibration.gyroScaleX = scale.x;
    imuCalibration.gyroScaleY = scale.y;
    imuCalibration.gyroScaleZ = scale.z;
}

/**
 * Update magnetometer calibration parameters
 */
void HybridNavigation::calibrateMagnetometer(const Vector3f bias, const Vector3f scale) {
    magCalibration.biasX = bias.x;
    magCalibration.biasY = bias.y;
    magCalibration.biasZ = bias.z;
    magCalibration.scaleX = scale.x;
    magCalibration.scaleY = scale.y;
    magCalibration.scaleZ = scale.z;
}

/**
 * Get singleton instance
 */
HybridNavigation* HybridNavigation::getInstance() {
    if (instance == nullptr) {
        instance = new HybridNavigation();
    }
    return instance;
}

/**
 * Reset navigation system
 */
void HybridNavigation::reset() {
    initializeStateVectors();
    navigationInitialized = false;
    gpsDataValid = false;
    imuDataValid = false;
}
