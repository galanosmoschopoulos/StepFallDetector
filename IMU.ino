#include <Wire.h>
#include <Kalman.h> // Source: https://github.com/TKJElectronics/KalmanFilter

#ifndef RESTRICT_PITCH
#define RESTRICT_PITCH // Comment out to restrict roll to ±90deg instead - please read: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf
#endif

Kalman kalmanX; // Create the Kalman instances
Kalman kalmanY;

/* IMU Data */
double accX, accY, accZ;
double gyroX, gyroY, gyroZ;
double kalAngleX, kalAngleY; // Calculated angle using a Kalman filter
uint32_t timer;
uint8_t i2cData[14]; // Buffer for I2C data
uint32_t lastIMUReadTime;

void imuInit() {
  Wire.begin();
  Wire.setClock(400000UL); // Set I2C frequency to 400kHz

  i2cData[0] = 7; // Set the sample rate to 1000Hz - 8kHz/(7+1) = 1000Hz
  i2cData[1] = 0x00; // Disable FSYNC and set 260 Hz Acc filtering, 256 Hz Gyro filtering, 8 KHz sampling
  i2cData[2] = 0x00; // Set Gyro Full Scale Range to ±250deg/s
  i2cData[3] = 0x00; // Set Accelerometer Full Scale Range to ±2g
  while (i2cWrite(0x19, i2cData, 4, false)); // Write to all four registers at once
  while (i2cWrite(0x6B, 0x01, true)); // PLL with X axis gyroscope reference and disable sleep mode

  while (i2cRead(0x75, i2cData, 1));
  if (i2cData[0] != 0x68) { // Read "WHO_AM_I" register
    Serial.print(F("Error reading sensor"));
    while (1);
  }

  delay(100); // Wait for sensor to stabilize

  /* Set kalman and gyro starting angle */
  while (i2cRead(0x3B, i2cData, 6));
  accX = (int16_t)((i2cData[0] << 8) | i2cData[1]);
  accY = (int16_t)((i2cData[2] << 8) | i2cData[3]);
  accZ = (int16_t)((i2cData[4] << 8) | i2cData[5]);
  lastIMUReadTime = millis();
  // Source: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf eq. 25 and eq. 26
  // atan2 outputs the value of -π to π (radians) - see http://en.wikipedia.org/wiki/Atan2
  // It is then converted from radians to degrees
#ifdef RESTRICT_PITCH // Eq. 25 and 26
  double roll  = atan2(accY, accZ) * RAD_TO_DEG;
  double pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
#else // Eq. 28 and 29
  double roll  = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
  double pitch = atan2(-accX, accZ) * RAD_TO_DEG;
#endif

  kalmanX.setAngle(roll); // Set starting angle
  kalmanY.setAngle(pitch);

  timer = micros();
}

void IMUUpdate() {
  /* Update all the values */
  while (i2cRead(0x3B, i2cData, 14));
  accX = (int16_t)((i2cData[0] << 8) | i2cData[1]);
  accY = (int16_t)((i2cData[2] << 8) | i2cData[3]);
  accZ = (int16_t)((i2cData[4] << 8) | i2cData[5]);
  gyroX = (int16_t)((i2cData[8] << 8) | i2cData[9]);
  gyroY = (int16_t)((i2cData[10] << 8) | i2cData[11]);
  gyroZ = (int16_t)((i2cData[12] << 8) | i2cData[13]);;

  double dt = (double)(micros() - timer) / 1000000; // Calculate delta time
  timer = micros();

  // Source: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf eq. 25 and eq. 26
  // atan2 outputs the value of -π to π (radians) - see http://en.wikipedia.org/wiki/Atan2
  // It is then converted from radians to degrees
#ifdef RESTRICT_PITCH // Eq. 25 and 26
  double roll  = atan2(accY, accZ) * RAD_TO_DEG;
  double pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
#else // Eq. 28 and 29
  double roll  = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
  double pitch = atan2(-accX, accZ) * RAD_TO_DEG;
#endif

  double gyroXrate = gyroX / 131.0; // Convert to deg/s
  double gyroYrate = gyroY / 131.0; // Convert to deg/s

#ifdef RESTRICT_PITCH
  // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
  if ((roll < -90 && kalAngleX > 90) || (roll > 90 && kalAngleX < -90)) {
    kalmanX.setAngle(roll);
    kalAngleX = roll;
  } else
    kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt); // Calculate the angle using a Kalman filter

  if (abs(kalAngleX) > 90)
    gyroYrate = -gyroYrate; // Invert rate, so it fits the restriced accelerometer reading
  kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt);
#else
  // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
  if ((pitch < -90 && kalAngleY > 90) || (pitch > 90 && kalAngleY < -90)) {
    kalmanY.setAngle(pitch);
    kalAngleY = pitch;
  } else
    kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt); // Calculate the angle using a Kalman filter

  if (abs(kalAngleY) > 90)
    gyroXrate = -gyroXrate; // Invert rate, so it fits the restriced accelerometer reading
  kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt); // Calculate the angle using a Kalman filter
#endif
}

bool triggeredLying = false;
bool isLying;
uint32_t firstLTime;
#define MAX_LYING_TIME 4000
void checkLying() {
  double angle = abs(kalAngleY);
  if(angle <= 40.0) {
    if(triggeredLying == false) {
      triggeredLying = true;
      firstLTime = millis();
    }
  }
  else {
    triggeredLying = false;
  }
  if(triggeredLying && (millis() - firstLTime > MAX_LYING_TIME)) {
    isLying = true;
  }
  else {
    isLying = false;
  }
}


uint32_t lastAccelTime;
uint32_t lastTriggeredAccelTime;
bool triggeredAccel = false;
bool hasAccelerated = false;

#define FALL_DURATION 50
#define ACCEL_THRESHOLD 0.50
void checkAccel() {
  double ax = accX/16384, ay = accY/16384, az = accZ/16384;
  double a = sqrt(pow(ax, 2) + pow(ay, 2) + pow(az, 2));

  if(a < ACCEL_THRESHOLD) {
    if(!triggeredAccel) {
      triggeredAccel = true;
      lastTriggeredAccelTime = millis();
    }
  }
  else {
    triggeredAccel = false;
  }

  if (triggeredAccel && (millis() - lastTriggeredAccelTime > FALL_DURATION)) {
    hasAccelerated = true;
    lastAccelTime = millis();
    triggeredAccel = false;
  }

}


bool hasFallen;
uint32_t fallenTime;
#define TIME_FROM_TRIP_TO_LYING 600
void checkFallen() {
  if(hasAccelerated && millis()-lastAccelTime < MAX_LYING_TIME + TIME_FROM_TRIP_TO_LYING && isLying) {
    hasAccelerated = false;
    hasFallen = true;
    fallenTime = millis();
  }
}

void imuLoop() {
  if (millis() - lastIMUReadTime > 2) {
    IMUUpdate();
    lastIMUReadTime = millis();
  }
  checkLying();
  checkAccel();
  checkFallen();
  
}
