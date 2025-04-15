#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>
#include <Adafruit_L3GD20_U.h>
#include <Adafruit_9DOF.h>
#include <L3G4200D.h>
#include <Adafruit_BNO055.h>
#include "Adafruit_Sensor_Calibration.h"
#include <EEPROM.h>

#define BNO055_SAMPLERATE_DELAY_MS (20)

/* Assign a unique ID to the sensors */
Adafruit_9DOF dof = Adafruit_9DOF();
Adafruit_LSM303_Accel_Unified accelerometer = Adafruit_LSM303_Accel_Unified(30301);
Adafruit_LSM303_Mag_Unified magnetometer = Adafruit_LSM303_Mag_Unified(30302);
L3G4200D gyroscope;
Adafruit_Sensor_Calibration_EEPROM cal;
Adafruit_BNO055 bno = Adafruit_BNO055(-1, 0x29, &Wire);

unsigned long previousMillis = 0;
const long interval = 10;

float alpha = 0.95;

sensors_vec_t gyro_orientation;
sensors_vec_t orientation;

adafruit_bno055_offsets_t offset;
bool loadManualCalib = false;

float LSMAccel_bias[3] = { 0.0, 0.0, 0.0 };
float LSMAccel_scale[3] = { 9.93, 9.90, 9.99 };
float LSMMag_hardOffset[3] = { 0.0, 0.0, 0.0 };
float LSMMag_softOffset[9] = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
float L3GGyro_bias[3] = { 0.0, 0.0, 0.0 };

void initBNOOffset(adafruit_bno055_offsets_t* offsetVal) {
  memset(offsetVal, 0, sizeof(adafruit_bno055_offsets_t));

  offsetVal->accel_offset_x = 36;
  offsetVal->accel_offset_y = -1;
  offsetVal->accel_offset_z = -1;

  offsetVal->mag_offset_x = 45;
  offsetVal->mag_offset_y = 315;
  offsetVal->mag_offset_z = 233;

  offsetVal->gyro_offset_x = 1;
  offsetVal->gyro_offset_y = 1;
  offsetVal->gyro_offset_z = 1;

  offsetVal->accel_radius = 1000;

  offsetVal->mag_radius = 775;
}

void displaySensorDetails(void) {
  sensor_t sensor;
  bno.getSensor(&sensor);
  Serial.println("------------------------------------");
  Serial.print("Sensor:       ");
  Serial.println(sensor.name);
  Serial.print("Driver Ver:   ");
  Serial.println(sensor.version);
  Serial.print("Unique ID:    ");
  Serial.println(sensor.sensor_id);
  Serial.print("Max Value:    ");
  Serial.print(sensor.max_value);
  Serial.println(" xxx");
  Serial.print("Min Value:    ");
  Serial.print(sensor.min_value);
  Serial.println(" xxx");
  Serial.print("Resolution:   ");
  Serial.print(sensor.resolution);
  Serial.println(" xxx");
  Serial.println("------------------------------------");
  Serial.println("");
  delay(500);
}

void displaySensorStatus(void) {
  /* Get the system status values (mostly for debugging purposes) */
  uint8_t system_status, self_test_results, system_error;
  system_status = self_test_results = system_error = 0;
  bno.getSystemStatus(&system_status, &self_test_results, &system_error);

  /* Display the results in the Serial Monitor */
  Serial.println("");
  Serial.print("System Status: 0x");
  Serial.println(system_status, HEX);
  Serial.print("Self Test:     0x");
  Serial.println(self_test_results, HEX);
  Serial.print("System Error:  0x");
  Serial.println(system_error, HEX);
  Serial.println("");
  delay(500);
}

void displaySensorOffsets(const adafruit_bno055_offsets_t& calibData) {
  Serial.print("Accelerometer: ");
  Serial.print(calibData.accel_offset_x);
  Serial.print(" ");
  Serial.print(calibData.accel_offset_y);
  Serial.print(" ");
  Serial.print(calibData.accel_offset_z);
  Serial.print(" ");

  Serial.print("\nGyro: ");
  Serial.print(calibData.gyro_offset_x);
  Serial.print(" ");
  Serial.print(calibData.gyro_offset_y);
  Serial.print(" ");
  Serial.print(calibData.gyro_offset_z);
  Serial.print(" ");

  Serial.print("\nMag: ");
  Serial.print(calibData.mag_offset_x);
  Serial.print(" ");
  Serial.print(calibData.mag_offset_y);
  Serial.print(" ");
  Serial.print(calibData.mag_offset_z);
  Serial.print(" ");

  Serial.print("\nAccel Radius: ");
  Serial.print(calibData.accel_radius);

  Serial.print("\nMag Radius: ");
  Serial.print(calibData.mag_radius);
}

void displayCalStatus(void) {
  /* Get the four calibration values (0..3) */
  /* Any sensor data reporting 0 should be ignored, */
  /* 3 means 'fully calibrated" */
  uint8_t system, gyro, accel, mag;
  system = gyro = accel = mag = 0;
  bno.getCalibration(&system, &gyro, &accel, &mag);

  /* The data should be ignored until the system calibration is > 0 */
  /* Display the individual values */
  Serial.print(system, DEC);
  Serial.print(gyro, DEC);
  Serial.print(accel, DEC);
  Serial.print(mag, DEC);
}

void setupBNOSensor() {
  bno.setAxisRemap(Adafruit_BNO055::REMAP_CONFIG_P6);
  bno.setAxisSign(Adafruit_BNO055::REMAP_SIGN_P6);

  long bnoID;
  bool foundCalib = false;

  adafruit_bno055_offsets_t calibrationData;
  sensor_t sensor;

  delay(1000);

  /* Display some basic information on this sensor */
  displaySensorDetails();

  /* Optional: Display current status */
  displaySensorStatus();

  if (loadManualCalib) {
    initBNOOffset(&offset);
    bno.setSensorOffsets(offset);
    delay(1000);
  }

  /* Crystal must be configured AFTER loading calibration data into BNO055. */
  bno.setExtCrystalUse(true);

  sensors_event_t event;
  bno.getEvent(&event);
  /* always recal the mag as It goes out of calibration very often */
  if (loadManualCalib) {
    Serial.println("Move sensor slightly to calibrate magnetometers");
    while (!bno.isFullyCalibrated()) {
      bno.getEvent(&event);
      delay(BNO055_SAMPLERATE_DELAY_MS);
    }
  } else {
    Serial.println("Please Calibrate Sensor: ");
    while (!bno.isFullyCalibrated()) {
      bno.getEvent(&event);

      Serial.print("X: ");
      Serial.print(event.orientation.x, 4);
      Serial.print("\tY: ");
      Serial.print(event.orientation.y, 4);
      Serial.print("\tZ: ");
      Serial.print(event.orientation.z, 4);

      Serial.print("\t");
      /* Optional: Display calibration status */
      displayCalStatus();

      /* New line for the next sample */
      Serial.println("");

      /* Wait the specified delay before requesting new data */
      delay(BNO055_SAMPLERATE_DELAY_MS);
    }
  }

  Serial.println("\nFully calibrated!");
  Serial.println("--------------------------------");
  Serial.println("Calibration Results: ");
  adafruit_bno055_offsets_t newCalib;
  bno.getSensorOffsets(newCalib);
  displaySensorOffsets(newCalib);

  Serial.println("\n--------------------------------\n");
  delay(500);
  Serial.println("OK");
  delay(500);
}

void setupLSMAndL3GSensor() {
  if (!cal.begin()) {
    Serial.println("Failed to initialize calibration helper");
    while (1)
      ;
  }

  if (!cal.loadCalibration()) {
    Serial.println("**WARNING** No calibration loaded/found");
  }

  memcpy(LSMAccel_bias, cal.accel_zerog, sizeof(cal.accel_zerog));
  memcpy(LSMMag_hardOffset, cal.mag_hardiron, sizeof(cal.mag_hardiron));
  memcpy(LSMMag_softOffset, cal.mag_softiron, sizeof(cal.mag_softiron));
  memcpy(L3GGyro_bias, cal.gyro_zerorate, sizeof(cal.gyro_zerorate));
}

void initSensors() {
  if (!accelerometer.begin()) {
    /* There was a problem detecting the LSM303 ... check your connections */
    Serial.println(F("Ooops, no LSM303 detected ... Check your wiring!"));
    while (1)
      ;
  }
  if (!magnetometer.begin()) {
    /* There was a problem detecting the LSM303 ... check your connections */
    Serial.println("Ooops, no LSM303 detected ... Check your wiring!");
    while (1)
      ;
  }
  // Set scale 250 dps and 400HZ Output data rate (cut-off 50)
  while (!gyroscope.begin(L3G4200D_SCALE_250DPS, L3G4200D_DATARATE_400HZ_50)) {
    /* There was a problem detecting the L3G4200D ... check your connections */
    Serial.println("Ooops, no L3G4200D detected ... Check your wiring!");
    while (1)
      ;
  }

  if (!bno.begin()) {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1)
      ;
  }

  setupBNOSensor();
  setupLSMAndL3GSensor();

  gyro_orientation.roll = 0.0;
  gyro_orientation.pitch = 0.0;
  gyro_orientation.heading = 0.0;


  orientation.roll = 0.0;
  orientation.pitch = 0.0;
  orientation.heading = 0.0;
}

void setup(void) {
  Serial.begin(921600);

  delay(5000);

  /* Initialise the sensors */
  initSensors();
}

void loop(void) {
  sensors_event_t accel_event;
  sensors_event_t mag_event;
  sensors_event_t bno_event;
  sensors_vec_t temp_orientation;

  unsigned long currentMillis = millis();

  if (currentMillis - previousMillis >= interval) {
    float timeStep = (currentMillis - previousMillis) / 1000;
    // bno.getEvent(&bno_event);
    imu::Quaternion q = bno.getQuat();
    q.normalize();
    imu::Vector<3> euler = q.toEuler();
    euler.toDegrees();
    if(-1*euler.x() < 0)
      Serial.print(-1*euler.x() + 360, 2);
    else
      Serial.print(-1*euler.x(), 2);
    Serial.print(" ");
    // Serial.print(euler.x() + 360, 2);
    // Serial.print(" ");
    Serial.print(-1*euler.y(), 2);
    Serial.print(" ");
    Serial.print(euler.z(), 2);
    Serial.print(" ");

    // Serial.print(bno_event.orientation.x, 2);
    // Serial.print(" ");
    // Serial.print(bno_event.orientation.y, 2);
    // Serial.print(" ");
    // Serial.print(bno_event.orientation.z, 2);
    // Serial.print(" ");

    /* Calculate pitch and roll from the raw accelerometer data */
    accelerometer.getEvent(&accel_event);

    accel_event.acceleration.x = (accel_event.acceleration.x - LSMAccel_bias[0]) * (9.81 / LSMAccel_scale[0]);
    accel_event.acceleration.y = (accel_event.acceleration.y - LSMAccel_bias[1]) * (9.81 / LSMAccel_scale[1]);
    accel_event.acceleration.z = (accel_event.acceleration.z - LSMAccel_bias[2]) * (9.81 / LSMAccel_scale[2]);

    dof.accelGetOrientation(&accel_event, &temp_orientation);

    Vector norm = gyroscope.readNormalize();

    norm.XAxis = norm.XAxis - L3GGyro_bias[0];
    norm.YAxis = norm.YAxis - L3GGyro_bias[1];
    norm.ZAxis = norm.ZAxis - L3GGyro_bias[2];

    orientation.roll = alpha * (orientation.roll + norm.XAxis * timeStep) + (1 - alpha) * temp_orientation.roll;
    orientation.pitch = alpha * (orientation.pitch + norm.YAxis * timeStep) + (1 - alpha) * temp_orientation.pitch;

    /* Calculate the heading using the magnetometer */
    magnetometer.getEvent(&mag_event);

    mag_event.magnetic.x = mag_event.magnetic.x - LSMMag_hardOffset[0];
    mag_event.magnetic.y = mag_event.magnetic.y - LSMMag_hardOffset[1];
    mag_event.magnetic.z = mag_event.magnetic.z - LSMMag_hardOffset[2];

    mag_event.magnetic.x = LSMMag_softOffset[0] * mag_event.magnetic.x + LSMMag_softOffset[1] * mag_event.magnetic.y + LSMMag_softOffset[2] * mag_event.magnetic.z;
    mag_event.magnetic.y = LSMMag_softOffset[3] * mag_event.magnetic.x + LSMMag_softOffset[4] * mag_event.magnetic.y + LSMMag_softOffset[6] * mag_event.magnetic.z;
    mag_event.magnetic.z = LSMMag_softOffset[5] * mag_event.magnetic.x + LSMMag_softOffset[7] * mag_event.magnetic.y + LSMMag_softOffset[8] * mag_event.magnetic.z;

    if (dof.magTiltCompensation(SENSOR_AXIS_Z, &mag_event, &accel_event) && dof.magGetOrientation(SENSOR_AXIS_Z, &mag_event, &temp_orientation)) {
      orientation.heading = alpha * (orientation.heading + norm.ZAxis * timeStep) + (1 - alpha) * temp_orientation.heading;
    }

    Serial.print(orientation.heading, 2);
    Serial.print(" ");
    Serial.print(orientation.pitch, 2);
    Serial.print(" ");
    Serial.println(orientation.roll, 2);

    previousMillis = currentMillis;
  }
}