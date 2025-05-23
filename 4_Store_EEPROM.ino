#include "Adafruit_Sensor_Calibration.h"

// select either EEPROM or SPI FLASH storage:
#ifdef ADAFRUIT_SENSOR_CALIBRATION_USE_EEPROM
  Adafruit_Sensor_Calibration_EEPROM cal;
#else
  Adafruit_Sensor_Calibration_SDFat cal;
#endif

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);

  delay(3000);
  Serial.println("Calibration filesys test");
  if (!cal.begin()) {
    Serial.println("Failed to initialize calibration helper");
    while (1) yield();
  }
  Serial.print("Has EEPROM: "); Serial.println(cal.hasEEPROM());
  Serial.print("Has FLASH: "); Serial.println(cal.hasFLASH());

  if (! cal.loadCalibration()) {
    Serial.println("No calibration loaded/found... will start with defaults");
  } else {
    Serial.println("Loaded existing calibration");
  }

  // in g
  cal.accel_zerog[0] = 0.02;
  cal.accel_zerog[1] = 0.4;
  cal.accel_zerog[2] = -1.53;

  // in uTesla
  cal.mag_hardiron[0] = -0.21;
  cal.mag_hardiron[1] = 0.92;
  cal.mag_hardiron[2] = -4.68;

  // in uTesla
  cal.mag_softiron[0] = 1.024;
  cal.mag_softiron[1] = 0.004;
  cal.mag_softiron[2] = 0.007;  
  cal.mag_softiron[3] = 0.004;
  cal.mag_softiron[4] = 1.025;
  cal.mag_softiron[5] = -0.036;  
  cal.mag_softiron[6] = 0.007;
  cal.mag_softiron[7] = -0.036;
  cal.mag_softiron[8] = 0.954;
  // Earth total magnetic field strength in uTesla (dependent on location and time of the year),
  // visit: https://www.ngdc.noaa.gov/geomag/calculators/magcalc.shtml#igrfwmm)
  cal.mag_field = 52.00; // approximate value for locations along the equator

  // in Radians/s
  cal.gyro_zerorate[0] = 0.13;
  cal.gyro_zerorate[1] = -0.93;
  cal.gyro_zerorate[2] = 0.65;

  if (! cal.saveCalibration()) {
    Serial.println("**WARNING** Couldn't save calibration");
  } else {
    Serial.println("Wrote calibration");    
  }

  cal.printSavedCalibration();
}

void loop() {

}
