#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>

/* Assign a unique ID to this sensor at the same time */
Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(54321);

int count[6] = {0, 0, 0, 0, 0, 0};
float avg[6] = {0.0,0.0,0.0,0.0,0.0,0.0};
float prev_val[3] = {0.0, 0.0, 0.0};
float bias_x, bias_y, bias_z;
float scale_x, scale_y, scale_z;

void displaySensorDetails(void)
{
  sensor_t sensor;
  accel.getSensor(&sensor);
  Serial.println("------------------------------------");
  Serial.print  ("Sensor:       "); Serial.println(sensor.name);
  Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
  Serial.print  ("Unique ID:    "); Serial.println(sensor.sensor_id);
  Serial.print  ("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" m/s^2");
  Serial.print  ("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" m/s^2");
  Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" m/s^2");
  Serial.println("------------------------------------");
  Serial.println("");
  delay(500);
}

void setup(void)
{

  Serial.begin(19200);
  delay(2000);
  Serial.println("Accelerometer Test"); Serial.println("");

  /* Initialise the sensor */
  if(!accel.begin())
  {
    /* There was a problem detecting the ADXL345 ... check your connections */
    Serial.println("Ooops, no LSM303 detected ... Check your wiring!");
    while(1);
  }

  /* Display some basic information on this sensor */
  displaySensorDetails();
}

void loop(void)
{
  /* Get a new sensor event */
  sensors_event_t event;
  accel.getEvent(&event);

  float abs_x = abs(event.acceleration.x);
  float abs_y = abs(event.acceleration.y);
  float abs_z = abs(event.acceleration.z);

  if((abs(prev_val[0] - abs_x) < 0.05) && (abs(prev_val[1] - abs_y) < 0.05) && (abs(prev_val[2] - abs_z) < 0.05))
  {
      // Serial.println("Not moving");
      int axis = abs_x > abs_y ? (abs_x > abs_z ? 0 : 2) : (abs_y > abs_z ? 1 : 2);

      switch (axis) {
        case 0:
            {
              int id = event.acceleration.x > 0 ? 0 : 1;
              avg[id] = ((avg[id] * count[id]) + event.acceleration.x)/(count[id] + 1);
              count[id]++;
              break;
            }
        case 1:
            {
              int id = event.acceleration.y > 0 ? 2 : 3;
              avg[id] = ((avg[id] * count[id]) + event.acceleration.y)/(count[id] + 1);
              count[id]++;
              break;
            }
        case 2:
            {
              int id = event.acceleration.z > 0 ? 4 : 5;
              avg[id] = ((avg[id] * count[id]) + event.acceleration.z)/(count[id] + 1);
              count[id]++;
              break;
              }
      }
  }
  prev_val[0] = abs_x;
  prev_val[1] = abs_y;
  prev_val[2] = abs_z;

  Serial.print(avg[0]);
  Serial.print(" ");
  Serial.print(avg[1]);
  Serial.print(" ");
  Serial.print(avg[2]);
  Serial.print(" ");
  Serial.print(avg[3]);
  Serial.print(" ");
  Serial.print(avg[4]);
  Serial.print(" ");
  Serial.print(avg[5]);
  Serial.print(" ");

  float bias_x = (avg[0] + avg[1]) / 2.0;
  float bias_y = (avg[2] + avg[3]) / 2.0;
  float bias_z = (avg[4] + avg[5]) / 2.0;

  float scale_x = (avg[0] - avg[1]) / 2.0;
  float scale_y = (avg[2] - avg[3]) / 2.0;
  float scale_z = (avg[4] - avg[5]) / 2.0;

  Serial.print(bias_x);
  Serial.print(" ");
  Serial.print(bias_y);
  Serial.print(" ");
  Serial.print(bias_z);
  Serial.print(" ");
  Serial.print(scale_x);
  Serial.print(" ");
  Serial.print(scale_y);
  Serial.print(" ");
  Serial.println(scale_z);

  /* Delay before the next sample */
  delay(100);
}
