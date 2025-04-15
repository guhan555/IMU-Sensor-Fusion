#include <Wire.h>
#include <L3G4200D.h>

L3G4200D gyroscope;

// Timers
unsigned long timer = 0;
float timeStep = 0.01;

float avg_x = 0.0, avg_y = 0.0, avg_z = 0.0;
int count_x = 0, count_y = 0, count_z = 0;

void setup() 
{
  Serial.begin(115200);

  delay(2000);

  // Initialize L3G4200D
  // Set scale 250 dps and 400HZ Output data rate (cut-off 50)
  while (!gyroscope.begin(L3G4200D_SCALE_250DPS, L3G4200D_DATARATE_400HZ_50))
  {
    // Waiting for initialization
    delay(500);
  }

  delay(3000);
}

void loop()
{
  timer = millis();

  // Read normalized values
  Vector norm = gyroscope.readNormalize();

  avg_x = ((avg_x * count_x) + norm.XAxis) / (count_x + 1);
  avg_y = ((avg_y * count_y) + norm.YAxis) / (count_y + 1);
  avg_z = ((avg_z * count_z) + norm.ZAxis) / (count_z + 1);

  count_x++;
  count_y++;
  count_z++;

  Serial.print(avg_x);
  Serial.print(" ");
  Serial.print(avg_y);
  Serial.print(" ");
  Serial.print(avg_z);
  Serial.print(" ");
  Serial.print(norm.XAxis);
  Serial.print(" ");
  Serial.print(norm.YAxis);
  Serial.print(" ");
  Serial.println(norm.ZAxis);


  
  // Wait to full timeStep period
  delay((timeStep*1000) - (millis() - timer));
}
