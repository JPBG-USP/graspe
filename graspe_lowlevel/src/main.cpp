#include <Arduino.h>
#include"GraspeKinematics.h"

GraspeKinematics Kinematics;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
}

void loop() {
  // put your main code here, to run repeatedly:

  Serial.println("Comecou");
  unsigned long start_time = millis();

  std::vector<float> q = {1.57, 5.0, 12.0, 0.0};

  std::vector<float> position_cylindrical = {0.0, 10.0, 15.0, 0.0};

  SE3 position;
  position.transform = {
    {0.9553, 0.2955, 0, 25.9776},
    {0, 0, -1, 0},
    {-0.2955, 0.9553, 0, 8.6927},
    {0, 0, 0, 1}
  };

  std::vector<float> dado = Kinematics.inverseKinematics(position);
  std::vector<float> data = Kinematics.inverseKinematicsCylindrical(position_cylindrical);

  for (int i = 0; i < 4; i++)
  {
    Serial.print(data[i]);
    Serial.print(" ");
  }
  Serial.println("");

  unsigned long elapsed_time = millis() - start_time;
  Serial.print(elapsed_time);
  Serial.println("");
  Serial.println("Acabou");
  delay(1000);
}
