#include <Arduino.h>
#include"GraspeKinematics.h"

GraspeKinematics Kinematics;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
}

void loop() {
  // put your main code here, to run repeatedly:

  Serial.println("Começou");
  unsigned long initial_time = millis();

  std::vector<float> q = {0.0, 0.0, 0.0, 0.0};

  SE3 dado = Kinematics.directKinematics(q);

  for (int i = 0; i < 3; i++)
  {
    Serial.print(dado.pos[i]);
  };

  initial_time = millis() - initial_time;
  Serial.print(initial_time);
  Serial.println("Acabou");
  delay(1000);
}
