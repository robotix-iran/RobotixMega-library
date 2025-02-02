#include <RobotixMega.h>

RobotixMega mega;

void setup() {
  mega.begin();
  Serial.begin(115200);
}

void loop() {
  float accx, accy, accz;
  float gyrx, gyry, gyrz;
  float magx, magy, magz;

  mega.getAccelerations(accx, accy, accz);
  mega.getGyros(gyrx, gyry, gyrz);
  mega.getMagno(magx, magy, magz);

  Serial.print("\t\tAccel X: ");
  Serial.print(accx);
  Serial.print(" \tY: ");
  Serial.print(accy);
  Serial.print(" \tZ: ");
  Serial.print(accz);
  Serial.println(" m/s^2 ");

  Serial.print("\t\tMag X: ");
  Serial.print(magx);
  Serial.print(" \tY: ");
  Serial.print(magy);
  Serial.print(" \tZ: ");
  Serial.print(magz);
  Serial.println(" uT");

  Serial.print("\t\tGyro X: ");
  Serial.print(gyrx);
  Serial.print(" \tY: ");
  Serial.print(gyry);
  Serial.print(" \tZ: ");
  Serial.print(gyrz);
  Serial.println(" radians/s ");
  Serial.println();

  delay(100);
}
