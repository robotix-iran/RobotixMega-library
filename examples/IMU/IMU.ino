#include <RobotixMega.h>
#include <MadgwickAHRS.h>


RobotixMega mega;
Madgwick filter;

unsigned long microsPerReading, microsPrevious;

void setup() {
  mega.begin();
  Serial.begin(115200);
  Serial.println("start");

  filter.begin(25);

  microsPerReading = 1000000 / 25;
  microsPrevious = micros();
}

// the loop function runs over and over again forever
void loop() {
  float accx, accy, accz;
  float gyrx, gyry, gyrz;

  float roll, pitch, heading;
  unsigned long microsNow;

  mega.getIMU(accx, accy, accz, gyrx, gyry, gyrz);


  microsNow = micros();
  if (microsNow - microsPrevious >= microsPerReading) {
    // update the filter, which computes orientation
    filter.updateIMU(gyrx, gyry, gyrz, accx, accy, accz);

    // print the heading, pitch and roll
    roll = filter.getRoll();
    pitch = filter.getPitch();
    heading = filter.getYaw();
    Serial.print("Orientation: ");
    Serial.print(heading);
    Serial.print(" ");
    Serial.print(pitch);
    Serial.print(" ");
    Serial.println(roll);


    // increment previous time, so we keep proper pace
    microsPrevious = microsPrevious + microsPerReading;
  }

}