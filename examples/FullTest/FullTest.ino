//////////////////////////////////////////////////////////////////
// Test RJ Modules (LED,SRF,IR)

// Connection:
// Port 1: LED
// Port 2: IR
// Port 3: SRF
//////////////////////////////////////////////////////////////////

#include <RobotixMega.h>

RobotixMega mega;

#define PORT1 mega.port_1
#define PORT2 mega.port_2
#define PORT3 mega.port_3

unsigned long startMillis;
unsigned long currentMillis;
const unsigned long period = 1000;

char temp[20];



void setup() {
  mega.begin();
  mega.showLogo(LOGO_ROBOTIX);
  delay(1000);
  mega.clearLCD();
  mega.drawString(25, 10, "Playing:");
  mega.drawString(35, 30, "NOKIA");

  mega.playSong(NOKIA);
  mega.rjSrfInit(PORT1);
  mega.rjIrInit(PORT2);
  mega.rjLedInit(PORT3);

  Serial.begin(115200);
  Serial4.begin(115200);
  pinMode(61, OUTPUT);


  mega.setMotorSpeed(mega.motor_1, 100);
  mega.setMotorSpeed(mega.motor_2, 100);
  mega.setMotorSpeed(mega.motor_3, 100);
  mega.setMotorSpeed(mega.motor_4, 100);




  for (int i = 0; i < 10; i++) {
    digitalWrite(61, HIGH);
    delay(1);
    Serial4.write(0xFF);
    Serial4.write(0xFF);
    Serial4.write(0x02);
    Serial4.write(0x04);
    Serial4.write(0x03);
    Serial4.write(0x19);
    Serial4.write((byte)0x00);
    Serial4.write(0xDD);
    delay(1);
    digitalWrite(61, LOW);
    delay(100);


    digitalWrite(61, HIGH);
    delay(1);
    Serial4.write(0xFF);
    Serial4.write(0xFF);
    Serial4.write(0x02);
    Serial4.write(0x04);
    Serial4.write(0x03);
    Serial4.write(0x19);
    Serial4.write((byte)0x01);
    Serial4.write(0xDC);
    delay(1);
    digitalWrite(61, LOW);
    delay(100);
  }



  mega.setServoPositions(mega.servo_1, 180);
  mega.setServoPositions(mega.servo_2, 180);
  mega.setServoPositions(mega.servo_3, 180);
  mega.setServoPositions(mega.servo_4, 180);
  mega.setServoPositions(mega.servo_5, 180);
  mega.setServoPositions(mega.servo_6, 180);
  mega.setServoPositions(mega.servo_7, 180);
  mega.setServoPositions(mega.servo_8, 180);
}

// the loop function runs over and over again forever
void loop() {

  mega.rjLedWrite(PORT3, mega.rjIrRead(PORT2));

  currentMillis = millis();
  if (currentMillis - startMillis >= period) {
    float distance = mega.rjSrfRead(PORT1);
    sprintf(temp, "distance: %3.1f ", distance);
    mega.clearLCD();
    mega.drawString(0, 10, temp);
    Serial.print(temp);
    startMillis = currentMillis;
  }
}
