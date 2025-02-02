//////////////////////////////////////////////////////////////////
// Test RJ Modules (LED,SRF,IR)

// Connection:
// Port 1: SRF
// Port 2: IR
// Port 3: LED
//////////////////////////////////////////////////////////////////

#include <RobotixMega.h>
#include <Dynamixel2Arduino.h>

#define DXL_SERIAL Serial3
#define DEBUG_SERIAL Serial
#define DXL_DIR_PIN 53
const uint8_t DXL_ID = 5;
const float DXL_PROTOCOL_VERSION = 1.0;
Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);


RobotixMega mega;
#define PORT1 mega.port_1
#define PORT2 mega.port_2
#define PORT3 mega.port_3



unsigned long startMillis;
unsigned long currentMillis;
const unsigned long period = 500;

char temp[20];



void setup() {
  mega.begin();
  delay(1000);
  mega.clearLCD();
  mega.drawString(25, 10, "Playing:");
  mega.drawString(35, 30, "NOKIA");
  mega.playSong(NOKIA);
  mega.clearLCD();

  mega.rjIrInit(PORT2);
  mega.rjLedInit(PORT3);

  Serial.begin(115200);

  mega.servo_1.attach(PIN_SERVO_1);
  mega.servo_2.attach(PIN_SERVO_2);
  mega.servo_3.attach(PIN_SERVO_3);
  mega.servo_4.attach(PIN_SERVO_4);
  mega.servo_5.attach(PIN_SERVO_5);
  mega.servo_6.attach(PIN_SERVO_6);
  mega.servo_7.attach(PIN_SERVO_7);
  mega.servo_8.attach(PIN_SERVO_8);

  ////Motors //////////////////////////////////////////////////////////////
  mega.setMotorSpeed(mega.motor_1, 100);
  mega.setMotorSpeed(mega.motor_2, 100);
  mega.setMotorSpeed(mega.motor_3, 100);
  mega.setMotorSpeed(mega.motor_4, 100);
  delay(500);
  mega.setMotorSpeed(mega.motor_1, -200);
  mega.setMotorSpeed(mega.motor_2, -200);
  mega.setMotorSpeed(mega.motor_3, -200);
  mega.setMotorSpeed(mega.motor_4, -200);
  /////////////////////////////////////////////////////////////////////////

  ////Servos //////////////////////////////////////////////////////////////
  mega.setServoPositions(mega.servo_1, 180);
  mega.setServoPositions(mega.servo_2, 180);
  mega.setServoPositions(mega.servo_3, 180);
  mega.setServoPositions(mega.servo_4, 180);
  mega.setServoPositions(mega.servo_5, 180);
  mega.setServoPositions(mega.servo_6, 180);
  mega.setServoPositions(mega.servo_7, 180);
  mega.setServoPositions(mega.servo_8, 180);
  delay(500);
  mega.setServoPositions(mega.servo_1, 90);
  mega.setServoPositions(mega.servo_2, 90);
  mega.setServoPositions(mega.servo_3, 90);
  mega.setServoPositions(mega.servo_4, 90);
  mega.setServoPositions(mega.servo_5, 90);
  mega.setServoPositions(mega.servo_6, 90);
  mega.setServoPositions(mega.servo_7, 90);
  mega.setServoPositions(mega.servo_8, 90);
  delay(500);
  /////////////////////////////////////////////////////////////////////////

  ////Dynamixel ///////////////////////////////////////////////////////////
  dxl.begin(9600);
  dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);
  dxl.ping(DXL_ID);

  dxl.torqueOff(DXL_ID);
  dxl.setOperatingMode(DXL_ID, OP_VELOCITY);
  dxl.torqueOn(DXL_ID);
  if (dxl.setGoalVelocity(DXL_ID, 128)) {
    delay(1000);
    DEBUG_SERIAL.print("Present Velocity : ");
    DEBUG_SERIAL.println(dxl.getPresentVelocity(DXL_ID));
    DEBUG_SERIAL.println();
  }
  /////////////////////////////////////////////////////////////////////////
}

// the loop function runs over and over again forever
void loop() {
  mega.rjLedWrite(PORT3, mega.rjIrRead(PORT2));
}
