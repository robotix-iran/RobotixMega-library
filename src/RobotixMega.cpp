/*
    This file is part of the RobotixMega library.

*/

#include "RobotixMega.h"

RobotixMega::RobotixMega() : imu(Wire1, IMU_ADDRESS)
{
  u8g2 = U8G2_STE2007_96X68_F_3W_SW_SPI(U8G2_R0, /* clock=*/PIN_LCD_SCK, /* data=*/PIN_LCD_MOSI, /* cs=*/PIN_LCD_CS, /* reset=*/PIN_LCD_RESET);
}

int RobotixMega::begin()
{
  pinMode(PIN_LED1, OUTPUT);
  pinMode(PIN_LED2, OUTPUT);
  pinMode(PIN_LED3, OUTPUT);
  pinMode(PIN_LED4, OUTPUT);

  pinMode(motor_1.IN_1, OUTPUT);
  pinMode(motor_1.IN_2, OUTPUT);
  pinMode(motor_2.IN_1, OUTPUT);
  pinMode(motor_2.IN_2, OUTPUT);
  pinMode(motor_3.IN_1, OUTPUT);
  pinMode(motor_3.IN_2, OUTPUT);
  pinMode(motor_4.IN_1, OUTPUT);
  pinMode(motor_4.IN_2, OUTPUT);

  pinMode(PIN_BT1, INPUT);
  pinMode(PIN_BT2, INPUT);
  pinMode(PIN_BT3, INPUT);
  pinMode(PIN_BT4, INPUT);

  pinMode(PIN_LCD_BKL, OUTPUT);
  digitalWrite(PIN_LCD_BKL, HIGH);

  servo_1.attach(PIN_SERVO_1);
  servo_2.attach(PIN_SERVO_2);
  servo_3.attach(PIN_SERVO_3);
  servo_4.attach(PIN_SERVO_4);
  servo_5.attach(PIN_SERVO_5);
  servo_6.attach(PIN_SERVO_6);
  servo_7.attach(PIN_SERVO_7);
  servo_8.attach(PIN_SERVO_8);

  u8g2.begin();
  u8g2.setFont(u8g2_font_6x10_tf);
  u8g2.setFontRefHeightExtendedText();
  u8g2.setDrawColor(1);
  u8g2.setFontPosTop();
  u8g2.setFontDirection(0);

  int status = imu.begin();

  // setting the accelerometer full scale range to +/-8G
  imu.setAccelFS(ICM42688::gpm8);
  // setting the gyroscope full scale range to +/-500 deg/s
  imu.setGyroFS(ICM42688::dps500);

  // set output data rate to 25 Hz
  imu.setAccelODR(ICM42688::odr1k);
  imu.setGyroODR(ICM42688::odr1k);

  digitalWrite(PIN_LED1, ON);
  delay(50);
  digitalWrite(PIN_LED1, LOW);
  delay(20);
  digitalWrite(PIN_LED2, ON);
  delay(50);
  digitalWrite(PIN_LED2, LOW);
  delay(20);
  digitalWrite(PIN_LED3, ON);
  delay(50);
  digitalWrite(PIN_LED3, LOW);
  delay(20);
  digitalWrite(PIN_LED4, ON);
  delay(50);
  digitalWrite(PIN_LED4, LOW);
  delay(20);

  return 0;
}

void RobotixMega::setLED(uint8_t led, uint8_t state)
{
  digitalWrite(led, state);
}

uint8_t RobotixMega::readBTN(uint8_t btn)
{
  return digitalRead(btn);
}

void RobotixMega::playSongPri(const int *melody, int notes)
{

  // this calculates the duration of a whole note in ms
  int wholenote = (60000 * 4) / 180;

  int divider = 0, noteDuration = 0;

  for (int thisNote = 0; thisNote < notes * 2; thisNote = thisNote + 2)
  {
    // calculates the duration of each note
    divider = melody[thisNote + 1];
    if (divider > 0)
    {
      // regular note, just proceed
      noteDuration = (wholenote) / divider;
    }
    else if (divider < 0)
    {
      // dotted notes are represented with negative durations!!
      noteDuration = (wholenote) / abs(divider);
      noteDuration *= 1.5; // increases the duration in half for dotted notes
    }

    // we only play the note for 90% of the duration, leaving 10% as a pause
    tone(PIN_BUZZER, melody[thisNote], noteDuration * 0.9);

    // Wait for the specief duration before playing the next note.
    delay(noteDuration);

    // stop the waveform generation before the next note.
    noTone(PIN_BUZZER);
  }
}

void RobotixMega::playSong(uint8_t song)
{
  int notes = 0;

  switch (song)
  {
  case NOKIA:
    notes = sizeof(nokia_ringtones) / sizeof(nokia_ringtones[0]) / 2;
    playSongPri(nokia_ringtones, notes);
    break;
  case PINKPANTHER:
    notes = sizeof(pink_panther) / sizeof(pink_panther[0]) / 2;
    playSongPri(pink_panther, notes);
    break;
  case GAMEOFTHRONES:
    notes = sizeof(game_of_thrones) / sizeof(game_of_thrones[0]) / 2;
    playSongPri(game_of_thrones, notes);
    break;
  case HAPPYBIRTHDAY:
    notes = sizeof(happy_birthday) / sizeof(happy_birthday[0]) / 2;
    playSongPri(happy_birthday, notes);
    break;
  case PACMAN:
    notes = sizeof(pacman) / sizeof(pacman[0]) / 2;
    playSongPri(pacman, notes);
    break;
  case TETRIS:
    notes = sizeof(tetris) / sizeof(tetris[0]) / 2;
    playSongPri(tetris, notes);
    break;
  case THEGODFATHER:
    notes = sizeof(godfather) / sizeof(godfather[0]) / 2;
    playSongPri(godfather, notes);
    break;
  default:
    break;
  }
}

void RobotixMega::buzzerTone(uint32_t note)
{
  tone(PIN_BUZZER, note);
}

void RobotixMega::buzzerNoTone()
{
  noTone(PIN_BUZZER);
}

void RobotixMega::setMotorSpeed(motor_port motor, int32_t speed)
{
  if (speed > 0)
  {
    analogWrite(motor.IN_1, speed);
    digitalWrite(motor.IN_2, 0);
  }
  else if (speed < 0)
  {
    analogWrite(motor.IN_1, 255 - (speed * -1));
    digitalWrite(motor.IN_2, 1);
  }
  else
  {
    analogWrite(motor.IN_1, 255);
    digitalWrite(motor.IN_2, 1);
  }
}

void RobotixMega::rotate(uint8_t direction, int32_t speed)
{
}

void RobotixMega::move(uint8_t direction, int32_t speed)
{
}

void RobotixMega::brake()
{
}

float RobotixMega::getTemp(){
  imu.getAGT();
  return imu.temp();
}

void RobotixMega::getOrientation(float &roll, float &pitch, float &yaw)
{
}

void RobotixMega::getAccelerations(float &ax, float &ay, float &az)
{
  imu.getAGT();

  ax = imu.accX();
  ay = imu.accY();
  az = imu.accZ();
}

void RobotixMega::getGyros(float &gx, float &gy, float &gz)
{
  imu.getAGT();

  gx = imu.gyrX();
  gy = imu.gyrY();
  gz = imu.gyrZ();
}

void RobotixMega::getIMU(float &ax, float &ay, float &az, float &gx, float &gy, float &gz)
{
  imu.getAGT();

  ax = imu.accX();
  ay = imu.accY();
  az = imu.accZ();

  gx = imu.gyrX();
  gy = imu.gyrY();
  gz = imu.gyrZ();
}

void RobotixMega::setServoPositions(Servo servo, uint8_t position)
{
  servo.write(position);
}

void RobotixMega::showLogo(uint8_t logo)
{
  switch (logo)
  {
  case LOGO_ROBOTIX:
    u8g2.drawXBM(0, 0, 96, 68, LogoRobotix);
    break;
  case LOGO_ARDUINO:
    u8g2.drawXBM(0, 0, 96, 68, LogoArduino);
    break;

  default:
    break;
  }
  u8g2.sendBuffer();
}

void RobotixMega::drawString(uint8_t x, uint8_t y, const char *s)
{
  u8g2.drawStr(x, y, s);
  u8g2.sendBuffer();
}

void RobotixMega::clearRectangle(uint16_t x, uint16_t y, uint16_t w, uint16_t h)
{
  u8g2.setDrawColor(0);
  u8g2.drawBox(x,y,w,h);
  u8g2.setDrawColor(1);
  u8g2.sendBuffer();
}

void RobotixMega::clearLCD()
{
  u8g2.clear();
}

// RJ PORTS/////////////////////////////////
uint8_t RobotixMega::rjSrfInit(rj_port port)
{
  port.type = MOD_SRF;
  pinMode(port.pin_B, OUTPUT); // TRIG
  pinMode(port.pin_A, INPUT);  // ECHO
}

float RobotixMega::rjSrfRead(rj_port port)
{
  long duration;

  digitalWrite(port.pin_B, LOW);
  delayMicroseconds(2);
  // Set the TriggerPin on HIGH state for 10 microseconds
  digitalWrite(port.pin_B, HIGH);
  delayMicroseconds(10);
  digitalWrite(port.pin_B, LOW);
  // Read the EchoPin, returning the sound wave travel time in microseconds
  duration = pulseIn(port.pin_A, HIGH);
  // Calculate and return the distance
  return duration * 0.034 / 2;
}

uint8_t RobotixMega::rjSwInit(rj_port port)
{
  port.type = MOD_SWITCH;
  pinMode(port.pin_A, INPUT_PULLDOWN);
}

uint8_t RobotixMega::rjSwRead(rj_port port)
{
  return digitalRead(port.pin_A);
}

uint8_t RobotixMega::rjLedInit(rj_port port)
{
  port.type = MOD_LED;
  pinMode(port.pin_B, OUTPUT);
}

void RobotixMega::rjLedWrite(rj_port port, uint8_t state)
{
  digitalWrite(port.pin_B, state);
}

uint8_t RobotixMega::rjIrInit(rj_port port)
{
  port.type = MOD_IR;
  pinMode(port.pin_A, INPUT);
}
uint8_t RobotixMega::rjIrRead(rj_port port)
{
  return digitalRead(port.pin_A);
}