/*
    This file is part of the RobotixMega library.

*/

#ifndef __ROBOTIX_MEGA_H__
#define __ROBOTIX_MEGA_H__

#include "Arduino.h"
#include "definitions.h"
#include "SongNotes.h"
#include "Logo.h"
#include <EEPROM.h>
#include "Wire.h"
#include <string>
#include <U8g2lib.h>
#include "ICM42688.h"
#include <Servo.h>
#include "FspTimer.h"

class RobotixMega
{
private:
  void playSongPri(const int *melody, int notes);

  enum ModulesTypes
  {
    MOD_NONE,
    MOD_IR,
    MOD_LED,
    MOD_LDR,
    MOD_TE_HU,
    MOD_SWITCH,
    MOD_SRF
  };

  struct rj_port
  {
    ModulesTypes type;
    uint8_t pin_A;
    uint8_t pin_B;
  };

  struct motor_port
  {
    uint8_t IN_1;
    uint8_t IN_2;

    bool operator==(const motor_port &other) const
    {
      return (IN_1 == other.IN_1) && (IN_2 == other.IN_2);
    }
  };
  /*   rj_port port_2(MOD_NONE,IO_P2_R,IO_P2_L);
    rj_port port_3(MOD_NONE,IO_P3_R,IO_P3_L);

    rj_port port_4(MOD_NONE,IO_P1_R,IO_P1_L);
    rj_port port_5(MOD_NONE,IO_P1_R,IO_P1_L); */

  /*
      class RobotixMegaIMU{
        private:
        public:

      };

      class RobotixMegaMotor{
        private:
        public:
      };

      class RobotixMegaLCD{
      }; */

public:
  U8G2 u8g2;
  ICM42688 imu;

  rj_port port_1{MOD_NONE, PIN_PORT1_R, PIN_PORT1_L};
  rj_port port_2{MOD_NONE, PIN_PORT2_R, PIN_PORT2_L};
  rj_port port_3{MOD_NONE, PIN_PORT3_R, PIN_PORT3_L};

  motor_port motor_1{PIN_M1_IN1, PIN_M1_IN2};
  motor_port motor_2{PIN_M2_IN1, PIN_M2_IN2};
  motor_port motor_3{PIN_M3_IN1, PIN_M3_IN2};
  motor_port motor_4{PIN_M4_IN1, PIN_M4_IN2};

  Servo servo_1;
  Servo servo_2;
  Servo servo_3;
  Servo servo_4;
  Servo servo_5;
  Servo servo_6;
  Servo servo_7;
  Servo servo_8;

  RobotixMega();

  int begin();

  void setLED(uint8_t led, uint8_t state);
  uint8_t readBTN(uint8_t btn);

  void playSong(uint8_t song);
  void buzzerTone(uint32_t tone);
  void buzzerNoTone();

  void setMotorSpeed(motor_port motor, int32_t speed);

  void rotate(uint8_t direction, int32_t speed);
  void move(uint8_t direction, int32_t speed);

  void brake();

  float getTemp();
  void getOrientation(float &roll, float &pitch, float &yaw);
  void getAccelerations(float &ax, float &ay, float &az);
  void getGyros(float &gx, float &gy, float &gz);
  void getIMU(float &ax, float &ay, float &az, float &gx, float &gy, float &gz);

  void setServoPositions(Servo servo, uint8_t position);

  void showLogo(uint8_t logo);

  void drawString(uint8_t x, uint8_t y, const char *s);
  void clearRectangle(uint16_t x, uint16_t y, uint16_t w, uint16_t h);
  void clearLCD();

  // SRF-HC05
  uint8_t rjSrfInit(rj_port port);
  float rjSrfRead(rj_port port);

  // VLX
  uint8_t rj_vlxInit(rj_port port);
  uint16_t rj_vlx_read(rj_port port);

  // IR
  uint8_t rjIrInit(rj_port port);
  uint8_t rjIrRead(rj_port port);

  // Switch
  uint8_t rjSwInit(rj_port port);
  uint8_t rjSwRead(rj_port port);

  // LED
  uint8_t rjLedInit(rj_port port);
  void rjLedWrite(rj_port port, uint8_t state);

  // Temp
  uint8_t rjTempHumInit(rj_port port);
  int16_t rjHumRead(rj_port port);
  int16_t rjTempRead(rj_port port);

  // LDR
  uint8_t rjLDRInit(rj_port port);
  uint8_t rjLDRRead(rj_port port);

  // RGB
  uint8_t rjRGBInit(rj_port port);
  uint8_t rjRGBWrite(rj_port port, uint8_t r, uint8_t g, uint8_t b);
};

#endif