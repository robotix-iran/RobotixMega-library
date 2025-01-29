/*
    This file is part of the RobotixMega library.

*/

#ifndef __DEFINITIONS_H__
#define __DEFINITIONS_H__

#include "Arduino.h"

#define ON 1
#define OFF 0

#define IOPORT_CFG_PWM (IOPORT_CFG_PERIPHERAL_PIN | IOPORT_PERIPHERAL_GPT1)

#define IMU_ADDRESS 0x68

// SONG////////////////////////////////
#define NOKIA 0
#define PINKPANTHER 1
#define GAMEOFTHRONES 2
#define HAPPYBIRTHDAY 3
#define PACMAN 4
#define TETRIS 5
#define THEGODFATHER 6
///////////////////////////////////////



// MOTOR///////////////////////////////
#define LEFT 0
#define RIGHT 1

#define FORWARD 0
#define BACKWARD 1
///////////////////////////////////////



// LOGO////////////////////////////////
#define LOGO_ROBOTIX 0
#define LOGO_ARDUINO 1
///////////////////////////////////////



// PINS////////////////////////////////

////////////////////LCD
#define PIN_LCD_MOSI 28
#define PIN_LCD_SCK 29
#define PIN_LCD_RESET 30
#define PIN_LCD_CS 31
#define PIN_LCD_BKL 32
///////////////////Button
#define PIN_BT1 33
#define PIN_BT2 34
#define PIN_BT3 35
#define PIN_BT4 69
///////////////////MPU
#define PIN_IMU_SCK 37
#define PIN_IMU_SDA 38
///////////////////LED
#define PIN_LED1 LED_BUILTIN
#define PIN_LED2 39
#define PIN_LED3 40
#define PIN_LED4 41
////////////////////BUZZ
#define PIN_BUZZER 68
///////////////////MOTOR
#define PIN_M1_IN1 66
#define PIN_M1_IN2 67
#define PIN_M2_IN1 65
#define PIN_M2_IN2 64
#define PIN_M3_IN1 62
#define PIN_M3_IN2 63
#define PIN_M4_IN1 59
#define PIN_M4_IN2 61
////////////////////IO
#define PIN_PORT1_R 42
#define PIN_PORT1_L 43
#define PIN_PORT2_R 44
#define PIN_PORT2_L 45
#define PIN_PORT3_R 46
#define PIN_PORT3_L 47
////////////////////SERVO
#define PIN_SERVO_1 49
#define PIN_SERVO_2 50
#define PIN_SERVO_3 51
#define PIN_SERVO_4 52

#define PIN_SERVO_5 3
#define PIN_SERVO_6 5
#define PIN_SERVO_7 6
#define PIN_SERVO_8 10
////////////////////Dynamixel
#define PIN_MCU_RX_Dynamixel 54
#define PIN_MCU_TX_Dynamixel 55
#define PIN_Dynamixel_TX_EN 53

///////////////////////////////////////



#endif
