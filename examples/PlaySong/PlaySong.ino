#include <RobotixMega.h>

RobotixMega mega;

void setup() {
  mega.begin();

  mega.clearLCD();
  mega.drawString(25, 10, "Playing:");
  mega.drawString(35, 30, "NOKIA");
  mega.playSong(NOKIA);

  mega.clearLCD();
  mega.drawString(25, 10, "Playing:");
  mega.drawString(17, 30, "PINKPANTHER");
  mega.playSong(PINKPANTHER);

  mega.clearLCD();
  mega.drawString(25, 10, "Playing:");
  mega.drawString(10, 30, "GAMEOFTHRONES");
  mega.playSong(GAMEOFTHRONES);

  mega.clearLCD();
  mega.drawString(25, 10, "Playing:");
  mega.drawString(10, 30, "HAPPYBIRTHDAY");
  mega.playSong(HAPPYBIRTHDAY);

  mega.clearLCD();
  mega.drawString(25, 10, "Playing:");
  mega.drawString(30, 30, "PACMAN");
  mega.playSong(PACMAN);

  mega.clearLCD();
  mega.drawString(25, 10, "Playing:");
  mega.drawString(30, 30, "TETRIS");
  mega.playSong(TETRIS);

  mega.clearLCD();
  mega.drawString(25, 10, "Playing:");
  mega.drawString(15, 30, "THEGODFATHER");
  mega.playSong(THEGODFATHER);

  mega.clearLCD();
  mega.drawString(30, 10, "FINISH");
}

// the loop function runs over and over again forever
void loop() {
}