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

void setup()
{
  mega.begin();
  mega.showLogo(LOGO_ROBOTIX);
  mega.rjSrfInit(PORT1);
  mega.rjIrInit(PORT2);
  mega.rjLedInit(PORT3);


  Serial.begin(115200);
}

// the loop function runs over and over again forever
void loop()
{

  mega.rjLedWrite(PORT3, mega.rjIrRead(PORT2));

  currentMillis = millis();
  if (currentMillis - startMillis >= period)
  {
    float distance = mega.rjSrfRead(PORT1);
    sprintf(temp, "distance: %3.1f ",distance);
    mega.clearLCD();
    mega.drawString(0,10,temp);
    Serial.print(temp);
    startMillis = currentMillis;
  }
}