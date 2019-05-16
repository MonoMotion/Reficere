/*************************************
  ControlCore - YamaX Arduino Core v3.0
  Provides servo controlling feature
  made by Nyanyan & coord.e, Y-modify
  Copyright © 2017 Y-modify All Rights Reserved.
*************************************/

#include <IcsSoftSerialClass.h>

//definitions about ICS servo (KONDO)
const byte S_RX_PIN = 8;
const byte S_TX_PIN = 12;
const byte EN_PIN = 2;
const long BAUDRATE = 115200;
const int TIMEOUT = 200;
IcsSoftSerialClass krs(S_RX_PIN, S_TX_PIN, EN_PIN, BAUDRATE, TIMEOUT);

//definition about servos (depend on robot)
#define  numOfServo  6
float distance[numOfServo];
int servoPort[numOfServo] = {3, 4, 5, 0, 1, 2}; //右上から
float standDefinition[numOfServo] = {0, -1, -2, -3, -5, -2};
float formerDeg[numOfServo];
float reset[numOfServo] = {0, 0, 0, 0, 0, 0};


void ServoInit()
{
  krs.begin();
}

/*****Core Functions*****/
void stand()
{
  for (int i = 0; i < numOfServo; i++)
    setServoPulse(i, 0);
}

void setServoPulse(uint8_t servoNumber, float degress)
{
  if (abs(standDefinition[servoNumber] + degress) > 95)
  {
    krs.setPos(servoPort[servoNumber], 0);
    Serial.println("DEG ERROR");
  } else {
    krs.setPos(servoPort[servoNumber], krs.degPos(standDefinition[servoNumber] + degress));
    formerDeg[servoNumber] = degress;
  }
}


void smoothmotion(uint8_t servoNumber, float goal, uint8_t times, uint8_t now, uint8_t mode) //times:cutting time  now:now cut degress = form stand, +or- some  mode: 0,nomal;1,sin curve
{
  float cut = 0;
  if (now == 1)
    distance[servoNumber] = goal - formerDeg[servoNumber];

  if (mode == 0)
    cut = (goal - formerDeg[servoNumber]) / (float)(times - now + 1);
  else
    cut = (sin(now * PI / times - PI / 2) - sin((now - 1) * PI / times - PI / 2)) * distance[servoNumber] / 2;
  setServoPulse(servoNumber, formerDeg[servoNumber] + cut);
}

void convert(float deg[], int times, int frame, int mode)
{
  for (int i = 1; i <= times; i++)
  {
    for (int j = 0; j < numOfServo; j++)
      smoothmotion(j, deg[j] , times, i, mode);
    delay(frame);
  }
}


void setdeg(uint8_t servoNumber, int degress)
{
  int times = 20;
  for (int i = 1; i <= times; i++)
  {
    smoothmotion(servoNumber, degress, times, i, 0);
    delay(20);
  }
  Serial.print("set deg finished");
  Serial.print(servoNumber);
  Serial.print("\t");
  Serial.println(degress);
}
