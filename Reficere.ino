/*************************************
  Reficere system for YamaX 7.0 or any robot
  made by Y-modify, Nyanyan & coord.e
  Copyright © 2016-2019 Y-modify All Rights Reserved.
*************************************/

/*****Version Definition*****/
#define YAMAX_VER "7.0"
#define FIRMWARE_VER 7.0-uno"

#if defined(ARDUINO_AVR_UNO) || defined(ESP8266) // AX-01 or something nice
#define S_STABLE
#define BOARD_VER "01"
#else
#error This board is not supported! Please use ESP8266 or Arduino UNO.
#endif

/*****System Setup*****/
void setup() {
  Serial.begin(115200);
  delay(1000);

  ServoInit(); // Servo Setup
  StabilizationInit();

  stand();
  Serial.println("Reficere test");

  setScaleWeight();
}

/*****Waiting Loop*****/
void loop() {
  Reficere(1);
}

/*****That's all. Enjoy!*****/
