//
// Reficere system for YamaX 7.0 or any robot
// Copyright © 2016-2019 Nyanyan All Rights Reserved.
//
// This file is part of Reficere.
//
// Reficere is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// Reficere is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with Reficere.  If not, see <http://www.gnu.org/licenses/>.
//

/*****Version Definition*****/
#define YAMAX_VER "7.0"
#define FIRMWARE_VER "7.0-uno"

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
void loop() { Reficere(1); }

/*****That's all. Enjoy!*****/
