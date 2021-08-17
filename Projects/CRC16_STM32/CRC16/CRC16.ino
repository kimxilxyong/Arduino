/*
  CRC16

  Test crc16 on Arduino and stm32duino by reading calibration data from EEPROM

  modified 2 Aug 2021
  by Kim Il


  This example code is in the public domain.

*/

#include <Wire.h>
#include <EEPROM.h>
#include <elapsedMillis.h> 
#include <util/crc16.h>

#define NXP_MOTION_CAL_EEADDR  60
#define NXP_MOTION_CAL_SIZE    68

elapsedMillis millisRuntime;

unsigned char buf[NXP_MOTION_CAL_SIZE];
float cal[16]; // 0-8=offsets, 9=field strength, 10-15=soft iron map


// the setup function runs once when you press reset or power the board
void setup() {
  Serial.begin(115200);
  while (!Serial); // STM32F767ZI: wait for serial monitor
  // initialize digital pin LED_BUILTIN as an output.
  pinMode(LED_BUILTIN, OUTPUT);
  Serial.println("Setup 115200 for read_calibration() done");
}

// the loop function runs over and over again forever
void loop() {
  digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)

  uint8_t result = read_calibration();
  digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW

  if (result == 0) {
    Serial.println("read_calibration OK, dumping ...");
    for (int i = 0; i < NXP_MOTION_CAL_SIZE; i++) {
      Serial.print(", 0x");
      Serial.print(buf[i], HEX);
      if (((i-1) % 8 == 0) || (i+1 == NXP_MOTION_CAL_SIZE) ) {
        Serial.println();
      }
    }
    Serial.println("read_calibration dump finished");
  } else {
    Serial.print("read_calibration failed with code ");
    Serial.print(result);
  }
                       
  Serial.print("Loop time in millis ");
  Serial.println(millisRuntime);
  delay(1000);
}

uint8_t read_calibration() {
	uint8_t i;
  uint8_t r = 1;
	uint16_t crc;

  for (i=0; i < NXP_MOTION_CAL_SIZE; i++) {
		buf[i] = EEPROM.read(NXP_MOTION_CAL_EEADDR + i);
	}
	crc = 0xFFFF;
	for (i=0; i < NXP_MOTION_CAL_SIZE; i++) {
		crc = _crc16_update(crc, buf[i]);
	}
	if (crc == 0 && buf[0] == 117 && buf[1] == 84) {
		memcpy(cal, buf+2, sizeof(cal));
    r = 0;
	} else {
		memset(cal, 0, sizeof(cal));
		cal[9] = 40.0f;
    r = 2;
	}
  return r;
}

/*
Setup 115200 for read_calibration() done
read_calibration OK, d�Setup 115200 for read_calibration() done
read_calibration OK, dumping ...
, 0x75, 0x54
, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0
, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0
, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0
, 0x2F, 0xAC, 0x91, 0x41, 0x0, 0xB0, 0xAD, 0xBE
, 0x4A, 0x3E, 0x8E, 0x42, 0x66, 0xEE, 0x23, 0x42
, 0x1E, 0xF0, 0x82, 0x3F, 0xF6, 0x5F, 0x75, 0x3F
, 0x52, 0xB4, 0x82, 0x3F, 0x48, 0x2E, 0x96, 0xBC
, 0xA0, 0x7B, 0xE6, 0x3C, 0x40, 0x40, 0x21, 0x3C
, 0x9D, 0x2E
Loop time in millis 38
read_calibration OK, dumping ...
, 0x75, 0x54
, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0
, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0
, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0
, 0x2F, 0xAC, 0x91, 0x41, 0x0, 0xB0, 0xAD, 0xBE
, 0x4A, 0x3E, 0x8E, 0x42, 0x66, 0xEE, 0x23, 0x42
, 0x1E, 0xF0, 0x82, 0x3F, 0xF6, 0x5F, 0x75, 0x3F
, 0x52, 0xB4, 0x82, 0x3F, 0x48, 0x2E, 0x96, 0xBC
, 0xA0, 0x7B, 0xE6, 0x3C, 0x40, 0x40, 0x21, 0x3C
, 0x9D, 0x2E
read_calibration dump finished
Loop time in millis 80
*/
