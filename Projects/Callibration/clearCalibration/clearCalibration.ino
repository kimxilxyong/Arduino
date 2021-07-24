#include <EEPROM.h>

#define NXP_MOTION_CAL_EEADDR  60
#define NXP_MOTION_CAL_SIZE    68

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  while (!Serial) delay(10);

  delay(100);
  Serial.println("Deleting Calibration data ...");
  clearCalibration();
  delay(100);  
}

void loop() {
  Serial.println("------------");
  Serial.println("Stored calibration data");
  printSavedCalibration();

  delay(1000);
}

bool printSavedCalibration(void) {
  
  uint8_t i;
  for (i=0; i < NXP_MOTION_CAL_SIZE; i++) {
    uint8_t c = EEPROM.read(NXP_MOTION_CAL_EEADDR + i);
    Serial.print("0x");
    if (c < 0x10)
      Serial.print('0');
    Serial.print(c, HEX);
    Serial.print(", ");
    if ((i - NXP_MOTION_CAL_EEADDR) % 16 == 15) {
      Serial.println();
    }
  }
}

bool clearCalibration()
{
  uint8_t i;
	for (i=0; i < NXP_MOTION_CAL_SIZE; i++) {
		EEPROM.write(NXP_MOTION_CAL_EEADDR + i, 0x00);
	}
}