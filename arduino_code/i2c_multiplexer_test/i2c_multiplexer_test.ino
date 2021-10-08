/**
 * TCA9548 I2CScanner.ino -- I2C bus scanner for Arduino
 *
 * Based on https://playground.arduino.cc/Main/I2cScanner/
 *
 */

#include "Wire.h"
#include <AS5600.h>

#define TCAADDR 0x70
AMS_5600 ams5600_shoulder;

void tcaselect(uint8_t i) {
  if (i > 7) return;
 
  Wire.beginTransmission(TCAADDR);
  Wire.write(1 << i);
  Wire.endTransmission();  
}


// standard Arduino setup()
void setup()
{
    while (!Serial);
    delay(1000);

    Wire.begin();
    Wire.setClock(1000);
    Serial.begin(57600);
    Serial.println("\nTCAScanner ready!");
    
    for (uint8_t t=0; t<8; t++) {
      tcaselect(t);
      Serial.print("TCA Port #"); Serial.println(t);

      for (uint8_t addr = 0; addr<=127; addr++) {
        if (addr == TCAADDR) continue;

        Wire.beginTransmission(addr);
        if (!Wire.endTransmission()) {
          Serial.print("Found I2C 0x");  Serial.println(addr,HEX);
        }
      }
    }
    Serial.println("\ndone");
}

void loop() 
{
  tcaselect(2);
  Serial.print("2: ");
  Serial.println(ams5600_shoulder.getRawAngle());
  tcaselect(1);
  Serial.print("1: ");
  Serial.println(ams5600_shoulder.getRawAngle());
  tcaselect(3);
  Serial.print("3: ");
  Serial.println(ams5600_shoulder.getRawAngle());
  tcaselect(0);
  Serial.print("0: ");
  Serial.println(ams5600_shoulder.getRawAngle());
  tcaselect(4);
  Serial.print("4: ");
  Serial.println(ams5600_shoulder.getRawAngle());

}
