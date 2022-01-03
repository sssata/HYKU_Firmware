#define FLASH_DEBUG       0

#include <FlashStorage_SAMD.h>

// Note: the area of flash memory reserved for the variable is
// lost every time the sketch is uploaded on the board.

void setup()
{
  Serial.begin(115200);
  while (!Serial);

  delay(200);

  Serial.print(F("\nStart FlashStoreAndRetrieve on ")); Serial.println(BOARD_NAME);
  Serial.println(FLASH_STORAGE_SAMD_VERSION);

  Serial.print("EEPROM length: ");
  Serial.println(EEPROM.length());

  uint16_t address = 0;
  int number;

  // Read the content of emulated-EEPROM
  EEPROM.get(address, number);

  // Print the current number on the serial monitor
  Serial.print("Number = 0x"); Serial.println(number, HEX);

  // Save into emulated-EEPROM the number increased by 1 for the next run of the sketch
  EEPROM.put(address, (int) (number + 1));

  if (!EEPROM.getCommitASAP())
  {
    Serial.println("CommitASAP not set. Need commit()");
    EEPROM.commit();
  }

  Serial.println("Done writing to emulated EEPROM. You can reset now");
}

void loop()
{
  // Do nothing...
}