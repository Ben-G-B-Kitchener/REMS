#define RF69_COMPAT 1
#include <JeeLib.h>

typedef struct {
  byte node;
  byte node_status;
  float v_batt;
  int temp_internal;
  unsigned long time_on;
  float temperature;
  float humidity;
} Payload;

void setup () {
  int nodeId = eeprom_read_byte(RF12_EEPROM_ADDR + 0);
  int group  = eeprom_read_byte(RF12_EEPROM_ADDR + 1);
  long frequency = eeprom_read_word((uint16_t*) (RF12_EEPROM_ADDR + 4));
  rf12_initialize(nodeId, RF12_868MHZ, group, frequency); // base node ID = 1
  Serial.begin(115200);
}

void loop () {
    if (rf12_recvDone() && rf12_crc == 0) { 
      const Payload* p = (const Payload*) rf12_data;
      Serial.print((word) p->node);
      Serial.print("\t");
      Serial.print(p->node_status);
      Serial.print("\t");
      Serial.print(p->v_batt);
      Serial.print("\t");
      Serial.print(p->temp_internal);
      Serial.print("\t");
      Serial.print(p->time_on);
      Serial.print("\t");
      Serial.print(p->temperature);
      Serial.print("\t");
      Serial.print(p->humidity);
      Serial.print("\t");
      Serial.print(-(RF69::rssi>>1));
      Serial.println();
    }
}
