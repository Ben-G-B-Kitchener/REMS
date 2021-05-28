#define RF69_COMPAT 1
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <avr/wdt.h>
#include <avr/power.h>
#include <JeeLib.h>
#include <Wire.h>
#include <SHT3x.h>


SHT3x Sensor( 0x44,         //Set the address
                SHT3x::Zero,  //Functions will return zeros in case of error
                255,          //If you DID NOT connected RESET pin
                SHT3x::SHT35, //Sensor type
                SHT3x::Single_LowRep_ClockStretch //Low repetability mode
                );

boolean ping = false;

byte node_status = 0b00000000;
// bit number (n) =   76543210
// Heartbeat_Ping = 0b00000001  forces RF data transmission every 10 sleep cycles (1 sleep cycle = 60 seconds).
// SHT_Error      = 0b00000010  Detects faulty/missing SHT35 sensor.
// Batt_Low       = 0b00000100  Detects regulated input voltage <= 3.0V. This indicates that connected 9V battery is close to dead.
// <unassigned>   = 0b00001000
// <unassigned>   = 0b00010000
// <unassigned>   = 0b00100000
// <unassigned>   = 0b01000000
// <unassigned>   = 0b10000000

byte setBit(byte data, byte value, byte n){
  byte result = data;
  if (value == 0){
    result &= ~ (1UL << n);
  }
  else if (value == 1){
    result |= (1UL << n);
  }
  return result;
}
typedef struct {
  byte node;
  byte node_status;
  float v_batt;
  int temp_internal;
  unsigned long time_on;
  float temperature;
  float humidity;
} Payload;

Payload payload;

unsigned int tempCal;
unsigned int wake_count = 0;
float previous_temp = -128;
float previous_hum = -128;
float previous_batt = -128;
int previous_int_temp = -128;

static void rfwrite(){
  rf12_sleep(-1);     //wake up RF module
  rf12_recvDone();
  rf12_sendNow(0, &payload, sizeof payload);
  delay(10);
  rf12_sleep(0);    //put RF module to sleep
}

void wake ()                            
{
  wdt_disable();  // disable watchdog
}  // end of wake

// watchdog interrupt
ISR (WDT_vect) 
{
  wake ();
}  // end of WDT_vect

void watchdogEnable (const byte interval) 
{ 
  noInterrupts();   // timed sequence below

  MCUSR = 0;                          // reset various flags
  WDTCSR |= 0b00011000;               // see docs, set WDCE, WDE
  WDTCSR =  0b01000000 | interval;    // set WDIE, and appropriate delay
  wdt_reset();
  
  byte adcsra_save = ADCSRA;
  ADCSRA = 0;  // disable ADC
  power_all_disable();   // turn off all modules
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);   // sleep mode is set here
  sleep_enable();
  interrupts();
  sleep_cpu();            // now goes to Sleep and waits for the interrupt  
  
  ADCSRA = adcsra_save;  // stop power reduction
  power_all_enable();   // turn on all modules
}  // end of watchdogEnable

void setup()
{
  int nodeId = eeprom_read_byte(RF12_EEPROM_ADDR + 0);
  int group  = eeprom_read_byte(RF12_EEPROM_ADDR + 1);
  long frequency = eeprom_read_word((uint16_t*) (RF12_EEPROM_ADDR + 4));
  rf12_initialize(nodeId, RF12_868MHZ, group, frequency);
  payload.node = nodeId & RF12_HDR_MASK;
  tempCal = eeprom_read_word(0x60);
  Wire.begin();
  readSensors();
  rfwrite(); 
}  // end of setup

void readSensors(){
  // read internal temperature
  ADCSRA =  bit (ADEN);   // turn ADC on
  ADCSRA |= bit (ADPS0) |  bit (ADPS1) | bit (ADPS2);  // Prescaler of 128
  ADMUX = bit (REFS0) | bit (REFS1)  | 0x08;    // internal temperature sensor
  delay (20);  // let it stabilize
  bitSet (ADCSRA, ADSC);  // start a conversion  
  while (bit_is_set(ADCSRA, ADSC)){}  // wait for conversion to complete
  int t = ADC - tempCal;
  ADMUX = bit(REFS0) | 14; // use VCC and internal bandgap
  delay (20);  // let it stabilize
  bitSet (ADCSRA, ADSC);  // start a conversion  
  while (bit_is_set(ADCSRA, ADSC)){}  // wait for conversion to complete
  // convert ADC readings to fit in one byte, i.e. 20 mV steps:
  // 1.0V = 0, 1.8V = 40, 3.3V = 115, 5.0V = 200, 6.0V = 250
  byte b = (55U * 1023U) / (ADC + 1) - 50 ;
  float batt = ((float)b * 0.02) + 1 ;
  
  Sensor.UpdateData();
  delay(500);
  float temperature = Sensor.GetTemperature();
  float humidity = Sensor.GetRelHumidity();
  //assign data to payload struct
    //payload.node_status = node_status;
    payload.v_batt = batt;
    payload.temp_internal = t;
    payload.time_on = millis();
    payload.humidity = humidity; 
    payload.temperature = temperature;  
}

void loop()
{
  // sleep bit patterns:
  //  1 second:  0b000110
  //  2 seconds: 0b000111
  //  4 seconds: 0b100000
  //  8 seconds: 0b100001

  // sleep for 60 seconds (1 sleep-cycle)
//*************************************************************************************************  
  watchdogEnable (0b100001);  // 8 seconds
  watchdogEnable (0b100001);  // 8 seconds
  watchdogEnable (0b100001);  // 8 seconds
  watchdogEnable (0b100001);  // 8 seconds
  watchdogEnable (0b100001);  // 8 seconds
  watchdogEnable (0b100001);  // 8 seconds
  watchdogEnable (0b100001);  // 8 seconds
  watchdogEnable (0b100000);  // 4 seconds 
//*************************************************************************************************  

//  delay(1000); // used instead of sleeping during testing
  wake_count++;
  ping = false;

//*************************************************************************************************  

  readSensors();
  if (payload.v_batt <= 3.0){
    payload.node_status = setBit(payload.node_status, 1, 2); // set low battery status
  }
  else { 
    payload.node_status = setBit(payload.node_status, 0, 2); 
  }

//*************************************************************************************************  
  if (payload.temperature == -999 || payload.humidity == -999){
    payload.node_status = setBit(payload.node_status, 1, 1); // set SHT_Error status
  }
  else { 
    payload.node_status = setBit(payload.node_status, 0, 1); 
  }
  
//*************************************************************************************************  
//*************************************************************************************************  

    if (wake_count > 9){
      payload.node_status = setBit(payload.node_status, 1, 0); // this is a ping - data is transmitted regardless of change
      wake_count = 0;
      ping = true;
    }
    else {
      payload.node_status = setBit(payload.node_status, 0, 0);
    }

 
  // if internal temperature has changed then transmit data, or send a ping if 10 mins has elapsed
  if (payload.temperature > (previous_temp + 0.5) || payload.temperature < (previous_temp - 0.5)  ||
      payload.humidity > (previous_hum + 0.5) || payload.humidity < (previous_hum - 0.5) || 
      payload.temp_internal > (previous_int_temp + 2) || payload.temp_internal < (previous_int_temp - 2) ||
      payload.v_batt < previous_batt || wake_count > 9)
      {
        ping = true;
      }  
  
  if (ping){ 
//  if (true){ 
    // send rf data
    rfwrite();
//    Serial.println(payload.temperature);
    } 

  previous_temp = payload.temperature;
  previous_hum = payload.humidity;
  previous_batt = payload.v_batt;
  previous_int_temp = payload.temp_internal;
//*************************************************************************************************  
  
}  // end of loop
