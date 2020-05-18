/*
   Soil moisture detector. Uses Sparkfun soil moisture
   detector to masure soil moisture level and report
   the result via LoRa to a Gateway.

   https://www.sparkfun.com/products/13322

   Intended to work on 3.3V Arduino Mini Pro

  Pin Assignments:
     3 -> LoRa DOI0
     4 -> LED (Optional)
     5 -> Moisture Sensor Power
     6 -> Temperature Sensor Power
     7 -> Teperature Sensor Pin
     9 -> LoRa Reset
    10 -> LoRa Clock Select (NSS)
    11 -> LoRa MISO
    12 -> LoRa MOSI
    13 -> LoRa SCK
    A0 -> Moisture Sensor Pin

  Quentin McDonald
  May 2020
*/

// comment out the next line to exclude temperature sensing
#define INCLUDE_TEMPERATURE 1

// comment out the next line to exclude LoRa etup transmission
#define INCLUDE_LORA 1



#include "LowPower.h"
#ifdef INCLUDE_LORA
#include <SPI.h>
#include <RH_RF95.h>
#endif

#include <stdlib.h>
#ifdef INCLUDE_TEMPERATURE
#include <OneWire.h>
#include <DallasTemperature.h>
#endif
#include <EEPROM.h>




#define MOISTURE_SENSOR_POWER_PIN 5
#define TEMPERATURE_SENSOR_POWER_PIN 6
#define TEMPERATURE_SENSOR_PIN 7
#define MOISTURE_SENSOR_PIN A0
#define NUM_SAMPLES 10   // Average 10 samples
#define NUM_SLEEPS 113    // 113 sleeps of 8 seconds each gives us about
// a reading each 15 minutes.

const int OPT_LED_PIN = 4;

#ifdef INCLUDE_LORA
#define RFM95_CS 10  // Clock select should be on 10
#define RFM95_RST 9  // Reset on 9
#define RFM95_INT 3  // Interrupt on 3

// Operating at 433 Mhz
#define RF95_FREQ 433.0

// Singleton instance of the radio driver
RH_RF95 rf95(RFM95_CS, RFM95_INT);

#endif

const int ID_LEN = 6;
// The station ID
char id[ID_LEN];

void flashLED( int numflash, int on_time, int off_time );

const int MAX_RETRIES = 3; // Try to send three times:

#ifdef INCLUDE_TEMPERATURE
OneWire oneWire(TEMPERATURE_SENSOR_PIN);
DallasTemperature temp_sensors(&oneWire);
#endif

// If there is a serial connection then allow configuration of the
// ID. The ID is a six character code which is stored in EEPROM
void configureID() {

  Serial.setTimeout(10000); // Will wait 10 seconds for input
  Serial.println("To configure ID enter 'y'");

  char answer;
  int i;

  int bytes_read = Serial.readBytes(&answer, 1);
  if ( bytes_read == 1 && answer == 'y') {
    Serial.println("Enter six character ID");
    bytes_read = Serial.readBytes(id, 6);
    if ( bytes_read == 6 ) {
      Serial.print("Id = ");
      Serial.println(id);
      for (  i = 0; i < ID_LEN; i++ ) {
        EEPROM.write(i, id[i]);
      }
    }
  }
  for (  i = 0; i < ID_LEN; i++ ) {
    id[i] = EEPROM.read(i);
  }

  Serial.print("Using station ID:" );
  Serial.println(id);

}

// Configure the LoRa radio
void setupLoRa() {


#ifdef INCLUDE_LORA

  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);



  Serial.println("Initializing LoRa radio");

  // manual reset
  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);

  while (!rf95.init()) {
    Serial.println("LoRa radio init failed");
    while (1);
  }

  Serial.println("LoRa radio init OK");
  // Defaults after init are 434.0MHz, modulation GFSK_Rb250Fd250, +13dbM
  if (!rf95.setFrequency(RF95_FREQ)) {
    Serial.println("setFrequency failed");
    while (1);
  }

  Serial.print("Set Freq to: "); Serial.println(RF95_FREQ);


  // Defaults after init are 434.0MHz, 13dBm, Bw = 125 kHz, Cr = 4/5, Sf = 128chips/symbol, CRC on

  // The default transmitter power is 13dBm, using PA_BOOST.
  // If you are using RFM95/96/97/98 modules which uses the PA_BOOST transmitter pin, then
  // you can set transmitter powers from 5 to 23 dBm:
  rf95.setTxPower(23, false);

  // Set to slow speed for longer range
  rf95.setModemConfig(RH_RF95::Bw31_25Cr48Sf512);
#endif
}

void sendData( int value, const char* code ) {

#ifdef INCLUDE_LORA
  rf95.setModeIdle();

  delay(1000);

  setupLoRa();

  char radiopacket[20] = "XX:        ";
  radiopacket[0] = code[0];
  radiopacket[1] = code[1];

  memcpy( radiopacket + 4, id, 6 );
  char str_value[6];
  itoa( value, str_value, 10);
  memcpy( radiopacket + 11, str_value, 6);


  radiopacket[19] = 0;
  Serial.print("Sending |"); Serial.print(radiopacket); Serial.println("|");


  for ( int attempt = 0; attempt < MAX_RETRIES; attempt++ ) {

    Serial.print("Sending in attempt ");
    Serial.print(attempt + 1, DEC);
    Serial.print(" of ");
    Serial.println(MAX_RETRIES, DEC);

    delay(10);

    long int send_time = millis();
    rf95.send((uint8_t *)radiopacket, 20);


    Serial.println("Waiting for packet to complete...");
    delay(10);


    rf95.waitPacketSent();

    Serial.print("Time to send (ms) = ");
    Serial.println(millis() - send_time);


    // Now wait for a reply
    uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
    uint8_t len = sizeof(buf);
    Serial.println("Waiting for reply..."); delay(100);
    if (rf95.waitAvailableTimeout(4000))
    {
      // Should be a reply message for us now
      if (rf95.recv(buf, &len))
      {
        Serial.print("Got reply: ");
        Serial.println((char*)buf);
        Serial.print("RSSI: ");
        Serial.println(rf95.lastRssi(), DEC);
        break;
      }
      else
      {
        Serial.println("Receive failed");
      }
    }
    else
    {
      Serial.println("No reply, is there a listener around?");
    }
    delay(1000);


  }
#endif

}



void setup() {
  pinMode(OPT_LED_PIN, OUTPUT);

  while (!Serial);
  Serial.begin(9600);
  delay(100);

  Serial.println("Soil Moisture Detector");

  configureID();

  setupLoRa();

#ifdef INCLUDE_TEMPERATURE
  temp_sensors.begin();
#endif

  flashLED( 5, 250, 50);


  pinMode( MOISTURE_SENSOR_POWER_PIN, OUTPUT);
  digitalWrite(MOISTURE_SENSOR_POWER_PIN, LOW); // Start with power off
  pinMode( TEMPERATURE_SENSOR_POWER_PIN, OUTPUT);
  digitalWrite(TEMPERATURE_SENSOR_POWER_PIN, LOW); // Start with power off

}



void loop() {

  Serial.println("About to sleep");
  Serial.flush();
#ifdef INCLUDE_LORA
  rf95.sleep();
#endif

  // Can only sleep for 8 seconds but do it enough time to do it
  // for a total of five minutes
  for ( int i = 0; i < NUM_SLEEPS; i++ ) {
    LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF);
  }

  Serial.println("Waking up");
  Serial.flush();

  int moisture = readSoil();
  Serial.print("Soil Moisture is: ");
  Serial.println(moisture);

#ifdef INCLUDE_TEMPERATURE
  float temperature = readTemperature();
  Serial.print("Soil temperature is: ");
  Serial.println(temperature);
  int tempInt = int(temperature*100);
  
#endif

#ifdef INCLUDE_LORA

  sendData( moisture, "MS");

#ifdef INCLUDE_TEMPERATURE
  sendData( tempInt, "TP");
#endif

#endif
  flashLED( 5, 400, 100);

}

int readSoil() {
  int sum = 0;
  int i;

  digitalWrite(MOISTURE_SENSOR_POWER_PIN, HIGH);

  for ( i = 0; i < NUM_SAMPLES; i++ ) {
    delay(10);
    sum += analogRead( MOISTURE_SENSOR_PIN);
  }
  digitalWrite(MOISTURE_SENSOR_POWER_PIN, LOW);

  return int(sum / NUM_SAMPLES);

}

#ifdef INCLUDE_TEMPERATURE
float readTemperature() {
  float sum = 0.0;

  int i;



  digitalWrite(TEMPERATURE_SENSOR_POWER_PIN, HIGH);
  delay(1000);
  for ( i = 0; i < NUM_SAMPLES; i++ ) {
    temp_sensors.requestTemperatures();
    delay(10);
    sum += temp_sensors.getTempCByIndex(0);
  }

  digitalWrite(TEMPERATURE_SENSOR_POWER_PIN, LOW);
  return float(sum / NUM_SAMPLES);
}
#endif


void flashLED( int numflash, int on_time, int off_time ) {
  // Flash the builtin LED numflash times with on_time and off_time between each one
  int i;
  for ( i = 0; i < numflash; i++) {
    digitalWrite(OPT_LED_PIN, HIGH);
    delay(on_time);
    digitalWrite(OPT_LED_PIN, LOW);
    delay(off_time);
  }
}
