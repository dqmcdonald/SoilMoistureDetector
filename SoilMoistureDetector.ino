/*
   Soil moisture detector. Uses Sparkfun soil moisture
   detector to masure soil moisture level and report
   the result via LoRa to a Gateway.

   https://www.sparkfun.com/products/13322

   Intended to work on 3.3V Arduino Mini Pro

  Pin Assignments:
     3 -> LoRa DOI0
     5 -> Sensor Power
     9 -> LoRa Reset
    10 -> LoRa Clock Select (NSS)
    11 -> LoRa MISO
    12 -> LoRa MOSI
    13 -> LoRa SCK

  Quentin McDonald
  May 2020
*/

#include "LowPower.h"
#include <SPI.h>
#include <RH_RF95.h>
#include <stdlib.h>

#include <EEPROM.h>

#define NUM_SLEEPS 38
#define POWER_PIN 5
#define SENSOR_PIN A0
#define NUM_SAMPLES 10   // Average 10 samples
#define NUM_SLEEPS 38    // 38 sleeps of 8 seconds each gives us about
// a reading each five minutes.



#define RFM95_CS 10  // Clock select should be on 10
#define RFM95_RST 9  // Reset on 9
#define RFM95_INT 3  // Interrupt on 3

// Operating at 433 Mhz
#define RF95_FREQ 433.0

// Singleton instance of the radio driver
RH_RF95 rf95(RFM95_CS, RFM95_INT);

const int ID_LEN = 6;
// The station ID
char id[ID_LEN];

const int MAX_RETRIES = 3; // Try to send three times:

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
}

void setup() {
  while (!Serial);
  Serial.begin(9600);
  delay(100);

  Serial.println("Soil Moisture Detector");

  configureID();

  setupLoRa();

  pinMode( POWER_PIN, OUTPUT);
  digitalWrite(POWER_PIN, LOW); // Start with power off

}



void loop() {

  Serial.println("About to sleep");
  Serial.flush();

  rf95.sleep();

  // Can only sleep for 8 seconds but do it enough time to do it
  // for a total of five minutes
  for ( int i = 0; i < NUM_SLEEPS; i++ ) {
    LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF);
  }

  int value = readSoil();

  rf95.setModeIdle();

  delay(1000);

  setupLoRa();

  char radiopacket[20] = "SM:        ";

  memcpy( radiopacket + 4, id, 6 );
  char str_value[6];
  itoa( value, str_value, 10);
  memcpy( radiopacket + 11, str_value, 6);

  Serial.print("Sending "); Serial.println(radiopacket);

  radiopacket[19] = 0;

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

}

int readSoil() {
  int sum = 0;
  int i;

  digitalWrite(POWER_PIN, HIGH);

  for ( i = 0; i < NUM_SAMPLES; i++ ) {
    delay(10);
    sum += analogRead( SENSOR_PIN);
  }
  digitalWrite(POWER_PIN, LOW);

  return int(sum / NUM_SAMPLES);

}
