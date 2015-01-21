// Code for the 2 relay modules used in my room to control lights

/* 
 Arduino Outputs:
 Analog:
 0  --> unused
 1  --> Relay 1
 2  --> Relay 2
 3  --> Relay 3
 4  --> Relay 4
 5  --> unused
 Digital:
 0  --> unused
 1  --> unused
 2  --> unused
 3  --> unused
 4  --> unused
 5  --> unused
 6  --> unused
 7  --> unused
 8  --> unused
 9  --> RF CSN
 10 --> RF CE
 11 --> RF MOSI
 12 --> RF MISO
 13 --> RF SCK
 */

/*
  Relay attachments:
    Module 1:
      1: LEDGrid Power Supply
      2: Monitor 1
      3: Monitor 2
      4: Rope light
    Module 2:
      1: Desk lamp
      2: Party light 1
      3: Party light 2
      4: Speakers
*/

#include <SPI.h>
#include "nRF24L01.h"
#include "RF24.h"

#define CE_PIN 10
#define CSN_PIN 9
#define RELAY_ONE 18
#define RELAY_TWO 17
#define RELAY_THREE 16
#define RELAY_FOUR 15

#define RELAY_MODULE 2

const uint64_t pipe = 0xE8E8F0F0E1LL;

int data[3];
int curRelays = 1;

RF24 radio(CE_PIN, CSN_PIN);

void setRelays() {
  digitalWrite(RELAY_ONE, (curRelays & 8) / 8);
  digitalWrite(RELAY_TWO, (curRelays & 4) / 4);
  digitalWrite(RELAY_THREE, (curRelays & 2) / 2);
  digitalWrite(RELAY_FOUR, (curRelays & 1) / 1);
}

void setup() {
  // set relay pins
  pinMode(RELAY_ONE, OUTPUT);
  pinMode(RELAY_TWO, OUTPUT);
  pinMode(RELAY_THREE, OUTPUT);
  pinMode(RELAY_FOUR, OUTPUT);

  pinMode(8, OUTPUT);
  Serial.begin(9600);

  digitalWrite(8, HIGH);
  delay(100);
  digitalWrite(8, LOW); 
  delay(100);

  radio.begin();
  radio.openReadingPipe(1,pipe);
  radio.startListening();

  setRelays();
}

void loop() {
  while (radio.available()) {
    radio.read(data, sizeof(data));
    if (data[1] == RELAY_MODULE) {
      curRelays = data[2];
      digitalWrite(8, HIGH);
      delay(100);
      digitalWrite(8, LOW);
      
      Serial.println(data[2]);
      setRelays();
    }
  }
}


