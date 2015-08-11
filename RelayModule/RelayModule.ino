// Code for the 2 relay modules used in my room to control lights

/* 
 Arduino Outputs:
 Analog:
 0  --> unused
 1  --> unused
 2  --> unused
 3  --> unused
 4  --> unused
 5  --> unused
 Digital:
 0  --> unused
 1  --> unused
 2  --> unused
 3  --> unused
 4  --> Relay 1
 5  --> Relay 2
 6  --> Relay 3
 7  --> Relay 4
 8  --> Activity LED (if connected)
 9  --> RF CSN
 10 --> RF CE
 11 --> RF MOSI
 12 --> RF MISO
 13 --> RF SCK
 */

#include <SPI.h>
#include "nRF24L01.h"
#include "RF24.h"

#define CSN_PIN 9
#define CE_PIN 10
#define RELAY_ONE 4
#define RELAY_TWO 5
#define RELAY_THREE 6
#define RELAY_FOUR 7
#define ACTIVITY_LED 8

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

  pinMode(ACTIVITY_LED, OUTPUT);

  digitalWrite(ACTIVITY_LED, HIGH);
  delay(100);
  digitalWrite(ACTIVITY_LED, LOW); 
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
      digitalWrite(ACTIVITY_LED, HIGH);
      delay(100);
      digitalWrite(ACTIVITY_LED, LOW);
    }
  }
  setRelays();
}


