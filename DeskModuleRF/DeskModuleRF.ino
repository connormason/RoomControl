// Code for the RF receiver Arduino alongside the desk control module

/* 
Arduino Outputs:
  Analog:
    0  --> Comm to DeskModule 1
    1  --> Comm to DeskModule 2
    2  --> Comm to DeskModule 4
    3  --> Comm to DeskModule 8
    4  --> Comm to DeskModule 16
    5  --> Comm to DeskModule 32
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

Lighting Modes:
   0 --> party mode initialization
   1 --> party mode active
   2 --> normal lighting
   3 --> chill mode
   4 --> lights off
   5 --> night mode
   6 --> everything off
*/

#include <SPI.h>
#include "nRF24L01.h"
#include "RF24.h"

#define RELAY_ONE 0
#define RELAY_TWO 1
#define RELAY_THREE 2
#define RELAY_FOUR 3
#define CSN_PIN 9
#define CE_PIN 10
#define COMM_ONE 14
#define COMM_TWO 15
#define COMM_THREE 16
#define COMM_FOUR 17
#define COMM_FIVE 18
#define COMM_SIX 19

const uint64_t pipe = 0xE8E8F0F0E1LL;
int data[2];
int activeMode = 4;

RF24 radio(CE_PIN, CSN_PIN);

void setBinaryComm() {
  int c, k;
  for (c = 5; c >= 0; c--) {
    k = activeMode >> c;
    switch(c) {
      case 5:
        digitalWrite(COMM_SIX, k & 1);
        break;
      case 4:
        digitalWrite(COMM_FIVE, k & 1);
        break;
      case 3:
        digitalWrite(COMM_FOUR, k & 1);
        break;
      case 2:
        digitalWrite(COMM_THREE, k & 1);
        break;
      case 1:
        digitalWrite(COMM_TWO, k & 1);
        break;
      case 0:
        digitalWrite(COMM_ONE, k & 1);
        break;
    }
  }
}

void setup() {
  // set pins for communication to DeskModule Arduino)
  pinMode(COMM_ONE, OUTPUT);
  pinMode(COMM_TWO, OUTPUT);
  pinMode(COMM_THREE, OUTPUT);
  pinMode(COMM_FOUR, OUTPUT);
  pinMode(COMM_FIVE, OUTPUT);
  pinMode(COMM_SIX, OUTPUT);
  pinMode(8, OUTPUT);
  
  // start RF SPI
  radio.begin();
  radio.openReadingPipe(1,pipe);
  radio.startListening();
  
  digitalWrite(8, HIGH);
  delay(100);
  digitalWrite(8, LOW);
}

void loop() {
  // get data from RF transmission and set output
  while (radio.available()) {
    radio.read(data, sizeof(data));
    if (data[0] != activeMode) {
      activeMode = data[0];
      digitalWrite(8, HIGH);
      delay(100);
      digitalWrite(8, LOW);
    }
  }
  setBinaryComm();
}
