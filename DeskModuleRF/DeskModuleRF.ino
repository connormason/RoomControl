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
    0  --> Relay 1 (LEDGrid power supply)
    1  --> Relay 2 (Rope light)
    2  --> Relay 3 (Computer Monitor and speakers)
    3  --> Relay 4 (Party lights)
    4  --> Relay 5
    5  --> Relay 6
    6  --> Relay 7
    7  --> Relay 8
    8  --> Activity LED
    9  --> RF CSN
    10 --> RF CE
    11 --> RF MOSI
    12 --> RF MISO
    13 --> RF SCK

Lighting Modes:
   0 --> rage mode
   1 --> party mode
   2 --> normal lighting
   3 --> study mode
   4 --> chill mode
   5 --> lights off
   6 --> night mode
   7 --> everything off
*/

#include <SPI.h>
#include "nRF24L01.h"
#include "RF24.h"

#define RELAY_ONE 0
#define RELAY_TWO 1
#define RELAY_THREE 2
#define RELAY_FOUR 3
#define RELAY_FIVE 4
#define RELAY_SIX 5
#define RELAY_SEVEN 6
#define RELAY_EIGHT 7
#define CSN_PIN 9
#define CE_PIN 10
#define COMM_ONE 14
#define COMM_TWO 15
#define COMM_THREE 16
#define COMM_FOUR 17
#define COMM_FIVE 18
#define COMM_SIX 19
#define ACTIVITY_LED 8

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

void setRelays() {
  switch(activeMode) {
    case 0:
      digitalWrite(RELAY_ONE, HIGH);
      digitalWrite(RELAY_TWO, LOW);
      digitalWrite(RELAY_THREE, HIGH);
      digitalWrite(RELAY_FOUR, HIGH);
      break;
    case 1:
      digitalWrite(RELAY_ONE, HIGH);
      digitalWrite(RELAY_TWO, LOW);
      digitalWrite(RELAY_THREE, HIGH);
      digitalWrite(RELAY_FOUR, HIGH);
      break;
    case 2:
      digitalWrite(RELAY_ONE, HIGH);
      digitalWrite(RELAY_TWO, HIGH);
      digitalWrite(RELAY_THREE, HIGH);
      digitalWrite(RELAY_FOUR, LOW);
      break;
    case 3:
      digitalWrite(RELAY_ONE, HIGH);
      digitalWrite(RELAY_TWO, HIGH);
      digitalWrite(RELAY_THREE, HIGH);
      digitalWrite(RELAY_FOUR, LOW);
      break;
    case 4:
      digitalWrite(RELAY_ONE, HIGH);
      digitalWrite(RELAY_TWO, HIGH);
      digitalWrite(RELAY_THREE, HIGH);
      digitalWrite(RELAY_FOUR, LOW);
      break;
    case 5:
      digitalWrite(RELAY_ONE, LOW);
      digitalWrite(RELAY_TWO, LOW);
      digitalWrite(RELAY_THREE, HIGH);
      digitalWrite(RELAY_FOUR, LOW);
      break;
    case 6: 
      digitalWrite(RELAY_ONE, HIGH);
      digitalWrite(RELAY_TWO, LOW);
      digitalWrite(RELAY_THREE, LOW);
      digitalWrite(RELAY_FOUR, LOW);
      break;
    case 7:
      digitalWrite(RELAY_ONE, LOW);
      digitalWrite(RELAY_TWO, LOW);
      digitalWrite(RELAY_THREE, LOW);
      digitalWrite(RELAY_FOUR, LOW);
      break;
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
  pinMode(RELAY_ONE, OUTPUT);
  pinMode(RELAY_TWO, OUTPUT);
  pinMode(RELAY_THREE, OUTPUT);
  pinMode(RELAY_FOUR, OUTPUT);
  pinMode(RELAY_FIVE, OUTPUT);
  pinMode(RELAY_SIX, OUTPUT);
  pinMode(RELAY_SEVEN, OUTPUT);
  pinMode(RELAY_EIGHT, OUTPUT);
  pinMode(ACTIVITY_LED, OUTPUT);
  
  // start RF SPI
  radio.begin();
  radio.openReadingPipe(1,pipe);
  radio.startListening();
  
  digitalWrite(ACTIVITY_LED, HIGH);
  delay(100);
  digitalWrite(ACTIVITY_LED, LOW);
}

void loop() {
  // get data from RF transmission and set output
  while (radio.available()) {
    radio.read(data, sizeof(data));
    if (data[0] != activeMode) {
      activeMode = data[0];
      digitalWrite(ACTIVITY_LED, HIGH);
      delay(100);
      digitalWrite(ACTIVITY_LED, LOW);
    }
  }
  setBinaryComm();
  setRelays();
}
