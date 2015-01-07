#include <SPI.h>
#include "nRF24L01.h"
#include "RF24.h"

#define CE_PIN 9
#define CSN_PIN 8

const uint64_t pipe = 0xE8E8F0F0E1LL;

int data[2];
int activeMode = 4;

RF24 radio(CE_PIN, CSN_PIN);

void setup() {
  pinMode(6, OUTPUT);
  digitalWrite(13, LOW); 
  Serial.begin(9600);
  delay(1000);
  Serial.println("Nrf24L01 Receiver Starting");
  radio.begin();
  radio.openReadingPipe(1,pipe);
  radio.startListening();
}

void loop() {
  while (radio.available()) {
    radio.read(data, sizeof(data));
    if (data[0] != activeMode) {
      activeMode = data[0];
      delay(100);
    }
    
    if (activeMode == 6) {
      digitalWrite(6, HIGH);
      delay(100);
      digitalWrite(6, LOW); 
      delay(100);
    }
  }
}
