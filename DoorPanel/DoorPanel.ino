// Code for the room control panel mounted near the door (light switch)
#include <SPI.h>
#include "nRF24L01.h"
#include "RF24.h"
#include <SoftwareSerial.h>

// pin values
#define RELAY_ONE        0    // Lighting power supply
#define RELAY_TWO        0    // Speakers
#define RELAY_THREE      0    // Monitor 1
#define RELAY_FOUR       0    // Monitor 2
#define RELAY_FIVE       0    // Party light 1 (delayed start)
#define RELAY_SIX        0    // Party light 2 (delayed start)
#define RELAY_SEVEN      0    // Party light 3
#define RELAY_EIGHT      0    // Party light 4
#define ROTARY_CLICK     3
#define ROTARY_IN_A      5
#define ROTARY_IN_B      4
#define ROTARY_PINx      PIND
#define LCD_PIN          2
#define CE_PIN           7
#define CSN_PIN          8
#define DESK_MODULE_ADDR 0x60

// function declarations
int8_t readRotaryEncoder();
void setMode();
void setRelays(uint8_t input[]);
void initLCD();
void setLCDBacklight(uint8_t red, uint8_t green, uint8_t blue);
void clearLCD();
void setLCD();

// state declarations
uint8_t partyModeInit[8]   = {1,1,1,1,1,1,0,0};  // mode = 0
uint8_t partyModeActive[8] = {1,1,1,1,1,1,1,1};  // mode = 1
uint8_t normalLighting[8]  = {1,1,1,1,0,0,0,0};  // mode = 2
uint8_t chillMode[8]       = {1,1,1,1,1,1,0,0};  // mode = 3
uint8_t lightsOff[8]       = {0,1,1,1,0,0,0,0};  // mode = 4
uint8_t nightMode[8]       = {1,1,1,1,0,0,0,0};  // mode = 5
uint8_t everythingOff[8]   = {0,0,0,0,0,0,0,0};  // mode = 6

// variables for rotary encoder
static uint8_t enc_prev_pos = 0;
static uint8_t enc_flags    = 0;

// initialization and pipe for RF module
RF24 radio(CE_PIN, CSN_PIN);
const uint64_t pipe = 0xE8E8F0F0E1LL;
int data[2];

// initialize LCD display
SoftwareSerial lcd = SoftwareSerial(0,LCD_PIN); 

// mode variables
int curMode     = 4;
int activeMode  = 4;
int counter     = 0;
bool counting   = false;

void setup() {
  // set relay control pins as outputs
  pinMode(RELAY_ONE, OUTPUT);
  pinMode(RELAY_TWO, OUTPUT);
  pinMode(RELAY_THREE, OUTPUT);
  pinMode(RELAY_FOUR, OUTPUT);
  pinMode(RELAY_FIVE, OUTPUT);
  pinMode(RELAY_SIX, OUTPUT);
  pinMode(RELAY_SEVEN, OUTPUT);
  pinMode(RELAY_EIGHT, OUTPUT);
  
  // set rotary encoder pins (all pull-up)
  pinMode(ROTARY_CLICK, INPUT);
  pinMode(ROTARY_IN_A, INPUT);
  pinMode(ROTARY_IN_B, INPUT);
  digitalWrite(ROTARY_CLICK, HIGH);
  digitalWrite(ROTARY_IN_A, HIGH);
  digitalWrite(ROTARY_IN_B, HIGH);
  
  // initialize mode variables
  curMode = 4;
  activeMode = 4;
  
  // get initial reading from encoder pins
  if (digitalRead(ROTARY_IN_A) == LOW) {
    enc_prev_pos |= (1 << 0);
  }
  if (digitalRead(ROTARY_IN_B) == LOW) {
    enc_prev_pos |= (1 << 1);
  }
  setLCD();
  
  // begin LCD display and RF module
  Serial.begin(9600);
  initLCD();
  radio.begin();
  radio.openWritingPipe(pipe);
  
  delay(500);
}

void loop() {
  // read rotary encoder values
  int8_t rotaryVal = readRotaryEncoder();
  int8_t clickState = digitalRead(ROTARY_CLICK);
 
  if (clickState == LOW) {
    // selection made
    activeMode = curMode;
    setMode();
    
    // start counter for party mode initialization
    if (activeMode == 0) {
      counting = true; 
    } else {
      counting = false; 
    }
    setLCD();
  }
  
  // update screen selection
  if ((rotaryVal > 0) && (curMode < 6)) {
    curMode++;
    setLCD();
  }
  else if ((rotaryVal < 0) && (curMode > 0)) {
    curMode--;
    setLCD();
  }
  
  // start party mode and reset counter
  if (counting == true) {
    if (counter == 90) {
      activeMode = 1;
      counting = false;
      counter = 0;
      setMode();
    } else {
      counter++; 
    }
  }
}

int8_t readRotaryEncoder() {
  int8_t enc_action   = 0;  // 1 or -1 if moved, sign is direction
  uint8_t enc_cur_pos = 0;
  
  // read in the encoder state first
  if (bit_is_clear(ROTARY_PINx, ROTARY_IN_A)) {
    enc_cur_pos |= (1 << 0);
  }
  if (bit_is_clear(ROTARY_PINx, ROTARY_IN_B)) {
    enc_cur_pos |= (1 << 1);
  }
 
  // if any rotation at all
  if (enc_cur_pos != enc_prev_pos)
  {
    if (enc_prev_pos == 0x00)
    {
      // this is the first edge
      if (enc_cur_pos == 0x01) {
        enc_flags |= (1 << 0);
      }
      else if (enc_cur_pos == 0x02) {
        enc_flags |= (1 << 1);
      }
    }
 
    if (enc_cur_pos == 0x03)
    {
      // this is when the encoder is in the middle of a "step"
      enc_flags |= (1 << 4);
    }
    else if (enc_cur_pos == 0x00)
    {
      // this is the final edge
      if (enc_prev_pos == 0x02) {
        enc_flags |= (1 << 2);
      }
      else if (enc_prev_pos == 0x01) {
        enc_flags |= (1 << 3);
      }
 
      // check the first and last edge
      // or maybe one edge is missing, if missing then require the middle state
      // this will reject bounces and false movements
      if (bit_is_set(enc_flags, 0) && (bit_is_set(enc_flags, 2) || bit_is_set(enc_flags, 4))) {
        enc_action = 1;
      }
      else if (bit_is_set(enc_flags, 2) && (bit_is_set(enc_flags, 0) || bit_is_set(enc_flags, 4))) {
        enc_action = 1;
      }
      else if (bit_is_set(enc_flags, 1) && (bit_is_set(enc_flags, 3) || bit_is_set(enc_flags, 4))) {
        enc_action = -1;
      }
      else if (bit_is_set(enc_flags, 3) && (bit_is_set(enc_flags, 1) || bit_is_set(enc_flags, 4))) {
        enc_action = -1;
      }
 
      enc_flags = 0; // reset for next time
    }
  }
  enc_prev_pos = enc_cur_pos;
 
  return(enc_action);
}

void setMode() {
  sendToDesk();
  switch(activeMode) {
    case 0:
      setRelays(partyModeInit);
      break;
    case 1:
      setRelays(partyModeActive);
      break;
    case 2:
      setRelays(normalLighting);
      break;
    case 3:
      setRelays(chillMode);
      break;
    case 4:
      setRelays(lightsOff);
      break;
    case 5:
      setRelays(nightMode);
      break;
    case 6:
      setRelays(everythingOff);
      break;
    default:
      setRelays(lightsOff);
      break;
  }
}

void setRelays(uint8_t input[]) {
  // turn relays on or off based on values
  digitalWrite(RELAY_ONE, input[0]);
  digitalWrite(RELAY_TWO, input[1]);
  digitalWrite(RELAY_THREE, input[2]);
  digitalWrite(RELAY_FOUR, input[3]);
  digitalWrite(RELAY_FIVE, input[4]);
  digitalWrite(RELAY_SIX, input[5]);
  digitalWrite(RELAY_SEVEN, input[6]);
  digitalWrite(RELAY_EIGHT, input[7]);
}

void sendToDesk() {
  data[0] = activeMode;
  radio.write(data, sizeof(data));
}

void initLCD() {
  lcd.begin(9600);
  
  // set the size of the display
  lcd.write(0xFE);
  lcd.write(0xD1);
  lcd.write(16);    // 16 columns
  lcd.write(2);     // 2 rows
  delay(10); 

  // set the contrast
  lcd.write(0xFE);
  lcd.write(0x50);
  lcd.write(200);
  delay(10);   

  // set the brightness
  lcd.write(0xFE);
  lcd.write(0x99);
  lcd.write(255);
  delay(10);  

  // turn off cursors
  lcd.write(0xFE);
  lcd.write(0x4B);
  lcd.write(0xFE);
  lcd.write(0x54);
  delay(10);
  
  // splash screen
  lcd.print("Room Control by:");
  lcd.print("Connor Mason");
  delay(2000);

  // clear screen and return to top left corner
  clearLCD();
}

void setLCDBacklight(uint8_t red, uint8_t green, uint8_t blue) {
  // set backlight RGB values based on inputs
  lcd.write(0xFE);
  lcd.write(0xD0);
  lcd.write(red);
  lcd.write(green);
  lcd.write(blue);
  delay(10);
}

void clearLCD() {
  // clear the LCD and return cursor to top left corner
  lcd.write(0xFE);
  lcd.write(0x58);
  lcd.write(0xFE);
  lcd.write(0x48);
  delay(10);
}

void setLCD() {
  if ((curMode == 0) || (curMode == 1)) {
    setLCDBacklight(0xFF, 0x0, 0xFF);
    clearLCD();
    lcd.print("Party lighting  ");
    if ((activeMode == 0) || (activeMode == 1)) {
      lcd.print("  ACTIVE");
    }
  } else if (curMode == 2) {
    setLCDBacklight(0xFF, 0xFF, 0xFF);
    clearLCD();
    lcd.print("Normal lighting ");
    if (activeMode == 2) {
      lcd.print("  ACTIVE");
    }
  } else if (curMode == 3) {
    setLCDBacklight(0x0, 0x0, 0xFF);
    clearLCD();
    lcd.print("Chill lighting  ");
    if (activeMode == 3) {
      lcd.print("  ACTIVE");
    }
  } else if (curMode == 4) {
    setLCDBacklight(0x0, 0xFF, 0x0);
    clearLCD();
    lcd.print("Lights off      ");
    if (activeMode == 4) {
      lcd.print("  ACTIVE");
    }
  } else if (curMode == 5) {
    setLCDBacklight(0xFF, 0xFF, 0x0);
    clearLCD();
    lcd.print("Night mode      ");
    if (activeMode == 5) {
      lcd.print("  ACTIVE");
    }
  } else {
    setLCDBacklight(0xFF, 0x0, 0x0);
    clearLCD();
    lcd.print("Everything off  ");
    if (activeMode == 6) {
      lcd.print("  ACTIVE");
    }
  }
}
