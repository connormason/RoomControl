// Code for the control module on desk (also controls LEDGrid)
// Spectrum analyzer code by: Stabitha URL: https://github.com/Stabitha/Spectrum_Shield_Driver

/* 
Arduino Outputs:
  Analog:
    0  --> Spectrum Analyzer left channel signal
    1  --> Spectrum Analyzer right channel signal
    2  --> Comm from DeskModuleRF 1
    3  --> Comm from DeskModuleRF 2
    4  --> I2C SDA (for LCD)
    5  --> I2C SCL (for LCD)
  Digital:
    0  --> unused (USB RX)
    1  --> unused (USB TX)
    2  --> Comm from DeskModuleRF 4
    3  --> TLC5940 GSCLK
    4  --> Spectrum Analyzer strobe
    5  --> Spectrum Analyzer reset
    6  --> Comm from DeskModuleRF 8
    7  --> TLC5940 SIN
    8  --> Comm from DeskModuleRF 16
    9  --> TLC5940 XLAT
    10 --> TLC5940 BLANK
    11 --> TLC5940 SIN
    12 --> Comm from DeskModuleRF 32
    13 --> TLC5940 SCLK

Lighting Modes:
   0 --> party mode initialization
   1 --> party mode active
   2 --> normal lighting
   3 --> chill mode
   4 --> lights off
   5 --> night mode
   6 --> everything off
*/

#include <Wire.h>
#include "rgb_lcd.h"
#include "Tlc5940.h"

#define COMM_ONE 16
#define COMM_TWO 17
#define COMM_THREE 6
#define COMM_FOUR 8
#define COMM_FIVE 11
#define COMM_SIX 12


// variable for mode sent from control panel
int activeMode = 4;

// create custom characters for LCD
byte level0[8] = { 0b00000, 0b00000, 0b00000, 0b00000, 0b00000, 0b00000, 0b00000, 0b11111};
byte level1[8] = { 0b00000, 0b00000, 0b00000, 0b00000, 0b00000, 0b00000, 0b11111, 0b11111};
byte level2[8] = { 0b00000, 0b00000, 0b00000, 0b00000, 0b00000, 0b11111, 0b11111, 0b11111};
byte level3[8] = { 0b00000, 0b00000, 0b00000, 0b00000, 0b11111, 0b11111, 0b11111, 0b11111};
byte level4[8] = { 0b00000, 0b00000, 0b00000, 0b11111, 0b11111, 0b11111, 0b11111, 0b11111};
byte level5[8] = { 0b00000, 0b00000, 0b11111, 0b11111, 0b11111, 0b11111, 0b11111, 0b11111};
byte level6[8] = { 0b00000, 0b11111, 0b11111, 0b11111, 0b11111, 0b11111, 0b11111, 0b11111};
byte level7[8] = { 0b11111, 0b11111, 0b11111, 0b11111, 0b11111, 0b11111, 0b11111, 0b11111};

// spectrum shield pins
int strobe = 4; // strobe pins on digital 4
int res    = 5; // reset pins on digital 5

// smoothing constants
static const byte smoothP   = 1;                // Number of samples to compute rolling average over (empirically set)
static const boolean _INIT_ = true;             // increasing 'smoothP' makes bars jutter less, but also increases <S-<S>> = c 
static const byte tmpSqRt   = 5;                // sqroot(smoothP/2) = sqroot(32) = 5.65 = 5;

// spectrum smoothing and output variables
long checkL[7]        = { 0, 0, 0, 0, 0, 0, 0};
long checkR[7]        = { 0, 0, 0, 0, 0, 0, 0};             
int left[7];                                     // left analog data
int right[7];                                    // right analog data
int averageR[7]       = { 0, 0, 0, 0, 0, 0, 0};  // 'smoothP' running average of raw data 
int averageL[7]       = { 0, 0, 0, 0, 0, 0, 0};
int _zeroBSLR[7]      = { 0, 0, 0, 0, 0, 0, 0};  // bands with zero volume input
int _zeroBSLL[7]      = { 0, 0, 0, 0, 0, 0, 0};
uint32_t stdDevL[7]   = { 0, 0, 0, 0, 0, 0, 0};  // standard deviation of band inputs for change-point detection
uint32_t stdDevR[7]   = { 0, 0, 0, 0, 0, 0, 0};
uint32_t varianceL[7] = { 0, 0, 0, 0, 0, 0, 0};  // unsigned 32-bit integer to store (1023)*(1023) maximum
uint32_t varianceR[7] = { 0, 0, 0, 0, 0, 0, 0};
boolean chgPtL[7]     = { false, false, false, false, false, false, false};
boolean chgPtR[7]     = { false, false, false, false, false, false, false};
long cumSumSqL[7]     = { 0, 0, 0, 0, 0, 0, 0};
long cumSumSqR[7]     = { 0, 0, 0, 0, 0, 0, 0};
long tmpD_kL          = 0;
long tmpD_kR          = 0;
long D_kL[7]          = { 0, 0, 0, 0, 0, 0, 0};
long D_kR[7]          = { 0, 0, 0, 0, 0, 0, 0};
uint32_t initValueL   = 0;
uint32_t initValueR   = 0;
uint32_t tmpVarL      = 0;
uint32_t tmpVarR      = 0;
int prvL[7]           = { 0, 0, 0, 0, 0, 0, 0};
int prvR[7]           = { 0, 0, 0, 0, 0, 0, 0};
int outputL[7]        = { 0, 0, 0, 0, 0, 0, 0};
int outputR[7]        = { 0, 0, 0, 0, 0, 0, 0};
int tmpAvgL           = 0;
int tmpAvgR           = 0;
int band;                                        // counting variable for going through channels
byte cnt              = 1;
//rgb_lcd lcd;

inline void reduce(int &anInt, int aAmount, int aLimit, int aMin = 0) {
  int r = ((aAmount > aLimit) ? (anInt-aLimit) : (anInt-aAmount));
  if (r < aMin) {
    anInt = aMin;
  } else {
    anInt = r;
  }
}

inline void increase(int &anInt, int aAmount, int aLimit, int aMax = 1023) {
  int r = ((aAmount > aLimit) ? (anInt+aLimit) : (anInt+aAmount));
  if (r > aMax) {
    anInt = aMax;
  } else {
    anInt = r;
  }
}

inline byte increaseByte(byte aByte, byte aAmount, byte aLimit, byte aMax) {
  int r = ((aAmount > aLimit) ? (aByte+aLimit) : (aByte+aAmount));
  if (r > aMax) {
    return aMax;
  } else {
    return r;
  }
}

inline byte reduceByte(byte aByte, byte aAmount, byte aLimit, byte aMin) {
  int r = ((aAmount > aLimit) ? (aByte-aLimit) : (aByte-aAmount));
  if (r < aMin) {
    return aMin;
  } else {
    return r;
  }
}

uint32_t findSqRoot(uint32_t aVariance) {
  uint32_t result = 0;              
  uint32_t var = aVariance;
  uint32_t check_bit = 1;             
  check_bit <<= 30;
  while (check_bit > var) {         
    check_bit >>= 2;                       
  }
  while (check_bit != 0) {              
    if (var >= (result + check_bit)) {   
      var -= (result + check_bit);       
      result = (result>>1) + check_bit;  
    } else {                             
      result >>= 1;                     
    }
    check_bit >>= 2;                      
  }                               
  return result;                              
}

void resetCS(int aBand) {
    D_kL[aBand] = 0;
    D_kR[aBand] = 0;
    cumSumSqL[aBand] = 0;
    cumSumSqR[aBand] = 0;
}

void readMSGEQ7() {
  // Function to read 7 band equalizers
  digitalWrite(res, HIGH);
  digitalWrite(res, LOW);
  for(band=0; band <7; band++) {
    digitalWrite(strobe,LOW); // strobe pin on the shield - kicks the IC up to the next band
    delayMicroseconds(30); //
    left[band] = analogRead(0); // store left band reading
    right[band] = analogRead(1); // ... and the right
    digitalWrite(strobe,HIGH);
  }
}

void shapeMSGEQ7(int _k, boolean initialPass = false) { // Use Welford's algorithm, pass the step 'k' and whether we are on the intiial pass.
  readMSGEQ7();                       // read all 7 bands for left and right channels
  for (band = 0; band < 7; band++) {  // and for each band compute the running average and variance
    tmpAvgL = averageL[band];         // Store old average estimate M_k-1 from previous pass through
    tmpAvgR = averageR[band];
    if (!initialPass) {
      reduce(left[band], _zeroBSLL[band], 1023, 0);
      reduce(right[band], _zeroBSLR[band], 1023, 0);
    }  
    averageL[band] = tmpAvgL + ((left[band] - averageL[band])/_k);  // M_k = M_k-1 + (x_k - M_k-1)/k
    averageR[band] = tmpAvgR + ((right[band] - averageR[band])/_k); // Moving '_k' average of left and right channels
    if (!initialPass) {                     // If this is NOT the initial pass, subtract out the zero-point baseline and
      if (_k > 1) {                         // compute the variance if _k > 1 as well
        tmpVarL = varianceL[band];
        tmpVarR = varianceR[band];
        varianceL[band] = ((tmpVarL + ((left[band]-averageL[band])*(left[band]-tmpAvgL)))/(_k-1));
        varianceR[band] = ((tmpVarR + ((right[band]-averageR[band])*(right[band]-tmpAvgR)))/(_k-1));
        stdDevL[band] = findSqRoot(varianceL[band]);
        stdDevR[band] = findSqRoot(varianceR[band]);
        cumSumSqL[band] += ((prvL[band]-tmpAvgL)*(prvL[band]-tmpAvgL));       // NOTE: cumSumSq is reset to zero in resetCS();
        cumSumSqR[band] += ((prvR[band]-tmpAvgR)*(prvR[band]-tmpAvgR));       
        prvL[band] = left[band];
        prvR[band] = right[band];      
        tmpD_kL = D_kL[band];
        tmpD_kR = D_kR[band];
        D_kL[band] = (long)((_k*cumSumSqL[band]*tmpD_kL + ((left[band]-averageL[band])*(left[band]-averageL[band])))/(_k*cumSumSqL[band]));
        D_kR[band] = (long)((_k*cumSumSqR[band]*tmpD_kR + ((right[band]-averageR[band])*(right[band]-averageR[band])))/(_k*cumSumSqR[band]));
        tmpD_kL = (tmpSqRt*D_kL[band]);
        tmpD_kR = (tmpSqRt*D_kR[band]);
        tmpD_kL = (D_kL[band] > 0) ? tmpD_kL : -1*(tmpD_kL);
        tmpD_kR = (D_kR[band] > 0) ? tmpD_kR : -1*(tmpD_kR);
        if (tmpD_kL > 163) {                 // signal chgpt detected, using heuristic value, slightly larger than Inclan & Tiao
          chgPtL[band] = true;
        } else {
          chgPtL[band] = false;
        }
        if (tmpD_kR > 163) {
          chgPtR[band] = true;
        } else {
          chgPtR[band] = false;
        }
      checkL[band] = tmpD_kL;
      checkR[band] = tmpD_kR;
      }
      tmpAvgL = averageL[band];              // re-initialize tmpAvgL/R with current baseline values to correct
      tmpAvgR = averageR[band];   
      left[band] = tmpAvgL;
      right[band] = tmpAvgR;
    }
  }
}   

inline void limitLeft(int aLimit) {
  for (band = 0; band < 7; band++) {
    byte newS = 0;
    byte oldS = 0;
    byte diff = 0;
    newS = outputR[band];
    oldS = prvL[band];
    diff = ((newS < oldS) ? (oldS - newS) : (newS - oldS));
    prvL[band] = ((newS < oldS) ? (reduceByte(oldS, diff, aLimit, 0)) : (increaseByte(oldS, diff, aLimit, 255)));
  }
}

inline void limitRight(int aLimit) {
  for (band = 0; band < 7; band++) {
    byte newS = 0;
    byte oldS = 0;
    byte diff = 0;
    newS = outputR[band];
    oldS = prvR[band];
    diff = ((newS < oldS) ? (oldS - newS) : (newS - oldS));
    prvR[band] = ((newS < oldS) ? (reduceByte(oldS, diff, aLimit, 0)) : (increaseByte(oldS, diff, aLimit, 255)));
  }
}
    
void setup() {
  pinMode(res, OUTPUT);       // reset
  pinMode(strobe, OUTPUT);    // strobe
  digitalWrite(res,LOW);      // reset low
  digitalWrite(strobe,HIGH);  //pin 5 is RESET on the shield
  for (int i = 1; i < (smoothP+1); i++) {
    shapeMSGEQ7(i, _INIT_);   // grab band-specific baseline adjustments (assumes no audio on initialization)
  }
  for (band = 0; band < 7; band++) {
    _zeroBSLR[band] = averageR[band];
    _zeroBSLL[band] = averageL[band];
  }
  
  // LEDGrid multiplexer chip, RF initialization, and relay pinMode
  Tlc.init();
  Serial.begin(9600);
  
  // setup LCD and custom characters
//  lcd.begin(16, 2);
//  lcd.clear();
//  lcd.createChar(0,level0);
//  lcd.createChar(1,level1);
//  lcd.createChar(2,level2);
//  lcd.createChar(3,level3);
//  lcd.createChar(4,level4);
//  lcd.createChar(5,level5);
//  lcd.createChar(6,level6);
//  lcd.createChar(7,level7);
//  lcd.setCursor(0,1);
//  lcd.print("Left");
//  lcd.setCursor(11,1);
//  lcd.print("Right");
}

void loop() {
  shapeMSGEQ7(smoothP);
  if (cnt > 64) {                          // <----- change this value for different sample sizes
    for (band = 0; band < 7; band++) {
      resetCS(band);
    }
  }
  cnt++;
  
//  for(band = 0; band < 7; band++) {
//    lcd.setCursor(band,0);
//    if (left[band]>=895) { lcd.write((uint8_t)7); } else
//    if (left[band]>=767) { lcd.write((uint8_t)6); } else
//    if (left[band]>=639) { lcd.write((uint8_t)5); } else
//    if (left[band]>=511) { lcd.write((uint8_t)4); } else
//    if (left[band]>=383) { lcd.write((uint8_t)3); } else
//    if (left[band]>=255) { lcd.write((uint8_t)2); } else
//    if (left[band]>=127) { lcd.write((uint8_t)1); } else
//    if (left[band]>=0) { lcd.write((uint8_t)0); }
//  }
//  // display values of right channel on LCD
//  for(band = 0; band < 7; band++) {
//    lcd.setCursor(band+9,0);
//    if (right[band]>=895) { lcd.write((uint8_t)7); } else
//    if (right[band]>=767) { lcd.write((uint8_t)6); } else
//    if (right[band]>=639) { lcd.write((uint8_t)5); } else
//    if (right[band]>=511) { lcd.write((uint8_t)4); } else
//    if (right[band]>=383) { lcd.write((uint8_t)3); } else
//    if (right[band]>=255) { lcd.write((uint8_t)2); } else
//    if (right[band]>=127) { lcd.write((uint8_t)1); } else
//    if (right[band]>=0) { lcd.write((uint8_t)0); }
//  }

  activeMode = digitalRead(COMM_ONE) + (2 * digitalRead(COMM_TWO)) + (4 * digitalRead(COMM_THREE)) + (8 * digitalRead(COMM_FOUR)) + (16 *digitalRead(COMM_FIVE)) + (32 * digitalRead(COMM_SIX));
  Serial.println(activeMode);
  
  Tlc.clear();
  for (int offset = 0; offset < 3; offset++) {
    for (int i = 0; i < 24; i+=3) {
      Tlc.set(i + offset, 3500);
    }
    Tlc.update();
  }
  
//  lcd.setCursor(0,1);
//  switch(activeMode) {
//    case 0:
//      lcd.print("Party mode");
//      break;
//    case 1:
//      lcd.print("Party mode");
//      break;
//    case 2:
//      lcd.print("Normal lighting");
//      break;
//    case 3:
//      lcd.print("Chill mode");
//      break;
//    case 4:
//      lcd.print("Lights off");
//      break;
//    case 5: 
//      lcd.print("Night mode");
//      break;
//    case 6:
//      lcd.print("Everything off");
//      break;
//  }
  
  delay(1);
}
