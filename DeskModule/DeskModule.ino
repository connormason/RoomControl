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
   7 --> study mode
*/

#include <Wire.h>
#include "Tlc5940.h"

#define COMM_ONE 16
#define COMM_TWO 17
#define COMM_THREE 2
#define COMM_FOUR 6
#define COMM_FIVE 8
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

bool roomEntered = false;

// variables for music active mode
int counter1 = 0;
int counter2 = 25;
int counter3 = 50;
int red1 = 255;
int red2 = 230;
int red3 = 205;
int green1 = 0;
int green2 = 25;
int green3 = 50;
int blue1 = 0;
int blue2 = 0;
int blue3 = 255;

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

void halogen() {
  Tlc.clear();
  int redVal = 4095;
  int greenVal = 2400;
  int blueVal = 2400;
  int time = 125;
  
  if (roomEntered == false) {
    Tlc.set(0, redVal);
    Tlc.set(1, greenVal);
    Tlc.set(2, blueVal);
    Tlc.update();
    delay(time);
    
    Tlc.set(3, redVal);
    Tlc.set(4, greenVal);
    Tlc.set(5, blueVal);
    Tlc.update();
    delay(time);
    
    Tlc.set(6, redVal);
    Tlc.set(7, greenVal);
    Tlc.set(8, blueVal);
    Tlc.update();
    delay(time);
    
    Tlc.set(9, redVal);
    Tlc.set(10, greenVal);
    Tlc.set(11, blueVal);
    Tlc.update();
    delay(time);
    
    Tlc.set(12, redVal);
    Tlc.set(13, greenVal);
    Tlc.set(14, blueVal);
    Tlc.update();
    delay(time);
    
    Tlc.set(18, redVal);
    Tlc.set(19, greenVal);
    Tlc.set(20, blueVal);
    Tlc.update();
    delay(time);
    
    Tlc.set(15, redVal);
    Tlc.set(16, greenVal);
    Tlc.set(17, blueVal);
    Tlc.set(21, redVal);
    Tlc.set(22, greenVal);
    Tlc.set(23, blueVal);
    Tlc.update();
    delay(time);
    roomEntered = true;
  } else {
    Tlc.set(0, redVal);
    Tlc.set(1, greenVal);
    Tlc.set(2, blueVal);
    Tlc.set(3, redVal);
    Tlc.set(4, greenVal);
    Tlc.set(5, blueVal);
    Tlc.set(6, redVal);
    Tlc.set(7, greenVal);
    Tlc.set(8, blueVal);
    Tlc.set(9, redVal);
    Tlc.set(10, greenVal);
    Tlc.set(11, blueVal);
    Tlc.set(12, redVal);
    Tlc.set(13, greenVal);
    Tlc.set(14, blueVal);
    Tlc.set(15, redVal);
    Tlc.set(16, greenVal);
    Tlc.set(17, blueVal);
    Tlc.set(18, redVal);
    Tlc.set(19, greenVal);
    Tlc.set(20, blueVal);
    Tlc.set(21, redVal);
    Tlc.set(22, greenVal);
    Tlc.set(23, blueVal);
    Tlc.update();
  }
}

void chillMode() {
  Tlc.clear();
  Tlc.set(0, 1000);
  Tlc.set(2, 2000);
  Tlc.set(5, 2200);
  Tlc.set(6, 2000);
  Tlc.set(8, 1000);
  Tlc.set(11, 2200);
  Tlc.set(12, 1000);
  Tlc.set(14, 2000);
  Tlc.set(17, 2200);
  Tlc.set(18, 2000);
  Tlc.set(20, 1000);
  Tlc.set(23, 2200);
  Tlc.update();
}

void nightMode() {
  Tlc.clear();
  Tlc.set(0, 500);
  Tlc.set(3, 500);
  Tlc.set(6, 1000);
  Tlc.set(9, 500);
  Tlc.set(12, 500);
  Tlc.set(15, 500);
  Tlc.set(18, 1000);
  Tlc.set(21, 500);
  Tlc.update();
}

void studyMode() {
  Tlc.clear();
  Tlc.set(0, 4095);
  Tlc.set(1, 4095);
  Tlc.set(2, 4095);
  Tlc.set(3, 4095);
  Tlc.set(4, 4095);
  Tlc.set(5, 4095);
  Tlc.set(6, 0);
  Tlc.set(7, 0);
  Tlc.set(8, 4095);
  Tlc.set(9, 4095);
  Tlc.set(10, 4095);
  Tlc.set(11, 4095);
  Tlc.set(12, 4095);
  Tlc.set(13, 4095);
  Tlc.set(14, 4095);
  Tlc.set(15, 4095);
  Tlc.set(16, 4095);
  Tlc.set(17, 4095);
  Tlc.set(18, 0);
  Tlc.set(19, 0);
  Tlc.set(20, 4095);
  Tlc.set(21, 4095);
  Tlc.set(22, 4095);
  Tlc.set(23, 4095);
  Tlc.update();
}

void relaxMode() {
  Tlc.clear();
  Tlc.set(0, 500);
  Tlc.set(2, 1000);
  Tlc.set(5, 300);
  Tlc.set(6, 500);
  Tlc.set(8, 1000);
  Tlc.set(11, 300);
  Tlc.set(12, 500);
  Tlc.set(14, 1000);
  Tlc.set(17, 300);
  Tlc.set(18, 500);
  Tlc.set(20, 1000);
  Tlc.set(23, 300);
  Tlc.update();
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
  
  Tlc.init();
  Serial.begin(9600);
}



void loop() {
  shapeMSGEQ7(smoothP);
  if (cnt > 64) {                          // <----- change this value for different sample sizes
    for (band = 0; band < 7; band++) {
      resetCS(band);
    }
  }
  cnt++;
  
  // determine activeMode from DeskModuleRF
  activeMode = digitalRead(COMM_ONE) + (2 * digitalRead(COMM_TWO)) + (4 * digitalRead(COMM_THREE)) + (8 * digitalRead(COMM_FOUR)) + (16 *digitalRead(COMM_FIVE)) + (32 * digitalRead(COMM_SIX));
  Serial.println(activeMode);
  
  // RGB fading for horizonal spectrum bars
  if (counter1 < 256) {
    green1 = counter1;
    red1 = 255 - counter1;
  } else if (counter1 < 512) {
    blue1 = counter1 - 256; 
    green1 = 511 - counter1;
  } else if (counter1 < 768) {
    red1 = counter1 - 512;
    blue1 = 767 - counter1;
  } else {
    counter1 = 0;
  }
  counter1++;
  if (counter2 < 256) {
    green2 = counter2;
    red2 = 255 - counter2;
  } else if (counter2 < 512) {
    blue2 = counter2 - 256; 
    green2 = 511 - counter2;
  } else if (counter2 < 768) {
    red2 = counter2 - 512;
    blue2 = 767 - counter2;
  } else {
    counter2 = 0;
  }
  counter2++;
  if (counter3 < 256) {
    green3 = counter3;
    red3 = 255 - counter3;
  } else if (counter3 < 512) {
    blue3 = counter3 - 256; 
    green3 = 511 - counter3;
  } else if (counter3 < 768) {
    red3 = counter3 - 512;
    blue3 = 767 - counter3;
  } else {
    counter3 = 0;
  }
  counter3++;

  if (activeMode == 0) {
    Tlc.clear();
    int cutoff = 25;
    if ((left[0] > 200) && (left[1] > cutoff) && (left[2] > cutoff) && (left[3] > cutoff) && (left[4] > cutoff) && (left[5] > cutoff) && (left[6] > cutoff)) {
      if (left[0] > 250) {
        Tlc.set(6, red1 * 16);
        Tlc.set(7, green1 * 16);
        Tlc.set(8, blue1 * 16);
      }
      if (left[0] > 600) {
        Tlc.set(3, red2 * 16);
        Tlc.set(4, green2 * 16);
        Tlc.set(5, blue2 * 16);
        Tlc.set(9, red2 * 16);
        Tlc.set(10, green2 * 16);
        Tlc.set(11, blue2 * 16);
      }
      if (left[0] > 850) {
        Tlc.set(0, red3 * 16);
        Tlc.set(1, green3 * 16);
        Tlc.set(2, blue3 * 16);
        Tlc.set(12, red3 * 16);
        Tlc.set(13, green3 * 16);
        Tlc.set(14, blue3 * 16);
      }
      
      if (left[0] > 800) {
        Tlc.set(15, left[0] * 4);
        Tlc.set(16, left[0] * 4);
        Tlc.set(17, left[0] * 4);
        Tlc.set(18, left[0] * 4);
        Tlc.set(19, left[0] * 4);
        Tlc.set(20, left[0] * 4);
        Tlc.set(21, left[0] * 4);
        Tlc.set(22, left[0] * 4);
        Tlc.set(23, left[0] * 4);
      } else if (left[0] >= 600) {
        Tlc.set(15, left[0]);
        Tlc.set(16, left[0]);
        Tlc.set(17, left[0]);
        Tlc.set(18, left[0]);
        Tlc.set(19, left[0]);
        Tlc.set(20, left[0]);
        Tlc.set(21, left[0]);
        Tlc.set(22, left[0]);
        Tlc.set(23, left[0]);
      } else {
        Tlc.set(15, 250);
        Tlc.set(16, 250);
        Tlc.set(17, 250);
        Tlc.set(18, 250);
        Tlc.set(19, 250);
        Tlc.set(20, 250);
        Tlc.set(21, 250);
        Tlc.set(22, 250);
        Tlc.set(23, 250);
      }
      Tlc.update();
    } else {
      Tlc.clear();
      Tlc.update();
    }
  } else if (activeMode == 1) {
    Tlc.clear();
    Tlc.set(6, red1 * 16);
    Tlc.set(7, green1 * 16);
    Tlc.set(8, blue1 * 16);
    Tlc.set(3, red2 * 16);
    Tlc.set(4, green2 * 16);
    Tlc.set(5, blue2 * 16);
    Tlc.set(9, red2 * 16);
    Tlc.set(10, green2 * 16);
    Tlc.set(11, blue2 * 16);
    Tlc.set(0, red3 * 16);
    Tlc.set(1, green3 * 16);
    Tlc.set(2, blue3 * 16);
    Tlc.set(12, red3 * 16);
    Tlc.set(13, green3 * 16);
    Tlc.set(14, blue3 * 16);
    
    if (left[0] > 800) {
      Tlc.set(15, left[0] * 2);
      Tlc.set(16, left[0] * 2);
      Tlc.set(17, left[0] * 2);
      Tlc.set(18, left[0] * 2);
      Tlc.set(19, left[0] * 2);
      Tlc.set(20, left[0] * 2);
      Tlc.set(21, left[0] * 2);
      Tlc.set(22, left[0] * 2);
      Tlc.set(23, left[0] * 2);
    } else {
      Tlc.set(15, 250);
      Tlc.set(16, 250);
      Tlc.set(17, 250);
      Tlc.set(18, 250);
      Tlc.set(19, 250);
      Tlc.set(20, 250);
      Tlc.set(21, 250);
      Tlc.set(22, 250);
      Tlc.set(23, 250);
    }
    Tlc.update();
  } else if (activeMode == 2) {
    halogen();
  } else if (activeMode == 3) {
    studyMode();
  } else if (activeMode == 4) {
    chillMode();
  } else if (activeMode == 5) {
    roomEntered = false;
    Tlc.clear();
    Tlc.update();
  } else if (activeMode == 6) {
    nightMode();
  } else if (activeMode == 7) {
    roomEntered = false;
    Tlc.clear();
    Tlc.update();
  }
  
  delay(25);
}
