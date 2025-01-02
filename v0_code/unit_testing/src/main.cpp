#include <Arduino.h>
#include <MD_AD9833.h>
#include <Adafruit_ADS1X15.h>

// not enough pins to toggle IN_HIGH and OUT_HIGH
// should get a gpio extender or a board with more pins

#define MUX_OUT_LOW_ENABLE 27
#define MUX_IN_LOW_ENABLE 32
#define MUX_IN_HIGH_1 13
#define MUX_IN_HIGH_2 12
#define MUX_IN_HIGH_3 14
#define MUX_IN_LOW_1 26
#define MUX_IN_LOW_2 25
#define MUX_IN_LOW_3 23
#define MUX_OUT_HIGH_1
#define MUX_OUT_HIGH_2
#define MUX_OUT_HIGH_3
#define MUX_OUT_LOW_1
#define MUX_OUT_LOW_2
#define MUX_OUT_LOW_3



MD_AD9833 AD(23, 18, 5);
Adafruit_ADS1115 ads;



void setup() {
  Serial.begin(9600);

  ads.begin();

  AD.begin();
  AD.setMode(MD_AD9833::MODE_SINE);
  AD.setFrequency(MD_AD9833::CHAN_0, 20.0);

  pinMode(MUX_OUT_LOW_ENABLE, OUTPUT);
  pinMode(MUX_IN_LOW_ENABLE, OUTPUT);
  pinMode(MUX_IN_HIGH_1, OUTPUT);
  pinMode(MUX_IN_HIGH_2, OUTPUT);
  pinMode(MUX_IN_HIGH_3, OUTPUT);
  pinMode(MUX_IN_LOW_1, OUTPUT);
  pinMode(MUX_IN_LOW_2, OUTPUT);
  pinMode(MUX_IN_LOW_3, OUTPUT);

  // active low enable
  //digitalWrite(MUX_OUT_LOW_ENABLE, LOW);
  digitalWrite(MUX_IN_LOW_ENABLE, LOW);

  // select pin 1
  digitalWrite(MUX_IN_HIGH_1, LOW);
  digitalWrite(MUX_IN_HIGH_2, LOW);
  digitalWrite(MUX_IN_HIGH_3, LOW);

  // select pin 2
  digitalWrite(MUX_IN_LOW_1, HIGH);
  digitalWrite(MUX_IN_LOW_2, LOW);
  digitalWrite(MUX_IN_LOW_3, LOW);
}

void loop() {
  int16_t adc0 = 0;
  float volts0 = 0.0;

  adc0 = ads.readADC_SingleEnded(0);
  volts0 = ads.computeVolts(adc0);

 // Serial.printf("value: %f\n", volts0);
 // delay(500);
}
