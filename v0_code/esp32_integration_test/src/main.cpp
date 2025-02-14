#include <Arduino.h>
#include <MD_AD9833.h>
#include <Adafruit_ADS1X15.h>

// need a LOT of pins for the MUX
// should get a gpio extender or a board with more pins

// pinout 
// https://docs.espressif.com/projects/esp-dev-kits/en/latest/esp32s3/_images/ESP32-S3_DevKitC-1_pinlayout_v1.1.jpg

#define MUX_IN_HIGH_ENABLE 4
#define MUX_IN_LOW_ENABLE 5
#define MUX_OUT_HIGH_ENABLE 6
#define MUX_OUT_LOW_ENABLE 7
#define MUX_IN_HIGH_1 1
#define MUX_IN_HIGH_2 2
#define MUX_IN_HIGH_3 42
#define MUX_IN_LOW_1 41
#define MUX_IN_LOW_2 40
#define MUX_IN_LOW_3 39
#define MUX_OUT_HIGH_1 38
#define MUX_OUT_HIGH_2 37
#define MUX_OUT_HIGH_3 36
#define MUX_OUT_LOW_1 48
#define MUX_OUT_LOW_2 47
#define MUX_OUT_LOW_3 21



#define SPI_DATA_PIN 13 
#define SPI_CLK_PIN 12
#define SPI_CS_PIN 10
MD_AD9833 AD(SPI_DATA_PIN, SPI_CLK_PIN, SPI_CS_PIN);

#define I2C_SDA 11
#define I2C_SCL 14
TwoWire i2c_wire(0);

#define ADC_ADDR 0x48
Adafruit_ADS1115 ads;

enum MUX_TYPE {
  MUX_IN_HIGH,
  MUX_IN_LOW,
  MUX_OUT_HIGH,
  MUX_OUT_LOW
};

#define FIRST_PIN 1
#define LAST_PIN 8
int setMux(int mux_type, int target_pin);

void setup() {
  Serial.begin(9600);

  assert(i2c_wire.begin(I2C_SDA, I2C_SCL));
  assert(ads.begin(ADC_ADDR, &i2c_wire));

  ads.setDataRate(400);
  AD.begin();
  AD.setMode(MD_AD9833::MODE_SINE);
  AD.setFrequency(MD_AD9833::CHAN_0, 50000.0);

  pinMode(MUX_IN_HIGH_ENABLE, OUTPUT); pinMode(MUX_IN_LOW_ENABLE, OUTPUT);
  pinMode(MUX_OUT_HIGH_ENABLE, OUTPUT);
  pinMode(MUX_OUT_LOW_ENABLE, OUTPUT);

  pinMode(MUX_IN_HIGH_1, OUTPUT);
  pinMode(MUX_IN_HIGH_2, OUTPUT);
  pinMode(MUX_IN_HIGH_3, OUTPUT);
  pinMode(MUX_IN_LOW_1, OUTPUT);
  pinMode(MUX_IN_LOW_2, OUTPUT);
  pinMode(MUX_IN_LOW_3, OUTPUT);

  pinMode(MUX_OUT_HIGH_1, OUTPUT);
  pinMode(MUX_OUT_HIGH_2, OUTPUT);
  pinMode(MUX_OUT_HIGH_3, OUTPUT);
  pinMode(MUX_OUT_LOW_1, OUTPUT);
  pinMode(MUX_OUT_LOW_2, OUTPUT);
  pinMode(MUX_OUT_LOW_3, OUTPUT);

  // active low enable
  digitalWrite(MUX_IN_HIGH_ENABLE, HIGH);
  digitalWrite(MUX_IN_LOW_ENABLE, HIGH);
  digitalWrite(MUX_OUT_HIGH_ENABLE, HIGH);
  digitalWrite(MUX_OUT_LOW_ENABLE, HIGH);

  digitalWrite(MUX_IN_HIGH_1, LOW);
  digitalWrite(MUX_IN_HIGH_2, LOW);
  digitalWrite(MUX_IN_HIGH_3, LOW);
  digitalWrite(MUX_IN_LOW_1, LOW);
  digitalWrite(MUX_IN_LOW_2, LOW);
  digitalWrite(MUX_IN_LOW_3, LOW);

  digitalWrite(MUX_OUT_HIGH_1, LOW);
  digitalWrite(MUX_OUT_HIGH_2, LOW);
  digitalWrite(MUX_OUT_HIGH_3, LOW);
  digitalWrite(MUX_OUT_LOW_1, LOW);
  digitalWrite(MUX_OUT_LOW_2, LOW);
  digitalWrite(MUX_OUT_LOW_3, LOW);

}

void loop() {
  int16_t adc0 = 0;
  float volts0 = 0.0;
  for (int i = 0; i < 8; i++) {
    int node_in_high = ((0 + i) % 8) + 1;
    int node_in_low = ((1 + i) % 8) + 1;
    //Serial.printf("%d %d\n", node_in_high, node_in_low);

    setMux(MUX_IN_HIGH, node_in_high);
    setMux(MUX_IN_LOW, node_in_low);
    for (int j = 0; j < 5; j++) {
      int node_out_high = (node_in_high - 1 + 2 + j) % 8 + 1;
      int node_out_low = (node_in_low - 1 + 2 + j) % 8 + 1;
      // Serial.printf("%d %d,", node_out_high, node_out_low);

      setMux(MUX_OUT_HIGH, node_out_high);
      setMux(MUX_OUT_LOW, node_out_low);
      delay(10);

      adc0 = ads.readADC_SingleEnded(0);
      volts0 = ads.computeVolts(adc0);

      Serial.printf("/*%f*/\n", volts0);
    }
    Serial.printf("\n");
  }

  Serial.printf("\n\n");
  delay(1000);
}

int setMux(int mux_type, int target_pin) 
{
  if (target_pin < FIRST_PIN || target_pin > LAST_PIN) {
    return EXIT_FAILURE;
  }

  if (mux_type == MUX_IN_HIGH) {
    digitalWrite(MUX_IN_HIGH_ENABLE, HIGH);
    digitalWrite(MUX_IN_HIGH_1, ((target_pin - 1) >> 0) & 0b01);
    digitalWrite(MUX_IN_HIGH_2, ((target_pin - 1) >> 1) & 0b01);
    digitalWrite(MUX_IN_HIGH_3, ((target_pin - 1) >> 2) & 0b01);
    digitalWrite(MUX_IN_HIGH_ENABLE, LOW);
  } else if (mux_type == MUX_IN_LOW) {
    digitalWrite(MUX_IN_LOW_ENABLE, HIGH);
    digitalWrite(MUX_IN_LOW_1, ((target_pin - 1) >> 0) & 0b01);
    digitalWrite(MUX_IN_LOW_2, ((target_pin - 1) >> 1) & 0b01);
    digitalWrite(MUX_IN_LOW_3, ((target_pin - 1) >> 2) & 0b01);
    digitalWrite(MUX_IN_LOW_ENABLE, LOW);
  } else if (mux_type == MUX_OUT_HIGH) {
    digitalWrite(MUX_OUT_HIGH_ENABLE, HIGH);
    digitalWrite(MUX_OUT_HIGH_1, ((target_pin - 1) >> 0) & 0b01);
    digitalWrite(MUX_OUT_HIGH_2, ((target_pin - 1) >> 1) & 0b01);
    digitalWrite(MUX_OUT_HIGH_3, ((target_pin - 1) >> 2) & 0b01);
    digitalWrite(MUX_OUT_HIGH_ENABLE, LOW);
  } else if (mux_type == MUX_OUT_LOW) {
    digitalWrite(MUX_OUT_LOW_ENABLE, HIGH);
    digitalWrite(MUX_OUT_LOW_1, ((target_pin - 1) >> 0) & 0b01);
    digitalWrite(MUX_OUT_LOW_2, ((target_pin - 1) >> 1) & 0b01);
    digitalWrite(MUX_OUT_LOW_3, ((target_pin - 1) >> 2) & 0b01);
    digitalWrite(MUX_OUT_LOW_ENABLE, LOW);
  } else {
    return EXIT_FAILURE;
  }

  return EXIT_SUCCESS;
}