#include <Arduino.h>
#include <SPI.h>
#include <Adafruit_LIS3DH.h>
#include <Adafruit_Sensor.h>

#define CS1 5
#define CS2 21

#define MOSI 23
#define MISO 19
#define SCK  18

// Two sensor objects
Adafruit_LIS3DH lis1 = Adafruit_LIS3DH(CS1, MOSI, MISO, SCK);
Adafruit_LIS3DH lis2 = Adafruit_LIS3DH(CS2, MOSI, MISO, SCK);

const unsigned long interval_us = 10000; // 100 Hz
unsigned long previousMicros = 0;

void setup() {
  Serial.begin(115200);

  if (!lis1.begin()) {
    Serial.println("LIS1 not found!");
    while (1);
  }

  if (!lis2.begin()) {
    Serial.println("LIS2 not found!");
    while (1);
  }

  // Configure both sensors
  lis1.setRange(LIS3DH_RANGE_2_G);
  lis1.setDataRate(LIS3DH_DATARATE_100_HZ);

  lis2.setRange(LIS3DH_RANGE_2_G);
  lis2.setDataRate(LIS3DH_DATARATE_100_HZ);

  Serial.println("Both LIS3DH sensors initialized.");
}

void loop() {

  if (micros() - previousMicros >= interval_us) {
    previousMicros += interval_us;

    sensors_event_t event1, event2;

    lis1.read();
    lis1.getEvent(&event1);

    lis2.read();
    lis2.getEvent(&event2);

    // Print clean format for Teleplot
float x1 = event1.acceleration.x;
float y1 = event1.acceleration.y;
float z1 = event1.acceleration.z;

float x2 = event2.acceleration.x;
float y2 = event2.acceleration.y;
float z2 = event2.acceleration.z;

// Total acceleration
float T1 = sqrt(x1*x1 + y1*y1 + z1*z1);
float T2 = sqrt(x2*x2 + y2*y2 + z2*z2);

// ---- Print for Teleplot ----

Serial.print("X[0]:"); Serial.print(x1,4);
Serial.print(" X[1]:"); Serial.print(x2,4);

Serial.print(" Y[0]:"); Serial.print(y1,4);
Serial.print(" Y[1]:"); Serial.print(y2,4);

Serial.print(" Z[0]:"); Serial.print(z1,4);
Serial.print(" Z[1]:"); Serial.println(z2,4);

Serial.print(">A1:"); Serial.println(T1,4);
Serial.print(">S2:"); Serial.println(T2,4);
  }
}