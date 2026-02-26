#include <Arduino.h>
#include <SPI.h>
#include <Adafruit_LIS3DH.h>
#include <Adafruit_Sensor.h>
#include <ArduinoFFT.h>

#define CS1 5
#define CS2 21
#define MOSI 23
#define MISO 19
#define SCK  18

#define SAMPLES 256
#define SAMPLING_FREQUENCY 100

Adafruit_LIS3DH lis1 = Adafruit_LIS3DH(CS1, MOSI, MISO, SCK);
Adafruit_LIS3DH lis2 = Adafruit_LIS3DH(CS2, MOSI, MISO, SCK);

double vReal1[SAMPLES];
double vImag1[SAMPLES];

double vReal2[SAMPLES];
double vImag2[SAMPLES];

ArduinoFFT<double> FFT1(vReal1, vImag1, SAMPLES, SAMPLING_FREQUENCY);
ArduinoFFT<double> FFT2(vReal2, vImag2, SAMPLES, SAMPLING_FREQUENCY);

unsigned long previousMicros = 0;
const unsigned long interval_us = 10000;  // 100 Hz

int sampleIndex = 0;

void setup() {
  Serial.begin(115200);
  delay(1000);

  if (!lis1.begin()) while (1);
  if (!lis2.begin()) while (1);

  lis1.setRange(LIS3DH_RANGE_2_G);
  lis2.setRange(LIS3DH_RANGE_2_G);

  lis1.setDataRate(LIS3DH_DATARATE_100_HZ);
  lis2.setDataRate(LIS3DH_DATARATE_100_HZ);
}

void loop() {

  if (micros() - previousMicros >= interval_us) {
    previousMicros += interval_us;

    sensors_event_t e1, e2;

    lis1.read();
    lis1.getEvent(&e1);

    lis2.read();
    lis2.getEvent(&e2);

    vReal1[sampleIndex] = e1.acceleration.x;
    vImag1[sampleIndex] = 0;

    vReal2[sampleIndex] = e2.acceleration.x;
    vImag2[sampleIndex] = 0;

    sampleIndex++;

    if (sampleIndex >= SAMPLES) {

      // ----- Remove DC -----
      double mean1 = 0, mean2 = 0;

      for (int i = 0; i < SAMPLES; i++) {
        mean1 += vReal1[i];
        mean2 += vReal2[i];
      }

      mean1 /= SAMPLES;
      mean2 /= SAMPLES;

      for (int i = 0; i < SAMPLES; i++) {
        vReal1[i] -= mean1;
        vReal2[i] -= mean2;
      }

      // ---- Sensor 1 FFT ----
      FFT1.windowing(FFT_WIN_TYP_HANN, FFT_FORWARD);
      FFT1.compute(FFT_FORWARD);
      FFT1.complexToMagnitude();

      // ---- Sensor 2 FFT ----
      FFT2.windowing(FFT_WIN_TYP_HANN, FFT_FORWARD);
      FFT2.compute(FFT_FORWARD);
      FFT2.complexToMagnitude();

      // ---- Print Frequency + Both FFT Magnitudes ----
      for (int i = 1; i < SAMPLES / 2; i++) {

        double freq = (i * 1.0 * SAMPLING_FREQUENCY) / SAMPLES;

        Serial.print(">F:");
        Serial.println(freq, 2);

        Serial.print(">S1:");
        Serial.println(vReal1[i], 4);

        Serial.print(">S2:");
        Serial.println(vReal2[i], 4);
      }

      Serial.println("END");

      sampleIndex = 0;
    }
  }
}