#include <Arduino.h>
#include <SPI.h>
#include <math.h>
#include <Adafruit_LIS3DH.h>
#include <Adafruit_Sensor.h>

// ── Pin Definitions ──────────────────────────────────────────────
#define ADXL355_CS  21
#define LIS3DH_CS   5
#define SPI_FREQ    4000000 // Increased to 4MHz for faster sensor reads

// ── ADXL355 Registers ────────────────────────────────────────────
#define ADXL_REG_DEVID_AD   0x00
#define ADXL_REG_STATUS     0x04
#define ADXL_REG_XDATA3     0x08
#define ADXL_REG_FILTER     0x28 // Filter register for ODR
#define ADXL_REG_RANGE      0x2C
#define ADXL_REG_POWER_CTL  0x2D
#define ADXL_REG_RESET      0x2F
#define ADXL_SCALE_2G       (1.0f / 256000.0f)

// ── Timing for 200Hz Accuracy ────────────────────────────────────
// 1000us = 1ms = 1kHz Sampling Rate
const uint32_t INTERVAL_US = 1000; 
uint32_t lastMicros = 0;

Adafruit_LIS3DH lis = Adafruit_LIS3DH(LIS3DH_CS);
SPISettings adxlSettings(SPI_FREQ, MSBFIRST, SPI_MODE0);

// ── Helpers ──────────────────────────────────────────────────────
uint8_t adxlRead(uint8_t reg) {
    uint8_t result;
    SPI.beginTransaction(adxlSettings);
    digitalWrite(ADXL355_CS, LOW);
    SPI.transfer((reg << 1) | 0x01);
    result = SPI.transfer(0x00);
    digitalWrite(ADXL355_CS, HIGH);
    SPI.endTransaction();
    return result;
}

void adxlWrite(uint8_t reg, uint8_t value) {
    SPI.beginTransaction(adxlSettings);
    digitalWrite(ADXL355_CS, LOW);
    SPI.transfer((reg << 1) & 0xFE);
    SPI.transfer(value);
    digitalWrite(ADXL355_CS, HIGH);
    SPI.endTransaction();
}

struct AccelData { float x, y, z, total; };

AccelData readADXL355() {
    uint8_t buf[9];
    SPI.beginTransaction(adxlSettings);
    digitalWrite(ADXL355_CS, LOW);
    SPI.transfer((ADXL_REG_XDATA3 << 1) | 0x01);
    for (uint8_t i = 0; i < 9; i++) buf[i] = SPI.transfer(0x00);
    digitalWrite(ADXL355_CS, HIGH);
    SPI.endTransaction();

    auto twosComp = [](uint32_t raw) -> int32_t {
        if (raw & 0x80000) return (int32_t)(raw | 0xFFF00000);
        return (int32_t)raw;
    };

    uint32_t rX = ((uint32_t)buf[0] << 12) | ((uint32_t)buf[1] << 4) | (buf[2] >> 4);
    uint32_t rY = ((uint32_t)buf[3] << 12) | ((uint32_t)buf[4] << 4) | (buf[5] >> 4);
    uint32_t rZ = ((uint32_t)buf[6] << 12) | ((uint32_t)buf[7] << 4) | (buf[8] >> 4);

    AccelData d;
    d.x = twosComp(rX) * ADXL_SCALE_2G;
    d.y = twosComp(rY) * ADXL_SCALE_2G;
    d.z = twosComp(rZ) * ADXL_SCALE_2G;
    d.total = sqrtf(d.x*d.x + d.y*d.y + d.z*d.z);
    return d;
}

void setup() {
    // 1. Mandatory high baud rate for high-frequency streaming
    Serial.begin(921600); 
    
    pinMode(ADXL355_CS, OUTPUT);
    pinMode(LIS3DH_CS,  OUTPUT);
    digitalWrite(ADXL355_CS, HIGH);
    digitalWrite(LIS3DH_CS,  HIGH);

    SPI.begin(18, 19, 23);
    delay(100);

    // ── Init ADXL355 ──
    adxlWrite(ADXL_REG_RESET, 0x52);
    delay(100);
    
    adxlWrite(ADXL_REG_FILTER, 0x02); 
    adxlWrite(ADXL_REG_RANGE, 0x01);     // ±2g
    adxlWrite(ADXL_REG_POWER_CTL, 0x00); // measurement mode

    // ── Init LIS3DH ──
    if (lis.begin()) {
        lis.setRange(LIS3DH_RANGE_2_G);
        // Set to 1.6kHz (Low Power) or 1.25kHz (Normal) to ensure data is always fresh
        lis.setDataRate(LIS3DH_DATARATE_LOWPOWER_1K6HZ); 
    }
}

void loop() {
    uint32_t now = micros();

    // Trigger loop every 1000 microseconds (1kHz)
    if (now - lastMicros >= INTERVAL_US) {
        lastMicros = now;

        // 1. Read ADXL
        AccelData a = readADXL355();
        
        // 2. Read LIS3DH
        sensors_event_t event;
        lis.getEvent(&event);
        float lx = event.acceleration.x / 9.80665f;
        float ly = event.acceleration.y / 9.80665f;
        float lz = event.acceleration.z / 9.80665f;
        float lt = sqrtf(lx*lx + ly*ly + lz*lz);

        // 3. Output to Teleplot
        // Only printing Total and Z to save Serial bandwidth, add X/Y if needed
        Serial.printf(">ADXL_Atotal:%f\n", a.total);
        Serial.printf(">LIS_Atotal:%f\n",  lt);
    }
}