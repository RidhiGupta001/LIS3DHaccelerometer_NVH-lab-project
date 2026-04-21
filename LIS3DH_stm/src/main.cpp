#include <Arduino.h>
#include <SPI.h>
#include <Adafruit_LIS3DH.h>

// ── Timing & Pins ───────────────────────────────────────────────
#define ADXL355_CS    4
#define LIS3DH_CS     5
const uint32_t INTERVAL_US = 2500; // Exact 400Hz (1,000,000 / 400)
uint32_t nextMicros = 0;

// ── Sensor Registers ─────────────────────────────────────────────
#define ADXL_REG_FILTER     0x28 
#define ADXL_REG_RANGE      0x2C
#define ADXL_REG_POWER_CTL  0x2D
#define ADXL_REG_XDATA3     0x08
#define ADXL_SCALE          (1.0f / 256000.0f)

Adafruit_LIS3DH lis = Adafruit_LIS3DH(LIS3DH_CS);

void adxlWrite(uint8_t reg, uint8_t val) {
    SPI.beginTransaction(SPISettings(4000000, MSBFIRST, SPI_MODE0));
    digitalWrite(ADXL355_CS, LOW);
    SPI.transfer((reg << 1) & 0xFE);
    SPI.transfer(val);
    digitalWrite(ADXL355_CS, HIGH);
    SPI.endTransaction();
}

void setup() {
    // 1. Ultra-high baud rate to prevent Serial buffer stalls
    Serial.begin(921600); 
    while(!Serial);

    pinMode(ADXL355_CS, OUTPUT);
    digitalWrite(ADXL355_CS, HIGH);

    SPI.begin(18, 19, 23);

    // Init ADXL355
    adxlWrite(0x2F, 0x52); // Reset
    delay(100);
    adxlWrite(ADXL_REG_FILTER, 0x04);    // 500Hz ODR (Best for 400Hz sampling)
    adxlWrite(ADXL_REG_RANGE, 0x01);     // ±2g
    adxlWrite(ADXL_REG_POWER_CTL, 0x00); // Measurement mode

    // Init LIS3DH
    if (lis.begin()) {
        lis.setRange(LIS3DH_RANGE_2_G);
        lis.setDataRate(LIS3DH_DATARATE_400_HZ); // Match 400Hz
    }

    // Print CSV Header
    Serial.println("Timestamp_us,ADXL_X,ADXL_Y,ADXL_Z,LIS_X,LIS_Y,LIS_Z");
    
    nextMicros = micros();
}

void loop() {
    // Precise hardware-aligned timing
    if ((int32_t)(micros() - nextMicros) >= 0) {
        nextMicros += INTERVAL_US;

        // --- Read ADXL355 ---
        uint8_t buf[9];
        SPI.beginTransaction(SPISettings(4000000, MSBFIRST, SPI_MODE0));
        digitalWrite(ADXL355_CS, LOW);
        SPI.transfer((ADXL_REG_XDATA3 << 1) | 0x01);
        for(int i=0; i<9; i++) buf[i] = SPI.transfer(0x00);
        digitalWrite(ADXL355_CS, HIGH);
        SPI.endTransaction();

        auto conv = [](uint32_t r) {
            if (r & 0x80000) return (int32_t)(r | 0xFFF00000);
            return (int32_t)r;
        };

        float ax = conv(((uint32_t)buf[0] << 12) | ((uint32_t)buf[1] << 4) | (buf[2] >> 4)) * ADXL_SCALE;
        float ay = conv(((uint32_t)buf[3] << 12) | ((uint32_t)buf[4] << 4) | (buf[5] >> 4)) * ADXL_SCALE;
        float az = conv(((uint32_t)buf[6] << 12) | ((uint32_t)buf[7] << 4) | (buf[8] >> 4)) * ADXL_SCALE;

        // --- Read LIS3DH ---
        sensors_event_t event;
        lis.getEvent(&event);
        float lx = event.acceleration.x / 9.80665f;
        float ly = event.acceleration.y / 9.80665f;
        float lz = event.acceleration.z / 9.80665f;

        // --- Optimized CSV Output ---
        // Using microsecond timestamp to verify 400Hz intervals in Excel later
        Serial.print(micros()); Serial.print(",");
        Serial.print(ax, 4); Serial.print(",");
        Serial.print(ay, 4); Serial.print(",");
        Serial.print(az, 4); Serial.print(",");
        Serial.print(lx, 4); Serial.print(",");
        Serial.print(ly, 4); Serial.print(",");
        Serial.println(lz, 4);
    }
}