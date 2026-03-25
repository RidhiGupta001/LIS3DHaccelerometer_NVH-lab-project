#include <Arduino.h>
#include <SPI.h>
#include <math.h>
#include <Adafruit_LIS3DH.h>
#include <Adafruit_Sensor.h>

// ── Pin Definitions ──────────────────────────────────────────────
#define ADXL355_CS  21
#define LIS3DH_CS   5
#define SPI_FREQ    1000000

// ── ADXL355 Registers ────────────────────────────────────────────
#define ADXL_REG_DEVID_AD   0x00
#define ADXL_REG_DEVID_MST  0x01
#define ADXL_REG_PARTID     0x02
#define ADXL_REG_STATUS     0x04
#define ADXL_REG_XDATA3     0x08
#define ADXL_REG_RANGE      0x2C
#define ADXL_REG_POWER_CTL  0x2D
#define ADXL_REG_RESET      0x2F
#define ADXL_SCALE_2G       (1.0f / 256000.0f)

// ── FFT Configuration ─────────────────────────────────────────────
// FFT_SIZE must be a power of 2.
// Effective sample rate ~100 Hz (loop delay 10ms).
// Frequency resolution = SAMPLE_RATE / FFT_SIZE
#define FFT_SIZE    128
#define SAMPLE_RATE 100.0f
#define FFT_BINS    (FFT_SIZE / 2)

// ── LIS3DH ───────────────────────────────────────────────────────
Adafruit_LIS3DH lis = Adafruit_LIS3DH(LIS3DH_CS);

// ── ADXL355 SPI Settings ─────────────────────────────────────────
SPISettings adxlSettings(SPI_FREQ, MSBFIRST, SPI_MODE0);

// ── FFT Buffers ───────────────────────────────────────────────────
float adxl_xBuf[FFT_SIZE], adxl_yBuf[FFT_SIZE], adxl_zBuf[FFT_SIZE];
float lis_xBuf[FFT_SIZE],  lis_yBuf[FFT_SIZE],  lis_zBuf[FFT_SIZE];
float re[FFT_SIZE], im[FFT_SIZE];  // shared working arrays

int sampleIndex = 0;

// ────────────────────────────────────────────────────────────────
//  In-place Cooley–Tukey radix-2 FFT
// ────────────────────────────────────────────────────────────────
void fft(float* re, float* im, int n) {
    // Bit-reversal permutation
    for (int i = 1, j = 0; i < n; i++) {
        int bit = n >> 1;
        for (; j & bit; bit >>= 1) j ^= bit;
        j ^= bit;
        if (i < j) {
            float tr = re[i]; re[i] = re[j]; re[j] = tr;
            float ti = im[i]; im[i] = im[j]; im[j] = ti;
        }
    }
    // Butterfly passes
    for (int len = 2; len <= n; len <<= 1) {
        float ang = -2.0f * M_PI / len;
        float wRe = cosf(ang), wIm = sinf(ang);
        for (int i = 0; i < n; i += len) {
            float curRe = 1.0f, curIm = 0.0f;
            for (int j = 0; j < len / 2; j++) {
                float uRe = re[i + j], uIm = im[i + j];
                float vRe = re[i + j + len/2] * curRe - im[i + j + len/2] * curIm;
                float vIm = re[i + j + len/2] * curIm + im[i + j + len/2] * curRe;
                re[i + j]         = uRe + vRe;
                im[i + j]         = uIm + vIm;
                re[i + j + len/2] = uRe - vRe;
                im[i + j + len/2] = uIm - vIm;
                float newRe = curRe * wRe - curIm * wIm;
                float newIm = curRe * wIm + curIm * wRe;
                curRe = newRe; curIm = newIm;
            }
        }
    }
}

// ── Hanning Window ────────────────────────────────────────────────
void applyHanning(float* data, int n) {
    for (int i = 0; i < n; i++) {
        float w = 0.5f * (1.0f - cosf(2.0f * M_PI * i / (n - 1)));
        data[i] *= w;
    }
}

// ── Compute FFT and send as Teleplot XY series ────────────────────
// Teleplot XY format: >name:x:y  (x = freq Hz, y = magnitude g)
void sendFFT(const char* label, float* timeBuf) {
    for (int i = 0; i < FFT_SIZE; i++) {
        re[i] = timeBuf[i];
        im[i] = 0.0f;
    }
    applyHanning(re, FFT_SIZE);
    fft(re, im, FFT_SIZE);

    // Send bins 1..FFT_BINS-1  (skip DC bin 0)
    for (int k = 1; k < FFT_BINS; k++) {
        float freq = k * (SAMPLE_RATE / FFT_SIZE);
        float mag  = sqrtf(re[k]*re[k] + im[k]*im[k]) / FFT_BINS;
        Serial.printf(">%s:%.3f:%.6f\n", label, freq, mag);
    }
}

// ── ADXL355 Helpers ───────────────────────────────────────────────
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

void adxlReadBurst(uint8_t startReg, uint8_t* buf, uint8_t len) {
    SPI.beginTransaction(adxlSettings);
    digitalWrite(ADXL355_CS, LOW);
    SPI.transfer((startReg << 1) | 0x01);
    for (uint8_t i = 0; i < len; i++) buf[i] = SPI.transfer(0x00);
    digitalWrite(ADXL355_CS, HIGH);
    SPI.endTransaction();
}

int32_t adxlTwosComp20(uint32_t raw) {
    if (raw & 0x80000) return (int32_t)(raw | 0xFFF00000);
    return (int32_t)raw;
}

struct AccelData { float x, y, z, total; };

AccelData readADXL355() {
    uint8_t buf[9];
    adxlReadBurst(ADXL_REG_XDATA3, buf, 9);
    uint32_t rawX = ((uint32_t)buf[0] << 12) | ((uint32_t)buf[1] << 4) | (buf[2] >> 4);
    uint32_t rawY = ((uint32_t)buf[3] << 12) | ((uint32_t)buf[4] << 4) | (buf[5] >> 4);
    uint32_t rawZ = ((uint32_t)buf[6] << 12) | ((uint32_t)buf[7] << 4) | (buf[8] >> 4);
    AccelData d;
    d.x = adxlTwosComp20(rawX) * ADXL_SCALE_2G;
    d.y = adxlTwosComp20(rawY) * ADXL_SCALE_2G;
    d.z = adxlTwosComp20(rawZ) * ADXL_SCALE_2G;
    d.total = sqrtf(d.x*d.x + d.y*d.y + d.z*d.z);
    return d;
}

// ────────────────────────────────────────────────────────────────
//  Setup
// ────────────────────────────────────────────────────────────────
void setup() {
    Serial.begin(115200);
    delay(2000);
    Serial.println("\n=== Dual Sensor Init ===");

    pinMode(ADXL355_CS, OUTPUT);
    pinMode(LIS3DH_CS,  OUTPUT);
    digitalWrite(ADXL355_CS, HIGH);
    digitalWrite(LIS3DH_CS,  HIGH);
    delay(100);

    SPI.begin(18, 19, 23);
    delay(100);

    // ── Init ADXL355 ──
    Serial.println("\n--- ADXL355 ---");
    adxlWrite(ADXL_REG_RESET, 0x52);
    delay(100);
    uint8_t adxlId = adxlRead(ADXL_REG_DEVID_AD);
    Serial.printf("  DEVID_AD : 0x%02X (expect 0xAD) %s\n",
                  adxlId, adxlId == 0xAD ? "OK" : "FAIL");
    Serial.printf("  DEVID_MST: 0x%02X (expect 0x1D)\n", adxlRead(ADXL_REG_DEVID_MST));
    Serial.printf("  PART_ID  : 0x%02X (expect 0xED)\n", adxlRead(ADXL_REG_PARTID));

    if (adxlId == 0xAD) {
        adxlWrite(ADXL_REG_RANGE,     0x01);  // ±2g
        adxlWrite(ADXL_REG_POWER_CTL, 0x00);  // measure mode
        Serial.println("  ADXL355 ready!");
    } else {
        Serial.println("  ADXL355 NOT found - check wiring!");
    }

    // ── Init LIS3DH ──
    Serial.println("\n--- LIS3DH ---");
    if (!lis.begin(0x18)) {
        Serial.println("  LIS3DH NOT found - check wiring!");
    } else {
        lis.setRange(LIS3DH_RANGE_2_G);
        lis.setDataRate(LIS3DH_DATARATE_100_HZ);
        Serial.println("  LIS3DH ready!");
    }

    memset(adxl_xBuf, 0, sizeof(adxl_xBuf));
    memset(adxl_yBuf, 0, sizeof(adxl_yBuf));
    memset(adxl_zBuf, 0, sizeof(adxl_zBuf));
    memset(lis_xBuf,  0, sizeof(lis_xBuf));
    memset(lis_yBuf,  0, sizeof(lis_yBuf));
    memset(lis_zBuf,  0, sizeof(lis_zBuf));

    Serial.println("\n=== Streaming to Teleplot ===");
    Serial.printf("    FFT size      : %d samples\n",    FFT_SIZE);
    Serial.printf("    Sample rate   : %.0f Hz\n",       SAMPLE_RATE);
    Serial.printf("    Freq resolut. : %.3f Hz/bin\n",   SAMPLE_RATE / FFT_SIZE);
    Serial.printf("    Max frequency : %.1f Hz\n\n",     SAMPLE_RATE / 2.0f);
}

// ────────────────────────────────────────────────────────────────
//  Loop
// ────────────────────────────────────────────────────────────────
void loop() {
    // ── ADXL355 ──
    if (adxlRead(ADXL_REG_STATUS) & 0x01) {
        AccelData a = readADXL355();
        Serial.printf(">ADXL_Ax:%f\n",     a.x);
        Serial.printf(">ADXL_Ay:%f\n",     a.y);
        Serial.printf(">ADXL_Az:%f\n",     a.z);
        Serial.printf(">ADXL_Atotal:%f\n", a.total);

        adxl_xBuf[sampleIndex] = a.x;
        adxl_yBuf[sampleIndex] = a.y;
        adxl_zBuf[sampleIndex] = a.z;
    }

    // ── LIS3DH ──
    sensors_event_t event;
    lis.getEvent(&event);
    float lx = event.acceleration.x / 9.80665f;
    float ly = event.acceleration.y / 9.80665f;
    float lz = event.acceleration.z / 9.80665f;
    float lt = sqrtf(lx*lx + ly*ly + lz*lz);

    Serial.printf(">LIS_Ax:%f\n",     lx);
    Serial.printf(">LIS_Ay:%f\n",     ly);
    Serial.printf(">LIS_Az:%f\n",     lz);
    Serial.printf(">LIS_Atotal:%f\n", lt);

    lis_xBuf[sampleIndex] = lx;
    lis_yBuf[sampleIndex] = ly;
    lis_zBuf[sampleIndex] = lz;

    sampleIndex++;

    // ── When buffer full → compute & send FFT ──
    // Fires every 128 × 10 ms = 1.28 s
    if (sampleIndex >= FFT_SIZE) {
        sampleIndex = 0;
        sendFFT("ADXL_FFT_X", adxl_xBuf);
        sendFFT("ADXL_FFT_Y", adxl_yBuf);
        sendFFT("ADXL_FFT_Z", adxl_zBuf);
        sendFFT("LIS_FFT_X",  lis_xBuf);
        sendFFT("LIS_FFT_Y",  lis_yBuf);
        sendFFT("LIS_FFT_Z",  lis_zBuf);
    }

    delay(10);  // 10 ms → 100 Hz sample rate
}