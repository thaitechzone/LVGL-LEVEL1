// Level 1 / Lab 2 : Template ADC + LDR
//
// ต่อจาก Lab 1 (Hello World + RGB LED) เพิ่ม "ประสาทสัมผัส" ให้บอร์ด
//   1) อ่านค่าแสงจาก LDR บนบอร์ด (GPIO34, ADC) แล้วพิมพ์ Raw Data ออก Serial Monitor
//   2) เอาค่าแสงไปควบคุม "ความเร็ว" ที่ RGB LED วิ่ง R -> G -> B
//        มืด / บังแสง  = วิ่งเร็ว
//        สว่าง         = วิ่งช้าๆ
//
// แนวคิดที่ฝึกใน Lab นี้: ADC, oversampling + ตัวกรอง (smoothing), calibration,
// map()/constrain(), state machine แบบง่าย (Calibrating -> Running),
// และการใช้ millis() จับเวลาหลายตัวพร้อมกันโดยไม่ใช้ delay()

#include <Arduino.h>

#include "app_config.h"
#include "board_config.h"

namespace {

enum class Phase { Calibrating, Running };

struct LedStep {
    const char *name;
    uint8_t pin;
    bool red;
    bool green;
    bool blue;
};

const LedStep steps[] = {
    {"RED", cyd::LED_PIN_R, true, false, false},
    {"GREEN", cyd::LED_PIN_G, false, true, false},
    {"BLUE", cyd::LED_PIN_B, false, false, true},
};
constexpr size_t STEP_COUNT = sizeof(steps) / sizeof(steps[0]);

Phase phase = Phase::Calibrating;
size_t currentStep = 0;

uint32_t phaseStartedAt = 0;
uint32_t lastSampleAt = 0;
uint32_t lastPrintAt = 0;
uint32_t lastStepAt = 0;
uint32_t lastBlinkAt = 0;
bool blinkOn = true;

int lastRaw = 0;            // ค่าดิบล่าสุด (เฉลี่ยจากการอ่านหลายครั้งแล้ว)
float smoothedRaw = 0.0f;   // ค่าที่ผ่านตัวกรองให้นิ่งขึ้น
bool smoothedReady = false;

int calMin = 4095;          // ค่า raw ต่ำสุดที่เจอตอน calibrate  (= สว่างที่สุด)
int calMax = 0;             // ค่า raw สูงสุดที่เจอตอน calibrate  (= มืดที่สุด)
int rawBright = app::DEFAULT_RAW_BRIGHT;  // raw ที่ถือว่า "สว่างเต็มที่" (ได้จาก calibration)
int rawDark = app::DEFAULT_RAW_DARK;      // raw ที่ถือว่า "มืดสุด"      (ได้จาก calibration)

// อ่าน ADC หลายครั้งแล้วเฉลี่ย ค่าที่ได้นิ่งกว่าอ่านครั้งเดียว (ช่วง 0-4095)
int readLdrRaw()
{
    uint32_t sum = 0;
    for (uint8_t i = 0; i < app::OVERSAMPLE_COUNT; i++) {
        sum += analogRead(cyd::LDR_PIN);
    }
    return static_cast<int>(sum / app::OVERSAMPLE_COUNT);
}

// ตัวกรองแบบ exponential moving average: ค่าใหม่ = ค่าเก่า + (ค่าอ่าน - ค่าเก่า) * alpha
void updateLdr()
{
    lastRaw = readLdrRaw();
    if (!smoothedReady) {
        smoothedRaw = static_cast<float>(lastRaw);
        smoothedReady = true;
    } else {
        smoothedRaw += (static_cast<float>(lastRaw) - smoothedRaw) * app::SMOOTH_ALPHA;
    }
}

// แปลงเป็นความสว่าง 0-100%  (0 = มืดสุด, 100 = สว่างสุด)
// บอร์ดนี้ raw "สูง" = มืด จึงใส่ rawDark เป็นปลายทางที่ 0% และ rawBright เป็น 100%
int lightPercent()
{
    const long pct = map(lroundf(smoothedRaw), rawDark, rawBright, 0, 100);
    return static_cast<int>(constrain(pct, 0L, 100L));
}

// ยิ่งมืด (0%) ยิ่งเร็ว (FAST_STEP_MS)  ยิ่งสว่าง (100%) ยิ่งช้า (SLOW_STEP_MS)
uint32_t stepIntervalMs(int percent)
{
    return static_cast<uint32_t>(map(percent, 0, 100, app::FAST_STEP_MS, app::SLOW_STEP_MS));
}

const char *modeLabel(int percent)
{
    if (percent < 35) {
        return "DARK -> FAST";
    }
    if (percent > 65) {
        return "BRIGHT -> SLOW";
    }
    return "MID";
}

// สร้างแถบความสว่างยาว 10 ช่อง เช่น [#######---]
void makeBar(int percent, char *out)
{
    const int filled = (percent + 5) / 10;
    for (int i = 0; i < 10; i++) {
        out[i] = (i < filled) ? '#' : '-';
    }
    out[10] = '\0';
}

void showStep(const LedStep &step)
{
    cyd::ledSet(step.red, step.green, step.blue);
}

void startCalibration()
{
    phase = Phase::Calibrating;
    phaseStartedAt = millis();
    calMin = static_cast<int>(lroundf(smoothedRaw));
    calMax = calMin;
    lastBlinkAt = phaseStartedAt;
    blinkOn = true;
    cyd::ledSet(true, true, true);

    Serial.println();
    Serial.println("---------------- CALIBRATION ----------------");
    Serial.printf("For %lu s: COVER the LDR with your hand for a moment, then UNCOVER it.\n",
                  static_cast<unsigned long>(app::CALIBRATION_MS / 1000));
    Serial.println("(LDR = small sensor near the top-right corner of the screen, label R21)");
    Serial.println("LED blinks WHITE during calibration.");
}

void finishCalibration()
{
    const int range = calMax - calMin;
    Serial.println("---------------- CALIBRATION RESULT ----------------");
    Serial.printf("raw min (brightest) = %d, raw max (darkest) = %d, range = %d\n", calMin, calMax, range);

    if (range >= app::MIN_CALIBRATION_RANGE) {
        rawBright = calMin;
        rawDark = calMax;
        Serial.println("OK: using the range measured in this room.");
    } else {
        rawBright = app::DEFAULT_RAW_BRIGHT;
        rawDark = app::DEFAULT_RAW_DARK;
        Serial.printf("Range too small (< %d): did you cover the sensor? Using defaults bright=%d dark=%d.\n",
                      app::MIN_CALIBRATION_RANGE, rawBright, rawDark);
        Serial.println("Press the EN/RESET button on the board to calibrate again.");
    }
    Serial.println("Running: DARK -> LED runs FAST,  BRIGHT -> LED runs SLOW.");
    Serial.println("----------------------------------------------------");

    phase = Phase::Running;
    currentStep = 0;
    showStep(steps[currentStep]);
    lastStepAt = millis();
}

void printCalibrationLine(uint32_t now)
{
    const uint32_t elapsed = now - phaseStartedAt;
    const uint32_t remainingMs = (elapsed < app::CALIBRATION_MS) ? (app::CALIBRATION_MS - elapsed) : 0;
    Serial.printf("[CAL] %lu s left  raw=%4d  smooth=%6.1f  min=%4d  max=%4d\n",
                  static_cast<unsigned long>((remainingMs + 999) / 1000), lastRaw, smoothedRaw, calMin, calMax);
}

void printRunningLine()
{
    const int percent = lightPercent();
    char bar[11];
    makeBar(percent, bar);
    Serial.printf("[LDR] raw=%4d  smooth=%6.1f  light=%3d%% [%s]  step=%4lu ms  LED=%-5s  %s\n", lastRaw,
                  smoothedRaw, percent, bar, static_cast<unsigned long>(stepIntervalMs(percent)),
                  steps[currentStep].name, modeLabel(percent));
}

}  // namespace

void setup()
{
    Serial.begin(app::SERIAL_BAUD);
    delay(200);

    Serial.println();
    Serial.println("==========================================");
    Serial.println("Hello World from ESP32-2432S028 (CYD)!");
    Serial.println("Level 1 / Lab 2 : Template ADC + LDR");
    Serial.println("==========================================");

    cyd::ledInit();

    // LDR บนบอร์ดนี้ให้แรงดันเปลี่ยนแค่ช่วงแคบๆ ใกล้ 0 V การลด attenuation เป็น 0 dB
    // (ช่วงวัดเต็มสเกลราว 1.1 V แทน 3.3 V) ทำให้ค่า raw กว้างขึ้นและแยกมืด/สว่างได้ชัดกว่า
    analogSetPinAttenuation(cyd::LDR_PIN, ADC_0db);

    updateLdr();
    startCalibration();
    lastSampleAt = millis();
    lastPrintAt = millis();
}

void loop()
{
    const uint32_t now = millis();

    // 1) อ่านค่า LDR เป็นระยะ (ตัวจับเวลาที่ 1)
    if (now - lastSampleAt >= app::SAMPLE_MS) {
        lastSampleAt = now;
        updateLdr();

        if (phase == Phase::Calibrating) {
            const int value = static_cast<int>(lroundf(smoothedRaw));
            if (value < calMin) {
                calMin = value;
            }
            if (value > calMax) {
                calMax = value;
            }
        }
    }

    if (phase == Phase::Calibrating) {
        // 2a) ระหว่าง calibrate: LED ขาวกะพริบ (ตัวจับเวลาที่ 2)
        if (now - lastBlinkAt >= 250) {
            lastBlinkAt = now;
            blinkOn = !blinkOn;
            cyd::ledSet(blinkOn, blinkOn, blinkOn);
        }

        if (now - lastPrintAt >= 1000) {
            lastPrintAt = now;
            printCalibrationLine(now);
        }

        if (now - phaseStartedAt >= app::CALIBRATION_MS) {
            finishCalibration();
        }
        return;
    }

    // 2b) ใช้งานจริง: เปลี่ยนสี R -> G -> B ด้วยความเร็วที่ขึ้นกับแสง (ตัวจับเวลาที่ 2)
    if (now - lastStepAt >= stepIntervalMs(lightPercent())) {
        lastStepAt = now;
        currentStep = (currentStep + 1) % STEP_COUNT;
        showStep(steps[currentStep]);
    }

    // 3) พิมพ์ Raw Data (ตัวจับเวลาที่ 3)
    if (now - lastPrintAt >= app::PRINT_MS) {
        lastPrintAt = now;
        printRunningLine();
    }
}
