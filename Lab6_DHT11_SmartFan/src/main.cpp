// Level 1 / Lab 6 : DHT11 Smart Fan
//
// ต่อวงจรภายนอกตัวแรก: เซนเซอร์อุณหภูมิ/ความชื้น DHT11 (ขา DATA -> GPIO27)
// แล้วใช้อุณหภูมิสั่ง "พัดลมจำลอง" = LED น้ำเงินที่คุมความสว่างด้วย PWM (ต่อยอดจาก Lab 5)
//
//   AUTO   : อุณหภูมิ >= FAN_ON_TEMP_C พัดลมเริ่มหมุน (duty เพิ่มตามอุณหภูมิ)
//            อุณหภูมิ <= FAN_OFF_TEMP_C พัดลมหยุด  (ON กับ OFF ห่างกัน = hysteresis กันพัดลมสั่น)
//   MANUAL : กำหนด duty เองผ่าน Serial (+ - 0-9)
//   อ่านเซนเซอร์พลาดติดกันหลายครั้ง -> แจ้ง SENSOR ERROR (LED แดง) และไม่ใช้ค่าเก่าสั่งพัดลม
//   ไม่ได้ต่อเซนเซอร์ (หรือกด s ใน Serial) = ใช้ค่าจำลอง (SIM) สุ่มอุณหภูมิ/ความชื้นแทน โดยบนจอมีป้าย REAL หรือ SIM บอกเสมอ
// ปุ่ม BOOT บนบอร์ด = สลับ AUTO / MANUAL
//
// ขาที่ใช้ใน Lab นี้ (ค่าจริงอยู่ใน include/board_config.h):
//   DHT11 DATA     = GPIO27  (ห้ามใช้ GPIO21 เพราะเป็นขาไฟหลังจอ)
//   LED น้ำเงิน (B) = GPIO17  -> PWM (LEDC ช่อง 0) เป็นพัดลมจำลอง
//   LED แดง   (R)  = GPIO4   -> digitalWrite ธรรมดา บอกว่าเซนเซอร์มีปัญหา
//   LED เป็น active-LOW (ส่ง LOW = ติด) จึงกลับค่า duty ก่อนเขียน
//   ปุ่ม BOOT      = GPIO0

#include <Arduino.h>
#include <DHT.h>
#include <SPI.h>
#include <TFT_eSPI.h>

#include "app_config.h"
#include "board_config.h"

namespace {

// ---------------------------------------------------------------------
// สีและตำแหน่งบนจอ (เหมือน Lab 4-5)
// ---------------------------------------------------------------------
constexpr uint16_t rgb565(uint8_t r, uint8_t g, uint8_t b)
{
    return static_cast<uint16_t>(((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3));
}

constexpr uint16_t COLOR_BG = rgb565(8, 12, 18);
constexpr uint16_t COLOR_HEADER = rgb565(20, 70, 120);
constexpr uint16_t COLOR_CARD = rgb565(22, 30, 42);
constexpr uint16_t COLOR_BORDER = rgb565(60, 76, 98);
constexpr uint16_t COLOR_TEXT = rgb565(240, 244, 250);
constexpr uint16_t COLOR_DIM = rgb565(140, 155, 175);
constexpr uint16_t COLOR_GREEN = rgb565(60, 220, 120);
constexpr uint16_t COLOR_BLUE = rgb565(90, 165, 255);
constexpr uint16_t COLOR_RED = rgb565(240, 85, 85);
constexpr uint16_t COLOR_YELLOW = rgb565(250, 205, 70);
constexpr uint16_t COLOR_ORANGE = rgb565(255, 150, 60);

constexpr int16_t SCREEN_W = 320;
constexpr int16_t HEADER_H = 28;

constexpr int16_t TEMP_CARD_X = 8, TEMP_CARD_Y = 34, TEMP_CARD_W = 148, TEMP_CARD_H = 80;
constexpr int16_t HUM_CARD_X = 164, HUM_CARD_Y = 34, HUM_CARD_W = 148, HUM_CARD_H = 80;
constexpr int16_t FAN_CARD_X = 8, FAN_CARD_Y = 118, FAN_CARD_W = 304, FAN_CARD_H = 88;
constexpr int16_t BAR_X = 16, BAR_Y = 172, BAR_W = 288, BAR_H = 12;
constexpr int16_t FOOTER_Y = 212;

enum class Mode : uint8_t { Auto, Manual };

TFT_eSPI tft;
DHT dht(cyd::DHT11_PIN, DHT11);

Mode mode = Mode::Auto;

// ---- ผลการอ่าน DHT11 ----
float sensorTempC = NAN;       // ค่าล่าสุดที่อ่านสำเร็จ
float sensorHumidity = NAN;
bool haveReading = false;      // เคยอ่านสำเร็จอย่างน้อย 1 ครั้งหรือยัง
uint8_t consecutiveFails = 0;
bool sensorError = false;      // อ่านพลาดติดกัน >= DHT_FAIL_LIMIT
uint32_t readCount = 0;
uint32_t okCount = 0;
uint32_t failCount = 0;
uint32_t lastOkAt = 0;
uint32_t lastReadMicros = 0;   // อ่านครั้งล่าสุดใช้เวลากี่ µs (วัดจริง)

// ---- ค่าจำลอง (SIM) ----
// simChoice = สิ่งที่ผู้ใช้เลือกด้วย s: Off = ใช้เซนเซอร์จริง (และ fallback เป็น SIM เมื่อเซนเซอร์เสียถ้าเปิดไว้)
enum class SimChoice : uint8_t { Off, Random, Sweep };
// DataSource = แหล่งที่มาของค่าที่ "ใช้อยู่จริง" ขณะนี้ (ใช้แสดงป้าย REAL / SIM)
enum class DataSource : uint8_t { None, Real, SimRandom, SimSweep };

SimChoice simChoice = SimChoice::Off;
float simTemp = app::SIM_TEMP_START_C;
float simHum = app::SIM_HUM_START;
uint32_t lastSimAt = 0;
DataSource lastReportedSource = DataSource::None;

// ---- พัดลม ----
bool fanOn = false;            // สถานะ ON/OFF ของโหมด AUTO (ตัวจำสำหรับ hysteresis)
int fanDuty = 0;               // duty ที่สั่ง (%)
uint32_t fanValue = 0;         // ค่าที่เขียนเข้า LEDC จริง (0 ถึง PWM_MAX_VALUE)
bool errorLedOn = false;

// ---- จับเวลา ----
uint32_t lastReadAt = 0;
uint32_t lastFanAt = 0;
uint32_t lastDisplayAt = 0;
uint32_t lastStatusAt = 0;
uint32_t lastHeartbeatAt = 0;
uint32_t lastSerialAt = 0;
bool heartbeatOn = true;

// สิ่งที่วาดบนจอไปล่าสุด (ไว้เทียบว่าเปลี่ยนไหม จะได้วาดเฉพาะส่วนที่เปลี่ยน)
constexpr int NEVER = -32768;
int drawnTempKey = NEVER;
int drawnHumKey = NEVER;
int drawnDuty = NEVER;
int drawnMode = NEVER;
int drawnSource = NEVER;

// ---------------------------------------------------------------------
// ตัวช่วยวาด
// ---------------------------------------------------------------------
void drawText(const char *text, int16_t x, int16_t y, uint8_t font, uint16_t fg, uint16_t bg, uint16_t paddingW = 0)
{
    tft.setTextColor(fg, bg);
    tft.setTextPadding(paddingW);
    tft.drawString(text, x, y, font);
    tft.setTextPadding(0);
}

void drawHeader(const char *title)
{
    tft.fillRect(0, 0, SCREEN_W, HEADER_H, COLOR_HEADER);
    drawText(title, 8, 6, 2, COLOR_TEXT, COLOR_HEADER);
}

void drawCard(int16_t x, int16_t y, int16_t w, int16_t h, const char *label)
{
    tft.fillRoundRect(x, y, w, h, 6, COLOR_CARD);
    tft.drawRoundRect(x, y, w, h, 6, COLOR_BORDER);
    drawText(label, x + 8, y + 4, 2, COLOR_DIM, COLOR_CARD);
}

// ---------------------------------------------------------------------
// อุณหภูมิที่ "ใช้งาน" (จากเซนเซอร์จริง หรือจำลอง)
// ---------------------------------------------------------------------

// SIM sweep: คลื่นสามเหลี่ยม 0..1..0 ทุก SIM_PERIOD_MS (ไล่ขึ้น-ลงสม่ำเสมอ ผลทำซ้ำได้ ใช้ดู hysteresis ชัดๆ)
float sweepFraction(uint32_t now)
{
    const float half = static_cast<float>(app::SIM_PERIOD_MS) / 2.0f;
    const float t = static_cast<float>(now % app::SIM_PERIOD_MS);
    return (t < half) ? (t / half) : ((static_cast<float>(app::SIM_PERIOD_MS) - t) / half);
}

// SIM random: เดินสุ่ม (random walk) เปลี่ยนทีละน้อยในช่วง +-step แล้วบีบให้อยู่ในขอบเขต
float randomStep(float maxStep)
{
    return (static_cast<float>(random(-1000, 1001)) / 1000.0f) * maxStep;
}

void updateSimulationRandom()
{
    simTemp = constrain(simTemp + randomStep(app::SIM_TEMP_STEP_C), app::SIM_TEMP_MIN_C, app::SIM_TEMP_MAX_C);
    simHum = constrain(simHum + randomStep(app::SIM_HUM_STEP), app::SIM_HUM_MIN, app::SIM_HUM_MAX);
}

bool sensorValid()
{
    return haveReading && !sensorError;
}

// ตัดสินใจว่าตอนนี้ "ใช้ค่าจากไหน"
DataSource currentSource()
{
    if (simChoice == SimChoice::Random) return DataSource::SimRandom;
    if (simChoice == SimChoice::Sweep) return DataSource::SimSweep;
    if (sensorValid()) return DataSource::Real;
    if (sensorError && app::SIM_WHEN_NO_SENSOR) return DataSource::SimRandom;  // เซนเซอร์ไม่ตอบ -> ใช้ค่า SIM
    return DataSource::None;  // ยังไม่ได้อ่าน หรือเซนเซอร์เสียและปิด fallback ไว้
}

bool isSimulated(DataSource source)
{
    return source == DataSource::SimRandom || source == DataSource::SimSweep;
}

const char *sourceName(DataSource source)
{
    switch (source) {
        case DataSource::Real: return "REAL";
        case DataSource::SimRandom: return "SIM-RANDOM";
        case DataSource::SimSweep: return "SIM-SWEEP";
        case DataSource::None: return "NONE";
    }
    return "?";
}

// คืน NAN ถ้าไม่มีค่าที่ใช้ได้
float effectiveTemp(uint32_t now)
{
    switch (currentSource()) {
        case DataSource::Real: return sensorTempC;
        case DataSource::SimRandom: return simTemp;
        case DataSource::SimSweep:
            return app::SIM_TEMP_MIN_C + sweepFraction(now) * (app::SIM_TEMP_MAX_C - app::SIM_TEMP_MIN_C);
        case DataSource::None: break;
    }
    return NAN;
}

float effectiveHumidity(uint32_t now)
{
    switch (currentSource()) {
        case DataSource::Real: return sensorHumidity;
        case DataSource::SimRandom: return simHum;
        case DataSource::SimSweep:  // ความชื้นไล่สวนทางกับอุณหภูมิ
            return app::SIM_HUM_MAX - sweepFraction(now) * (app::SIM_HUM_MAX - app::SIM_HUM_MIN);
        case DataSource::None: break;
    }
    return NAN;
}

// ---------------------------------------------------------------------
// พัดลมจำลอง = PWM บน LED น้ำเงิน
//
// ขา LED บนบอร์ด (ค่าจริงอยู่ใน include/board_config.h):
//   LED น้ำเงิน (B) = GPIO17  <- ใช้ PWM (LEDC ช่อง FAN_CHANNEL) เป็น "พัดลมจำลอง" duty มาก = สว่างมาก = หมุนเร็ว
//   LED แดง   (R)  = GPIO4   <- เปิด/ปิดด้วย digitalWrite ธรรมดา ใช้บอกว่า "เซนเซอร์มีปัญหา" (ไม่ใช้ PWM)
//   LED เขียว (G)  = GPIO16  <- Lab นี้ไม่ใช้ (Lab 5 ใช้เป็น PWM)
// LED ทั้งสามเป็นแบบ active-LOW (ส่ง LOW = ติด) จึงต้องกลับค่า duty ก่อนเขียน เหมือน Lab 5
// ---------------------------------------------------------------------
void writeFanPwm(uint32_t value)
{
    // value = ค่า PWM ที่ "อยากให้ LED สว่าง" (0-255) -> active-LOW ต้องส่ง 255 - value
    ledcWrite(app::FAN_CHANNEL, cyd::LED_ACTIVE_LOW ? (app::PWM_MAX_VALUE - value) : value);
}

void applyFanDuty(int percent)
{
    fanDuty = constrain(percent, 0, 100);
    fanValue = (static_cast<uint32_t>(fanDuty) * app::PWM_MAX_VALUE + 50) / 100;
    writeFanPwm(fanValue);
}

void pwmBegin()
{
    ledcSetup(app::FAN_CHANNEL, app::PWM_FREQUENCY_HZ, app::PWM_RESOLUTION_BITS);  // ตั้งความถี่ 5 kHz, 8 บิต
    ledcAttachPin(cyd::LED_PIN_B, app::FAN_CHANNEL);  // ผูกขา GPIO17 (LED น้ำเงิน) เข้ากับช่อง PWM
    applyFanDuty(0);
}

// LED แดง (GPIO4) = เซนเซอร์มีปัญหา เปิด/ปิดด้วย digitalWrite ปกติ (ไม่ใช่ PWM) ไม่ยุ่งกับขา GPIO17 ของ LED น้ำเงินที่เป็น PWM
void setErrorLed(bool on)
{
    if (on == errorLedOn) return;
    errorLedOn = on;
    // active-LOW: ติด = LOW, ดับ = HIGH
    digitalWrite(cyd::LED_PIN_R, (on == cyd::LED_ACTIVE_LOW) ? LOW : HIGH);
}

// duty ของพัดลมตามอุณหภูมิ: FAN_ON_TEMP_C -> FAN_MIN_DUTY_PERCENT ... FAN_FULL_TEMP_C -> 100%
int dutyForTemp(float tempC)
{
    const float span = app::FAN_FULL_TEMP_C - app::FAN_ON_TEMP_C;
    const float fraction = constrain((tempC - app::FAN_ON_TEMP_C) / span, 0.0f, 1.0f);
    return app::FAN_MIN_DUTY_PERCENT + static_cast<int>(lroundf(fraction * (100 - app::FAN_MIN_DUTY_PERCENT)));
}

// ตรรกะพัดลมโหมด AUTO (มี hysteresis)
void updateFanAuto(uint32_t now)
{
    const float tempC = effectiveTemp(now);
    const bool wasOn = fanOn;

    if (isnan(tempC)) {
        fanOn = false;  // ไม่มีค่าที่เชื่อถือได้ -> หยุดพัดลม (เลือกแบบนี้เพื่อความง่าย ดูแบบฝึกหัดเรื่อง fail-safe)
    } else if (!fanOn && tempC >= app::FAN_ON_TEMP_C) {
        fanOn = true;
    } else if (fanOn && tempC <= app::FAN_OFF_TEMP_C) {
        fanOn = false;
    }
    // ระหว่าง FAN_OFF_TEMP_C กับ FAN_ON_TEMP_C: คงสถานะเดิมไว้ นี่คือ hysteresis

    applyFanDuty((fanOn && !isnan(tempC)) ? dutyForTemp(tempC) : 0);

    if (wasOn != fanOn) {
        if (isnan(tempC)) {
            Serial.printf("[FAN] %s -> %s (no valid temperature)\n", wasOn ? "ON" : "OFF", fanOn ? "ON" : "OFF");
        } else {
            Serial.printf("[FAN] %s -> %s at %.1f C\n", wasOn ? "ON" : "OFF", fanOn ? "ON" : "OFF", tempC);
        }
    }
}

// ---------------------------------------------------------------------
// อ่าน DHT11
// ---------------------------------------------------------------------
void readDht(uint32_t now)
{
    readCount++;
    const uint32_t startedAt = micros();
    const float humidity = dht.readHumidity();
    const float tempC = dht.readTemperature();
    lastReadMicros = micros() - startedAt;

    if (!isnan(humidity) && !isnan(tempC)) {
        sensorTempC = tempC;
        sensorHumidity = humidity;
        haveReading = true;
        consecutiveFails = 0;
        sensorError = false;
        okCount++;
        lastOkAt = now;
        Serial.printf("[DHT11] #%lu OK     temp=%.1f C  hum=%.0f %%  (read took %lu us)\n",
                      static_cast<unsigned long>(readCount), tempC, humidity, static_cast<unsigned long>(lastReadMicros));
    } else {
        failCount++;
        if (consecutiveFails < 255) consecutiveFails++;
        sensorError = consecutiveFails >= app::DHT_FAIL_LIMIT;
        Serial.printf("[DHT11] #%lu FAILED (checksum/timeout) fails in a row=%u%s  (read took %lu us)\n",
                      static_cast<unsigned long>(readCount), consecutiveFails,
                      sensorError ? "  -> SENSOR ERROR: check wiring (VCC/DATA/GND)" : "",
                      static_cast<unsigned long>(lastReadMicros));
    }
}

// ---------------------------------------------------------------------
// Serial และปุ่ม BOOT
// ---------------------------------------------------------------------
const char *modeName(Mode m)
{
    return m == Mode::Auto ? "AUTO" : "MANUAL";
}

void printHelp()
{
    Serial.println("[HELP] keys: m = AUTO/MANUAL,  + / - = fan duty +-10% (MANUAL),  0-9 = fan duty 0..90% (MANUAL),");
    Serial.println("[HELP]       s = data source REAL -> SIM random -> SIM sweep,  h = help.   BOOT button on the board = AUTO/MANUAL");
}

void printStatus()
{
    const uint32_t now = millis();
    const DataSource source = currentSource();
    const float tempC = effectiveTemp(now);
    const float humidity = effectiveHumidity(now);
    char tempText[16], humText[16];
    if (isnan(tempC)) snprintf(tempText, sizeof(tempText), "--"); else snprintf(tempText, sizeof(tempText), "%.1f", tempC);
    if (isnan(humidity)) snprintf(humText, sizeof(humText), "--"); else snprintf(humText, sizeof(humText), "%.0f", humidity);

    Serial.printf("[STATUS] src=%-10s temp=%s C hum=%s %% | mode=%s fan=%s duty=%3d%% value=%3lu/%lu | dht ok=%lu fail=%lu%s\n",
                  sourceName(source), tempText, humText, modeName(mode), fanDuty > 0 ? "RUN " : "STOP", fanDuty,
                  static_cast<unsigned long>(fanValue), static_cast<unsigned long>(app::PWM_MAX_VALUE),
                  static_cast<unsigned long>(okCount), static_cast<unsigned long>(failCount),
                  sensorError ? "  SENSOR ERROR" : "");
}

void setMode(Mode next)
{
    mode = next;
    if (mode == Mode::Auto) {
        fanOn = false;  // เริ่มประเมินใหม่จาก "ปิด" เพื่อให้ผลแน่นอน
    }
    Serial.printf("[MODE] -> %s\n", modeName(mode));
}

void toggleMode()
{
    setMode(mode == Mode::Auto ? Mode::Manual : Mode::Auto);
}

// คำสั่งที่ตั้ง duty เอง -> บังคับเป็น MANUAL (ไม่เช่นนั้น AUTO จะเขียนทับใน 200 ms)
void manualDuty(int percent)
{
    if (mode != Mode::Manual) {
        setMode(Mode::Manual);
    }
    applyFanDuty(percent);
    Serial.printf("[FAN] manual duty = %d%% (value %lu/%lu)\n", fanDuty, static_cast<unsigned long>(fanValue),
                  static_cast<unsigned long>(app::PWM_MAX_VALUE));
}

void handleSerialInput()
{
    while (Serial.available() > 0) {
        const int c = Serial.read();
        if (c == 'm' || c == 'M') {
            toggleMode();
        } else if (c == '+' || c == '=') {
            manualDuty(fanDuty + app::MANUAL_STEP_PERCENT);
        } else if (c == '-' || c == '_') {
            manualDuty(fanDuty - app::MANUAL_STEP_PERCENT);
        } else if (c >= '0' && c <= '9') {
            manualDuty((c - '0') * 10);
        } else if (c == 's' || c == 'S') {
            // s: REAL (เซนเซอร์จริง) -> SIM random -> SIM sweep -> REAL
            simChoice = static_cast<SimChoice>((static_cast<uint8_t>(simChoice) + 1) % 3);
            if (simChoice == SimChoice::Off) {
                Serial.println("[SIM] off: using the real DHT11 sensor");
            } else if (simChoice == SimChoice::Random) {
                Serial.println("[SIM] random: simulated temperature/humidity (random walk)");
            } else {
                Serial.println("[SIM] sweep: simulated temperature/humidity (smooth up-down cycle)");
            }
        } else if (c == 'h' || c == 'H' || c == '?') {
            printHelp();
        }
        // ตัวอักษรอื่น (เช่น Enter) ข้ามไป
    }
}

// ปุ่ม BOOT: คืน true หนึ่งครั้งเมื่อ "เพิ่งกด" (หลังกันปุ่มเด้งแล้ว)
bool bootButtonPressed(uint32_t now)
{
    static int stableState = HIGH;
    static uint32_t lastChangeAt = 0;
    const int reading = digitalRead(cyd::BOOT_BUTTON_PIN);
    if (reading != stableState && now - lastChangeAt >= app::BUTTON_DEBOUNCE_MS) {
        stableState = reading;
        lastChangeAt = now;
        return reading == LOW;
    }
    return false;
}

// ---------------------------------------------------------------------
// หน้าจอ
// ---------------------------------------------------------------------
void drawMainScreenStatic()
{
    tft.fillScreen(COLOR_BG);
    drawHeader("Lab 6 - DHT11 Smart Fan");

    drawCard(TEMP_CARD_X, TEMP_CARD_Y, TEMP_CARD_W, TEMP_CARD_H, "TEMP");
    drawCard(HUM_CARD_X, HUM_CARD_Y, HUM_CARD_W, HUM_CARD_H, "HUMIDITY");
    drawText("C", TEMP_CARD_X + 116, TEMP_CARD_Y + 48, 4, COLOR_DIM, COLOR_CARD);
    drawText("%", HUM_CARD_X + 76, HUM_CARD_Y + 48, 4, COLOR_DIM, COLOR_CARD);

    drawCard(FAN_CARD_X, FAN_CARD_Y, FAN_CARD_W, FAN_CARD_H, "FAN (PWM)");
    tft.drawRect(BAR_X - 1, BAR_Y - 1, BAR_W + 2, BAR_H + 2, COLOR_BORDER);

    char text[48];
    snprintf(text, sizeof(text), "ON >= %.0fC  OFF <= %.0fC  MAX %.0fC", app::FAN_ON_TEMP_C, app::FAN_OFF_TEMP_C,
             app::FAN_FULL_TEMP_C);
    drawText(text, FAN_CARD_X + 8, 188, 2, COLOR_DIM, COLOR_CARD, 288);

    drawnTempKey = NEVER;
    drawnHumKey = NEVER;
    drawnDuty = NEVER;
    drawnMode = NEVER;
    drawnSource = NEVER;
}

// อัปเดตเฉพาะส่วนที่เปลี่ยน (หลักการเดียวกับ Lab 4)
void updateDisplay(uint32_t now)
{
    char text[40];

    // ---- อุณหภูมิ ----
    const float tempC = effectiveTemp(now);
    const int tempKey = isnan(tempC) ? -1 : static_cast<int>(lroundf(tempC * 10.0f));

    // ---- ป้ายบอกแหล่งที่มาของค่า: REAL (เขียว) / SIM (เหลือง) / N/A (เทา) ----
    const DataSource source = currentSource();
    if (static_cast<int>(source) != drawnSource) {
        const char *badge = (source == DataSource::Real) ? "REAL" : (isSimulated(source) ? "SIM" : "N/A");
        const uint16_t badgeColor = (source == DataSource::Real) ? COLOR_GREEN : (isSimulated(source) ? COLOR_YELLOW : COLOR_DIM);
        drawText(badge, TEMP_CARD_X + 104, TEMP_CARD_Y + 4, 2, badgeColor, COLOR_CARD, 40);
        drawText(badge, HUM_CARD_X + 104, HUM_CARD_Y + 4, 2, badgeColor, COLOR_CARD, 40);
        drawnSource = static_cast<int>(source);
    }

    if (tempKey != drawnTempKey) {
        uint16_t color = COLOR_TEXT;
        if (isnan(tempC)) {
            snprintf(text, sizeof(text), "--.-");
            color = sensorError ? COLOR_RED : COLOR_DIM;
        } else {
            snprintf(text, sizeof(text), "%.1f", tempC);
            color = (tempC >= app::FAN_ON_TEMP_C) ? COLOR_ORANGE : COLOR_TEXT;
        }
        tft.setTextSize(2);
        drawText(text, TEMP_CARD_X + 8, TEMP_CARD_Y + 22, 4, color, COLOR_CARD, 104);
        tft.setTextSize(1);
        drawnTempKey = tempKey;
    }

    // ---- ความชื้น ----
    const float humidity = effectiveHumidity(now);
    const int humKey = isnan(humidity) ? -1 : static_cast<int>(lroundf(humidity));
    if (humKey != drawnHumKey) {
        uint16_t color = COLOR_TEXT;
        if (isnan(humidity)) {
            snprintf(text, sizeof(text), "--");
            color = sensorError ? COLOR_RED : COLOR_DIM;
        } else {
            snprintf(text, sizeof(text), "%d", humKey);
        }
        tft.setTextSize(2);
        drawText(text, HUM_CARD_X + 8, HUM_CARD_Y + 22, 4, color, COLOR_CARD, 64);
        tft.setTextSize(1);
        drawnHumKey = humKey;
    }

    // ---- พัดลม ----
    if (static_cast<int>(mode) != drawnMode) {
        snprintf(text, sizeof(text), "MODE: %s", modeName(mode));
        drawText(text, FAN_CARD_X + 162, FAN_CARD_Y + 4, 2, mode == Mode::Auto ? COLOR_GREEN : COLOR_BLUE, COLOR_CARD, 134);
        drawnMode = static_cast<int>(mode);
    }
    if (fanDuty != drawnDuty) {
        snprintf(text, sizeof(text), "%d%%", fanDuty);
        drawText(text, FAN_CARD_X + 8, FAN_CARD_Y + 22, 4, COLOR_TEXT, COLOR_CARD, 100);
        drawText(fanDuty > 0 ? "RUNNING" : "STOPPED", FAN_CARD_X + 162, FAN_CARD_Y + 22, 4,
                 fanDuty > 0 ? COLOR_GREEN : COLOR_DIM, COLOR_CARD, 130);

        const int16_t filled = static_cast<int16_t>((static_cast<long>(fanDuty) * BAR_W) / 100);
        tft.fillRect(BAR_X, BAR_Y, filled, BAR_H, COLOR_BLUE);
        tft.fillRect(BAR_X + filled, BAR_Y, BAR_W - filled, BAR_H, COLOR_BG);
        drawnDuty = fanDuty;
    }
}

// บรรทัดสถานะท้ายจอ: บอกว่าเซนเซอร์เป็นอย่างไร
void updateStatusLine(uint32_t now)
{
    char text[48];
    uint16_t color = COLOR_DIM;
    const DataSource source = currentSource();
    if (simChoice == SimChoice::Random) {
        snprintf(text, sizeof(text), "SIM random (s = next source)");
        color = COLOR_YELLOW;
    } else if (simChoice == SimChoice::Sweep) {
        snprintf(text, sizeof(text), "SIM sweep (s = next source)");
        color = COLOR_YELLOW;
    } else if (source == DataSource::SimRandom) {
        snprintf(text, sizeof(text), "SIM: no sensor - check wiring");  // fallback อัตโนมัติเมื่อเซนเซอร์ไม่ตอบ
        color = COLOR_YELLOW;
    } else if (sensorError) {
        snprintf(text, sizeof(text), "SENSOR ERROR - check wiring");
        color = COLOR_RED;
    } else if (!haveReading) {
        snprintf(text, sizeof(text), "waiting for the first reading...");
    } else {
        snprintf(text, sizeof(text), "REAL OK %lu FAIL %lu last %lus", static_cast<unsigned long>(okCount),
                 static_cast<unsigned long>(failCount), static_cast<unsigned long>((now - lastOkAt) / 1000));
    }
    drawText(text, 8, FOOTER_Y, 2, color, COLOR_BG, 280);
}

void updateHeartbeat()
{
    heartbeatOn = !heartbeatOn;
    tft.fillCircle(SCREEN_W - 16, FOOTER_Y + 8, 5, heartbeatOn ? COLOR_GREEN : COLOR_BORDER);
}

}  // namespace

void setup()
{
    Serial.begin(app::SERIAL_BAUD);
    delay(200);

    Serial.println();
    Serial.println("==========================================");
    Serial.println("Hello World from ESP32-2432S028 (CYD)!");
    Serial.println("Level 1 / Lab 6 : DHT11 Smart Fan");
    Serial.println("==========================================");
    Serial.printf("[DHT11] data pin = GPIO%u (do NOT use GPIO21: it is the TFT backlight)\n", cyd::DHT11_PIN);

    cyd::ledInit();
    pinMode(cyd::BOOT_BUTTON_PIN, INPUT_PULLUP);

    tft.init();  // TFT_eSPI เปิดไฟหลังจอให้ตรงนี้
    tft.setRotation(cyd::DISPLAY_ROTATION);
    tft.setTextDatum(TL_DATUM);
    tft.fillScreen(COLOR_BG);

    pwmBegin();
    dht.begin();
    lastReadAt = millis();  // DHT11 ต้องรอให้เซนเซอร์พร้อมก่อน จึงให้อ่านครั้งแรกหลังผ่านไป DHT_READ_INTERVAL_MS

    drawMainScreenStatic();
    printHelp();
}

void loop()
{
    const uint32_t now = millis();

    // 1) อ่านเซนเซอร์เป็นระยะ
    if (now - lastReadAt >= app::DHT_READ_INTERVAL_MS) {
        lastReadAt = now;
        readDht(now);
    }

    // 2) ปุ่มและคำสั่ง Serial
    if (bootButtonPressed(now)) {
        toggleMode();
    }
    handleSerialInput();

    // 3) แหล่งข้อมูล: สุ่มค่า SIM (ถ้ากำลังใช้) และแจ้งเมื่อแหล่งเปลี่ยน (REAL <-> SIM)
    const DataSource source = currentSource();
    if (source == DataSource::SimRandom && now - lastSimAt >= app::SIM_UPDATE_MS) {
        lastSimAt = now;
        updateSimulationRandom();
    }
    if (source != lastReportedSource) {
        Serial.printf("[SRC] data source: %s -> %s\n", sourceName(lastReportedSource), sourceName(source));
        lastReportedSource = source;
    }

    // 4) ตรรกะพัดลม + LED แดงแจ้งปัญหา
    if (now - lastFanAt >= app::FAN_UPDATE_MS) {
        lastFanAt = now;
        if (mode == Mode::Auto) {
            updateFanAuto(now);
        }
        setErrorLed(sensorError && simChoice == SimChoice::Off);
    }

    // 5) วาดจอ (เฉพาะส่วนที่เปลี่ยน)
    if (now - lastDisplayAt >= app::DISPLAY_REFRESH_MS) {
        lastDisplayAt = now;
        updateDisplay(now);
    }
    if (now - lastStatusAt >= app::STATUS_REFRESH_MS) {
        lastStatusAt = now;
        updateStatusLine(now);
    }
    if (now - lastHeartbeatAt >= app::HEARTBEAT_MS) {
        lastHeartbeatAt = now;
        updateHeartbeat();
    }

    if (now - lastSerialAt >= app::SERIAL_PRINT_MS) {
        lastSerialAt = now;
        printStatus();
    }
}
