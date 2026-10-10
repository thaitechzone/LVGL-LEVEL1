// Level 1 / Lab 5 : PWM
//
// เรียนรู้ PWM (Pulse Width Modulation) โดยใช้ PWM ปรับความสว่างของ "ไฟหลังจอ" (GPIO21)
// และ LED สีเขียวบนบอร์ด พร้อมแสดงค่าและรูปคลื่น PWM บนจอ + Serial
//
// ขาที่ใช้ PWM ใน Lab นี้ (ค่าจริงอยู่ใน include/board_config.h):
//   ไฟหลังจอ      = GPIO21  -> LEDC ช่อง 0  (BACKLIGHT_CHANNEL)
//   LED เขียว (G) = GPIO16  -> LEDC ช่อง 1  (LED_CHANNEL)  ส่วน LED แดง (GPIO4) / น้ำเงิน (GPIO17) ไม่ได้ใช้ใน Lab นี้
//   LED เป็น active-LOW (ส่ง LOW = ติด) จึงกลับค่า duty ก่อนเขียน (ดู writeLed())
//
//   duty cycle = สัดส่วนเวลาที่สัญญาณเป็น HIGH ใน 1 รอบ  (เช่น 25% = สว่างเป็นช่วงสั้นๆ 1 ใน 4)
//   ความถี่สูงพอ ตาเรามองไม่ทันว่ากะพริบ จึงเห็นเป็นความสว่างเฉลี่ย  (duty มาก = สว่างมาก)
//
// โหมด:  MANUAL  = กำหนด duty เองผ่าน Serial   (+ - 0-9 f m)
//        BREATHE = duty ไล่ขึ้น-ลงนุ่มๆ เอง
//        AUTO    = duty ตามแสงห้อง (LDR) เหมือน auto-brightness ของมือถือ
// ปุ่ม BOOT บนบอร์ด = สลับโหมด

#include <Arduino.h>
#include <SPI.h>
#include <TFT_eSPI.h>

#include "app_config.h"
#include "board_config.h"

namespace {

// ---------------------------------------------------------------------
// สี (RGB565) และตำแหน่งบนจอ (เหมือน Lab 4)
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
constexpr uint16_t COLOR_YELLOW = rgb565(250, 205, 70);

constexpr int16_t SCREEN_W = 320;
constexpr int16_t HEADER_H = 28;

constexpr int16_t DUTY_CARD_X = 8, DUTY_CARD_Y = 34, DUTY_CARD_W = 304, DUTY_CARD_H = 76;
constexpr int16_t WAVE_CARD_X = 8, WAVE_CARD_Y = 112, WAVE_CARD_W = 304, WAVE_CARD_H = 96;
constexpr int16_t FOOTER_Y = 214;

// พื้นที่วาดรูปคลื่น: แสดง 3 รอบ รอบละ 80 พิกเซล เส้นหนา 2 พิกเซล
constexpr int16_t WAVE_X = 54;
constexpr int16_t WAVE_PERIOD_PX = 80;
constexpr int16_t WAVE_PERIODS = 3;
constexpr int16_t WAVE_HIGH_Y = 142;
constexpr int16_t WAVE_LOW_Y = 172;
constexpr int16_t WAVE_THICK = 2;

enum class Phase { Calibrating, Running };
enum class Mode : uint8_t { Manual, Breathe, Auto };

TFT_eSPI tft;

Phase phase = Phase::Calibrating;
Mode mode = Mode::Manual;

// ---- PWM ----
uint8_t frequencyIndex = app::DEFAULT_FREQUENCY_INDEX;
double actualFrequency = 0;  // ความถี่จริงที่ฮาร์ดแวร์ตั้งให้ (ledcSetup คืนค่ามา)
int dutyPercent = 100;       // duty ที่สั่ง (%)
uint32_t pwmValue = app::PWM_MAX_VALUE;  // ค่าที่เขียนเข้า LEDC จริง (0 ถึง PWM_MAX_VALUE)
bool ledMirror = false;      // true = LED เขียวทำตาม duty เดียวกับไฟหลังจอ

// ---- LDR ----
int lastRaw = 0;
float smoothedRaw = 0.0f;
bool smoothedReady = false;
int calMin = 4095;
int calMax = 0;
int rawBright = app::DEFAULT_RAW_BRIGHT;
int rawDark = app::DEFAULT_RAW_DARK;

// ---- จับเวลา ----
uint32_t phaseStartedAt = 0;
uint32_t lastSampleAt = 0;
uint32_t lastPwmUpdateAt = 0;
uint32_t lastDisplayAt = 0;
uint32_t lastHeartbeatAt = 0;
uint32_t lastSerialAt = 0;
bool heartbeatOn = true;

// สิ่งที่วาดบนจอไปล่าสุด (ไว้เทียบว่าเปลี่ยนไหม จะได้วาดเฉพาะส่วนที่เปลี่ยน)
int drawnDuty = -1;
int drawnWaveDuty = -1;
int drawnFrequencyIndex = -1;
int drawnMode = -1;

// ---------------------------------------------------------------------
// ตัวช่วยวาด (เหมือน Lab 4)
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
// ชื่อโหมด / ความถี่ / เวลา
// ---------------------------------------------------------------------
const char *modeName(Mode m)
{
    switch (m) {
        case Mode::Manual: return "MANUAL";
        case Mode::Breathe: return "BREATHE";
        case Mode::Auto: return "AUTO (LDR)";
    }
    return "?";
}

uint16_t modeColor(Mode m)
{
    switch (m) {
        case Mode::Manual: return COLOR_BLUE;
        case Mode::Breathe: return COLOR_GREEN;
        case Mode::Auto: return COLOR_YELLOW;
    }
    return COLOR_TEXT;
}

uint32_t requestedFrequency()
{
    return app::PWM_FREQUENCIES[frequencyIndex];
}

// คาบ (เวลา 1 รอบ) เป็นไมโครวินาที: T = 1,000,000 / f
float periodMicros()
{
    return 1000000.0f / static_cast<float>(requestedFrequency());
}

// แปลงเวลา (µs) เป็นข้อความ: น้อยกว่า 1000 ใช้ "us" ตั้งแต่ 1000 ขึ้นไปใช้ "ms"
void formatMicros(char *out, size_t size, float us)
{
    if (us >= 1000.0f) {
        snprintf(out, size, "%.1fms", us / 1000.0f);
    } else {
        snprintf(out, size, "%.0fus", us);
    }
}

// ---------------------------------------------------------------------
// PWM : ใช้ LEDC ของ ESP32  (ledcSetup -> ledcAttachPin -> ledcWrite)
// ---------------------------------------------------------------------

// ตั้งความถี่ (ทั้ง 2 ช่องใช้ความถี่เดียวกัน)
void applyFrequency()
{
    actualFrequency = ledcSetup(app::BACKLIGHT_CHANNEL, requestedFrequency(), app::PWM_RESOLUTION_BITS);
    ledcSetup(app::LED_CHANNEL, requestedFrequency(), app::PWM_RESOLUTION_BITS);
}

// เขียนค่า PWM ไปที่ LED เขียว (GPIO16)
// LED บนบอร์ดเป็น active-LOW (ส่ง LOW = ติด) จึงต้องกลับค่า: ส่ง (ค่าสูงสุด - value)
//   value = ค่าที่ "อยากให้ LED สว่าง" (0 = ดับ, ค่าสูงสุด = สว่างสุด)
void writeLed(uint32_t value)
{
    ledcWrite(app::LED_CHANNEL, cyd::LED_ACTIVE_LOW ? (app::PWM_MAX_VALUE - value) : value);
}

// ตั้ง duty เป็น % -> แปลงเป็นค่า 0..PWM_MAX_VALUE แล้วเขียนเข้า LEDC
void applyDuty(int percent)
{
    dutyPercent = constrain(percent, app::MIN_DUTY_PERCENT, 100);
    pwmValue = (static_cast<uint32_t>(dutyPercent) * app::PWM_MAX_VALUE + 50) / 100;  // +50 = ปัดเศษ
    ledcWrite(app::BACKLIGHT_CHANNEL, pwmValue);
    writeLed(ledMirror ? pwmValue : 0);
}

void pwmBegin()
{
    applyFrequency();
    ledcAttachPin(cyd::TFT_PIN_BACKLIGHT, app::BACKLIGHT_CHANNEL);  // GPIO21 = ไฟหลังจอ -> PWM ช่อง 0
    ledcAttachPin(cyd::LED_PIN_G, app::LED_CHANNEL);                // GPIO16 = LED เขียว  -> PWM ช่อง 1 (LED_PIN_B = GPIO17 สำหรับน้ำเงิน)
    applyDuty(100);
}

void nextFrequency()
{
    frequencyIndex = (frequencyIndex + 1) % app::PWM_FREQUENCY_COUNT;
    applyFrequency();
    applyDuty(dutyPercent);  // ตั้งความถี่ใหม่แล้วต้องเขียน duty ซ้ำ
}

// ---------------------------------------------------------------------
// LDR (ย่อมาจาก Lab 2/4)
// ---------------------------------------------------------------------
int readLdrRaw()
{
    uint32_t sum = 0;
    for (uint8_t i = 0; i < app::OVERSAMPLE_COUNT; i++) {
        sum += analogRead(cyd::LDR_PIN);
    }
    return static_cast<int>(sum / app::OVERSAMPLE_COUNT);
}

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

// ความสว่าง 0-100% (บอร์ดนี้ raw สูง = มืด)
int lightPercent()
{
    const long pct = map(lroundf(smoothedRaw), rawDark, rawBright, 0, 100);
    return static_cast<int>(constrain(pct, 0L, 100L));
}

// ---------------------------------------------------------------------
// โหมดอัตโนมัติ: คำนวณ duty ที่ต้องการ
// ---------------------------------------------------------------------

// Breathe: ใช้ฟังก์ชัน cos ทำให้ไล่ขึ้น-ลงนุ่มๆ (ช้าตรงสุดทางทั้งสองด้าน)
int breatheDuty(uint32_t now)
{
    const float phaseFraction = static_cast<float>(now % app::BREATHE_PERIOD_MS) / static_cast<float>(app::BREATHE_PERIOD_MS);
    const float level = 0.5f - 0.5f * cosf(2.0f * PI * phaseFraction);  // 0.0 (มืด) -> 1.0 (สว่าง) -> 0.0
    return app::MIN_DUTY_PERCENT + static_cast<int>(lroundf(level * (100 - app::MIN_DUTY_PERCENT)));
}

// Auto: ห้องสว่าง -> จอสว่าง, ห้องมืด -> จอหรี่ (แต่ไม่ต่ำกว่า MIN_DUTY_PERCENT)
int autoDuty()
{
    return app::MIN_DUTY_PERCENT + (lightPercent() * (100 - app::MIN_DUTY_PERCENT)) / 100;
}

// ---------------------------------------------------------------------
// Serial
// ---------------------------------------------------------------------
void printStatus(const char *tag)
{
    char period[16], onTime[16], offTime[16];
    formatMicros(period, sizeof(period), periodMicros());
    formatMicros(onTime, sizeof(onTime), periodMicros() * dutyPercent / 100.0f);
    formatMicros(offTime, sizeof(offTime), periodMicros() * (100 - dutyPercent) / 100.0f);
    Serial.printf("[PWM] %s mode=%s duty=%3d%% value=%3lu/%lu freq=%lu Hz (actual %.0f) period=%s ON=%s OFF=%s | LDR raw=%4d light=%3d%%\n",
                  tag, modeName(mode), dutyPercent, static_cast<unsigned long>(pwmValue),
                  static_cast<unsigned long>(app::PWM_MAX_VALUE), static_cast<unsigned long>(requestedFrequency()),
                  actualFrequency, period, onTime, offTime, lastRaw, lightPercent());
}

void printHelp()
{
    Serial.println("[HELP] keys: + / - = duty +-5%,  0-9 = duty 0..90% (min 5%),  f = next frequency,  m = next mode,  h = help");
    Serial.println("[HELP] BOOT button on the board = next mode");
}

void setMode(Mode next)
{
    mode = next;
    Serial.printf("[MODE] -> %s\n", modeName(mode));
    printStatus("mode ");
}

void nextMode()
{
    setMode(static_cast<Mode>((static_cast<uint8_t>(mode) + 1) % 3));
}

// คำสั่งที่ปรับ duty เอง -> บังคับกลับเป็นโหมด MANUAL
void manualDuty(int percent)
{
    if (mode != Mode::Manual) {
        mode = Mode::Manual;
        Serial.println("[MODE] -> MANUAL");
    }
    applyDuty(percent);
    printStatus("duty ");
}

void handleSerialInput()
{
    while (Serial.available() > 0) {
        const int c = Serial.read();
        if (c == '+' || c == '=') {
            manualDuty(dutyPercent + app::DUTY_STEP_PERCENT);
        } else if (c == '-' || c == '_') {
            manualDuty(dutyPercent - app::DUTY_STEP_PERCENT);
        } else if (c >= '0' && c <= '9') {
            manualDuty((c - '0') * 10);
        } else if (c == 'f' || c == 'F') {
            nextFrequency();
            printStatus("freq ");
        } else if (c == 'm' || c == 'M') {
            nextMode();
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
// หน้าจอ 1: Calibrate (ย่อมาจาก Lab 4)
// ---------------------------------------------------------------------
void drawCalibrationScreen()
{
    tft.fillScreen(COLOR_BG);
    drawHeader("Lab 5 - Light sensor calibration");

    tft.setTextDatum(TC_DATUM);
    tft.setTextColor(COLOR_TEXT, COLOR_BG);
    tft.drawString("COVER the light sensor", SCREEN_W / 2, 52, 4);
    tft.drawString("with your hand for a moment,", SCREEN_W / 2, 86, 2);
    tft.drawString("then UNCOVER it.", SCREEN_W / 2, 106, 2);
    tft.setTextColor(COLOR_DIM, COLOR_BG);
    tft.drawString("(used by the AUTO brightness mode)", SCREEN_W / 2, 132, 2);
    tft.setTextDatum(TL_DATUM);

    tft.drawRect(30, 170, 260, 18, COLOR_BORDER);
}

void updateCalibrationScreen(uint32_t now)
{
    const uint32_t elapsed = min(now - phaseStartedAt, app::CALIBRATION_MS);
    const int16_t filled = static_cast<int16_t>((elapsed * 256UL) / app::CALIBRATION_MS);

    tft.fillRect(32, 172, filled, 14, COLOR_GREEN);
    tft.fillRect(32 + filled, 172, 256 - filled, 14, COLOR_BG);

    char text[48];
    snprintf(text, sizeof(text), "raw %4d    min %4d    max %4d", lastRaw, calMin, calMax);
    tft.setTextDatum(TC_DATUM);
    tft.setTextColor(COLOR_TEXT, COLOR_BG);
    tft.setTextPadding(260);
    tft.drawString(text, SCREEN_W / 2, 198, 2);
    tft.setTextPadding(0);
    tft.setTextDatum(TL_DATUM);
}

// ---------------------------------------------------------------------
// หน้าจอ 2: หน้าหลัก PWM
// ---------------------------------------------------------------------

// วาดรูปคลื่น PWM 3 รอบ: ช่วงสูง (HIGH) ยาว = duty% ของรอบ
// ใช้ฟังก์ชันเดียวกัน "วาดด้วยสีเดิมของพื้นหลัง" เพื่อลบของเก่า แล้ววาดใหม่ด้วยสีคลื่น
void drawWave(int percent, uint16_t color)
{
    const int16_t highPx = static_cast<int16_t>((percent * WAVE_PERIOD_PX) / 100);
    const int16_t edgeH = WAVE_LOW_Y - WAVE_HIGH_Y + WAVE_THICK;

    for (int16_t p = 0; p < WAVE_PERIODS; p++) {
        const int16_t x0 = WAVE_X + p * WAVE_PERIOD_PX;
        if (highPx > 0) {
            tft.fillRect(x0, WAVE_HIGH_Y, highPx, WAVE_THICK, color);  // เส้นระดับสูง
        }
        if (highPx < WAVE_PERIOD_PX) {
            tft.fillRect(x0 + highPx, WAVE_LOW_Y, WAVE_PERIOD_PX - highPx, WAVE_THICK, color);  // เส้นระดับต่ำ
        }
        if (highPx > 0 && highPx < WAVE_PERIOD_PX) {
            tft.fillRect(x0 + highPx, WAVE_HIGH_Y, WAVE_THICK, edgeH, color);  // ขอบขาลง
            if (p > 0) {
                tft.fillRect(x0, WAVE_HIGH_Y, WAVE_THICK, edgeH, color);       // ขอบขาขึ้น (จุดเริ่มรอบถัดไป)
            }
        }
    }
}

void drawMainScreenStatic()
{
    tft.fillScreen(COLOR_BG);
    drawHeader("Lab 5 - PWM");

    drawCard(DUTY_CARD_X, DUTY_CARD_Y, DUTY_CARD_W, DUTY_CARD_H, "DUTY CYCLE");
    drawCard(WAVE_CARD_X, WAVE_CARD_Y, WAVE_CARD_W, WAVE_CARD_H, "PWM WAVEFORM (3 periods)");
    drawText("3.3V", 12, WAVE_HIGH_Y - 8, 2, COLOR_DIM, COLOR_CARD);
    drawText("0V", 12, WAVE_LOW_Y - 8, 2, COLOR_DIM, COLOR_CARD);

    drawText("BOOT=mode  Serial: + - 0-9 f", 8, FOOTER_Y, 2, COLOR_DIM, COLOR_BG);

    drawnDuty = -1;
    drawnWaveDuty = -1;
    drawnFrequencyIndex = -1;
    drawnMode = -1;
}

// อัปเดตเฉพาะส่วนที่เปลี่ยน (เหมือนหลักการของ Lab 4)
void updatePwmDisplay()
{
    char text[40];
    const bool dutyChanged = dutyPercent != drawnDuty;
    const bool freqChanged = static_cast<int>(frequencyIndex) != drawnFrequencyIndex;
    const bool modeChanged = static_cast<int>(mode) != drawnMode;

    if (dutyChanged) {
        // เลข % ตัวใหญ่ (ฟอนต์ 4 ขยาย 2 เท่า)
        snprintf(text, sizeof(text), "%d%%", dutyPercent);
        tft.setTextSize(2);
        drawText(text, DUTY_CARD_X + 8, DUTY_CARD_Y + 20, 4, COLOR_TEXT, COLOR_CARD, 130);
        tft.setTextSize(1);

        // ค่าที่เขียนเข้า LEDC จริง
        snprintf(text, sizeof(text), "VALUE: %lu/%lu", static_cast<unsigned long>(pwmValue),
                 static_cast<unsigned long>(app::PWM_MAX_VALUE));
        drawText(text, DUTY_CARD_X + 162, DUTY_CARD_Y + 50, 2, COLOR_TEXT, COLOR_CARD, 138);

        // รูปคลื่น: ลบของเก่า (ด้วยสีการ์ด) แล้ววาดใหม่
        if (drawnWaveDuty >= 0) {
            drawWave(drawnWaveDuty, COLOR_CARD);
        }
        drawWave(dutyPercent, COLOR_GREEN);
        drawnWaveDuty = dutyPercent;
    }

    if (freqChanged) {
        snprintf(text, sizeof(text), "FREQ: %lu Hz", static_cast<unsigned long>(requestedFrequency()));
        drawText(text, DUTY_CARD_X + 162, DUTY_CARD_Y + 30, 2, COLOR_TEXT, COLOR_CARD, 138);
    }

    if (modeChanged) {
        snprintf(text, sizeof(text), "MODE: %s", modeName(mode));
        drawText(text, DUTY_CARD_X + 162, DUTY_CARD_Y + 10, 2, modeColor(mode), COLOR_CARD, 138);
    }

    if (dutyChanged || freqChanged) {
        // เวลา ON / OFF ในแต่ละรอบ และแรงดันเฉลี่ย (duty x 3.3 V)
        char onTime[16], offTime[16];
        formatMicros(onTime, sizeof(onTime), periodMicros() * dutyPercent / 100.0f);
        formatMicros(offTime, sizeof(offTime), periodMicros() * (100 - dutyPercent) / 100.0f);
        const float averageVolts = 3.3f * static_cast<float>(pwmValue) / static_cast<float>(app::PWM_MAX_VALUE);
        snprintf(text, sizeof(text), "ON %s OFF %s AVG %.2fV", onTime, offTime, averageVolts);
        drawText(text, WAVE_CARD_X + 8, WAVE_CARD_Y + 72, 2, COLOR_TEXT, COLOR_CARD, 288);
    }

    drawnDuty = dutyPercent;
    drawnFrequencyIndex = frequencyIndex;
    drawnMode = static_cast<int>(mode);
}

void updateHeartbeat()
{
    heartbeatOn = !heartbeatOn;
    tft.fillCircle(SCREEN_W - 16, FOOTER_Y + 8, 5, heartbeatOn ? COLOR_GREEN : COLOR_BORDER);
}

// ---------------------------------------------------------------------
// Calibrate เริ่ม / จบ
// ---------------------------------------------------------------------
void startCalibration()
{
    phase = Phase::Calibrating;
    phaseStartedAt = millis();
    calMin = static_cast<int>(lroundf(smoothedRaw));
    calMax = calMin;
    ledMirror = false;
    applyDuty(100);  // calibrate ตอนจอสว่างเต็มที่
    drawCalibrationScreen();

    Serial.println();
    Serial.printf("[CAL] cover the LDR for a moment within %lu s, then uncover it.\n",
                  static_cast<unsigned long>(app::CALIBRATION_MS / 1000));
}

void finishCalibration()
{
    const int range = calMax - calMin;
    Serial.printf("[CAL] raw min (brightest) = %d, raw max (darkest) = %d, range = %d\n", calMin, calMax, range);
    if (range >= app::MIN_CALIBRATION_RANGE) {
        rawBright = calMin;
        rawDark = calMax;
        Serial.println("[CAL] OK: using the range measured in this room.");
    } else {
        rawBright = app::DEFAULT_RAW_BRIGHT;
        rawDark = app::DEFAULT_RAW_DARK;
        Serial.printf("[CAL] range too small: using defaults bright=%d dark=%d (press EN/RESET to try again)\n",
                      rawBright, rawDark);
    }

    phase = Phase::Running;
    mode = Mode::Manual;
    ledMirror = true;
    applyDuty(app::DEFAULT_DUTY_PERCENT);
    drawMainScreenStatic();
    printHelp();
    printStatus("start");
}

}  // namespace

void setup()
{
    Serial.begin(app::SERIAL_BAUD);
    delay(200);

    Serial.println();
    Serial.println("==========================================");
    Serial.println("Hello World from ESP32-2432S028 (CYD)!");
    Serial.println("Level 1 / Lab 5 : PWM");
    Serial.println("==========================================");

    cyd::ledInit();
    pinMode(cyd::BOOT_BUTTON_PIN, INPUT_PULLUP);

    tft.init();  // TFT_eSPI เปิดไฟหลังจอ (GPIO21 = HIGH) ให้ตรงนี้
    tft.setRotation(cyd::DISPLAY_ROTATION);
    tft.setTextDatum(TL_DATUM);
    tft.fillScreen(COLOR_BG);

    analogSetPinAttenuation(cyd::LDR_PIN, ADC_0db);

    pwmBegin();  // หลังจากนี้ GPIO21 ถูกคุมด้วย PWM (ledcWrite) แทน digitalWrite

    updateLdr();
    startCalibration();
    lastSampleAt = millis();
}

void loop()
{
    const uint32_t now = millis();

    // 1) อ่านค่า LDR เป็นระยะ
    if (now - lastSampleAt >= app::SAMPLE_MS) {
        lastSampleAt = now;
        updateLdr();
        if (phase == Phase::Calibrating) {
            const int value = static_cast<int>(lroundf(smoothedRaw));
            if (value < calMin) calMin = value;
            if (value > calMax) calMax = value;
        }
    }

    if (phase == Phase::Calibrating) {
        if (now - lastDisplayAt >= 100) {
            lastDisplayAt = now;
            updateCalibrationScreen(now);
        }
        if (now - phaseStartedAt >= app::CALIBRATION_MS) {
            finishCalibration();
        }
        return;
    }

    // 2) ปุ่มและคำสั่งจาก Serial
    if (bootButtonPressed(now)) {
        nextMode();
    }
    handleSerialInput();

    // 3) โหมดอัตโนมัติคำนวณ duty
    if (now - lastPwmUpdateAt >= app::PWM_UPDATE_MS) {
        lastPwmUpdateAt = now;
        if (mode == Mode::Breathe) {
            applyDuty(breatheDuty(now));
        } else if (mode == Mode::Auto) {
            applyDuty(autoDuty());
        }
    }

    // 4) วาดจอ (เฉพาะเมื่อมีค่าเปลี่ยน และไม่ถี่เกินไป)
    if (now - lastDisplayAt >= app::DISPLAY_REFRESH_MS) {
        lastDisplayAt = now;
        updatePwmDisplay();
    }

    if (now - lastHeartbeatAt >= app::HEARTBEAT_MS) {
        lastHeartbeatAt = now;
        updateHeartbeat();
    }

    if (now - lastSerialAt >= app::SERIAL_PRINT_MS) {
        lastSerialAt = now;
        printStatus("     ");
    }
}
