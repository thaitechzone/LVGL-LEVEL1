// Level 1 / Lab 4 : TFT Display
//
// ใช้จอ TFT ของบอร์ด (320x240) แสดงผลงานจาก Lab 2-3 เป็นภาพ:
//   - ค่าแสงจาก LDR เป็นตัวเลข % + แถบกราฟ (พร้อมหน้าจอ calibrate ตอนเปิดเครื่อง)
//   - สถานะ WiFi, ชื่อเครือข่าย, IP และความแรงสัญญาณ (ไอคอนแท่ง)
//   - เวลาที่เปิดเครื่อง (uptime) และจุดกะพริบบอกว่าโปรแกรมยังทำงานอยู่
//   RGB LED ยังบอกสถานะ WiFi อยู่ (เขียว = ต่อแล้ว, น้ำเงิน = กำลังต่อ, แดง = มีปัญหา)
//
// แนวคิดที่ฝึกใน Lab นี้: พิกัดหน้าจอ, การวาดรูปทรง (วงกลม สี่เหลี่ยม เส้น), ข้อความและฟอนต์,
// สี RGB565, "วาดส่วนที่ไม่เปลี่ยนครั้งเดียว แล้วอัปเดตเฉพาะส่วนที่เปลี่ยน" (ไม่ล้างจอทั้งหน้า
// ทุกครั้ง จึงไม่กะพริบ), และการแยกหน้าจอเป็นหลายหน้า (calibrate / หน้าหลัก)

#include <Arduino.h>
#include <SPI.h>
#include <TFT_eSPI.h>
#include <WiFi.h>

#include "app_config.h"
#include "board_config.h"

// ถ้ายังไม่มีไฟล์ secrets.h โปรแกรมก็ยัง build ได้ แต่จะแจ้งเตือนบนจอและ Serial
#if __has_include("secrets.h")
#include "secrets.h"
#else
inline constexpr char WIFI_SSID[] = "";
inline constexpr char WIFI_PASSWORD[] = "";
#endif

namespace {

// ---------------------------------------------------------------------
// สี: จอใช้สี 16 บิต (RGB565) = แดง 5 บิต, เขียว 6 บิต, น้ำเงิน 5 บิต
// ฟังก์ชันนี้แปลงสี RGB ปกติ (0-255) เป็น RGB565
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

// ---------------------------------------------------------------------
// ตำแหน่งบนจอ (หน่วยพิกเซล มุมซ้ายบน = (0,0), กว้าง 320, สูง 240)
// ---------------------------------------------------------------------
constexpr int16_t SCREEN_W = 320;
constexpr int16_t HEADER_H = 28;

constexpr int16_t LIGHT_CARD_X = 8, LIGHT_CARD_Y = 34, LIGHT_CARD_W = 304, LIGHT_CARD_H = 92;
constexpr int16_t BAR_X = 16, BAR_Y = 110, BAR_W = 288, BAR_H = 12;

constexpr int16_t WIFI_CARD_X = 8, WIFI_CARD_Y = 130, WIFI_CARD_W = 304, WIFI_CARD_H = 84;
constexpr int16_t FOOTER_Y = 220;

enum class Phase { Calibrating, Running };
enum class WifiState : uint8_t { NotConfigured, Connecting, Connected, Failed };

TFT_eSPI tft;

Phase phase = Phase::Calibrating;
WifiState wifiState = WifiState::NotConfigured;
uint32_t wifiStateSince = 0;

// ค่าแสง
int lastRaw = 0;
float smoothedRaw = 0.0f;
bool smoothedReady = false;
int calMin = 4095;
int calMax = 0;
int rawBright = app::DEFAULT_RAW_BRIGHT;
int rawDark = app::DEFAULT_RAW_DARK;
bool usingDefaultRange = false;

// จับเวลา (millis)
uint32_t phaseStartedAt = 0;
uint32_t lastSampleAt = 0;
uint32_t lastLightDrawAt = 0;
uint32_t lastStatusDrawAt = 0;
uint32_t lastHeartbeatAt = 0;
uint32_t lastSerialAt = 0;
uint32_t lastBlinkAt = 0;
bool blinkOn = true;
bool heartbeatOn = true;
int lastDrawnPercent = -1;  // ค่า % ที่วาดไปล่าสุด (ไว้เช็คว่าเปลี่ยนไหม จะได้ไม่วาดซ้ำโดยไม่จำเป็น)

// ---------------------------------------------------------------------
// ตัวช่วยวาด
// ---------------------------------------------------------------------

// วาดข้อความที่ตำแหน่ง (x, y) มุมซ้ายบนของข้อความ
// paddingW = ความกว้างที่ต้องการให้ลบข้อความเก่าทิ้ง (ใช้ตอนอัปเดตค่าที่เปลี่ยนความยาวได้)
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

// ไอคอนความแรงสัญญาณ 4 แท่ง (activeBars = 0-4)
void drawSignalBars(int16_t x, int16_t y, int activeBars, uint16_t color)
{
    for (int i = 0; i < 4; i++) {
        const int16_t h = 5 + i * 4;
        tft.fillRect(x + i * 9, y + 17 - h, 6, h, (i < activeBars) ? color : COLOR_BORDER);
    }
}

int rssiToBars(int rssi)
{
    if (rssi >= -55) return 4;
    if (rssi >= -65) return 3;
    if (rssi >= -75) return 2;
    if (rssi >= -85) return 1;
    return 0;
}

// ---------------------------------------------------------------------
// ค่าแสง (ย่อมาจาก Lab 2: oversampling + smoothing + calibration)
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

// ความสว่าง 0-100%  (0 = มืดสุด, 100 = สว่างสุด)  บอร์ดนี้ raw สูง = มืด
int lightPercent()
{
    const long pct = map(lroundf(smoothedRaw), rawDark, rawBright, 0, 100);
    return static_cast<int>(constrain(pct, 0L, 100L));
}

const char *lightLabel(int percent)
{
    if (percent < app::LIGHT_DARK_BELOW) return "DARK";
    if (percent > app::LIGHT_BRIGHT_ABOVE) return "BRIGHT";
    return "NORMAL";
}

// สีของแถบ: มืด = น้ำเงิน, ปานกลาง = ส้ม, สว่าง = เหลือง (เหมือนแสงแดดที่แรงขึ้น)
uint16_t lightColor(int percent)
{
    if (percent < app::LIGHT_DARK_BELOW) return COLOR_BLUE;
    if (percent > app::LIGHT_BRIGHT_ABOVE) return COLOR_YELLOW;
    return COLOR_ORANGE;
}

// ---------------------------------------------------------------------
// WiFi (แบบย่อจาก Lab 3: ไม่สแกน ต่อตรงๆ แล้วลองใหม่เมื่อล้มเหลว)
// ---------------------------------------------------------------------

const char *wifiStateName(WifiState state)
{
    switch (state) {
        case WifiState::NotConfigured: return "NOT CONFIGURED";
        case WifiState::Connecting: return "CONNECTING...";
        case WifiState::Connected: return "CONNECTED";
        case WifiState::Failed: return "FAILED - will retry";
    }
    return "?";
}

uint16_t wifiStateColor(WifiState state)
{
    switch (state) {
        case WifiState::Connecting: return COLOR_BLUE;
        case WifiState::Connected: return COLOR_GREEN;
        case WifiState::NotConfigured:
        case WifiState::Failed: break;
    }
    return COLOR_RED;
}

// RGB LED สะท้อนสถานะ WiFi (ติดค้างสีเดียว เหมือนความหมายของ Lab 3)
void updateStatusLed()
{
    switch (wifiState) {
        case WifiState::Connected: cyd::ledSet(false, true, false); break;
        case WifiState::Connecting: cyd::ledSet(false, false, true); break;
        case WifiState::NotConfigured:
        case WifiState::Failed: cyd::ledSet(true, false, false); break;
    }
}

void setWifiState(WifiState next)
{
    Serial.printf("[WIFI] %s -> %s\n", wifiStateName(wifiState), wifiStateName(next));
    wifiState = next;
    wifiStateSince = millis();
    lastStatusDrawAt = 0;  // สั่งให้วาดส่วน WiFi ใหม่ทันที
    updateStatusLed();
}

void startWifiConnect()
{
    Serial.printf("[WIFI] connecting to \"%s\" ...\n", WIFI_SSID);
    WiFi.begin(WIFI_SSID, strlen(WIFI_PASSWORD) > 0 ? WIFI_PASSWORD : nullptr);
    setWifiState(WifiState::Connecting);
}

void updateWifi(uint32_t now)
{
    switch (wifiState) {
        case WifiState::NotConfigured:
            break;
        case WifiState::Connecting:
            if (WiFi.status() == WL_CONNECTED) {
                Serial.printf("[WIFI] connected, IP %s, RSSI %d dBm\n", WiFi.localIP().toString().c_str(), WiFi.RSSI());
                setWifiState(WifiState::Connected);
            } else if (now - wifiStateSince >= app::CONNECT_TIMEOUT_MS) {
                WiFi.disconnect(false);
                setWifiState(WifiState::Failed);
            }
            break;
        case WifiState::Connected:
            if (WiFi.status() != WL_CONNECTED) {
                Serial.println("[WIFI] connection lost");
                startWifiConnect();
            }
            break;
        case WifiState::Failed:
            if (now - wifiStateSince >= app::RETRY_DELAY_MS) {
                startWifiConnect();
            }
            break;
    }
}

// ---------------------------------------------------------------------
// หน้าจอที่ 1: Calibrate
// ---------------------------------------------------------------------

void drawCalibrationScreen()
{
    tft.fillScreen(COLOR_BG);
    drawHeader("Lab 4 - Light sensor calibration");

    tft.setTextDatum(TC_DATUM);  // จัดข้อความกึ่งกลางแนวนอนที่ x ที่ระบุ
    tft.setTextColor(COLOR_TEXT, COLOR_BG);
    tft.drawString("COVER the light sensor", SCREEN_W / 2, 52, 4);
    tft.drawString("with your hand for a moment,", SCREEN_W / 2, 86, 2);
    tft.drawString("then UNCOVER it.", SCREEN_W / 2, 106, 2);
    tft.setTextColor(COLOR_DIM, COLOR_BG);
    tft.drawString("Sensor = small part near the top-right edge (R21)", SCREEN_W / 2, 132, 2);
    tft.setTextDatum(TL_DATUM);

    tft.drawRect(30, 170, 260, 18, COLOR_BORDER);  // กรอบแถบความคืบหน้า
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
// หน้าจอที่ 2: หน้าหลัก
// ---------------------------------------------------------------------

// วาดส่วนที่ "ไม่เปลี่ยน" ครั้งเดียว (พื้นหลัง การ์ด ป้าย กรอบแถบ)
void drawMainScreenStatic()
{
    tft.fillScreen(COLOR_BG);
    drawHeader("Lab 4 - TFT Display");

    drawCard(LIGHT_CARD_X, LIGHT_CARD_Y, LIGHT_CARD_W, LIGHT_CARD_H, "LIGHT");
    tft.drawRect(BAR_X - 1, BAR_Y - 1, BAR_W + 2, BAR_H + 2, COLOR_BORDER);

    drawCard(WIFI_CARD_X, WIFI_CARD_Y, WIFI_CARD_W, WIFI_CARD_H, "WIFI");

    lastDrawnPercent = -1;  // บังคับให้วาดค่าแสงใหม่ในรอบถัดไป
    lastStatusDrawAt = 0;
}

// อัปเดตส่วนค่าแสง วาดใหม่เฉพาะเมื่อค่าเปลี่ยน
void updateLightCard()
{
    const int percent = lightPercent();
    if (percent == lastDrawnPercent) {
        return;
    }
    lastDrawnPercent = percent;
    const uint16_t color = lightColor(percent);

    // ตัวเลขเปอร์เซ็นต์ตัวใหญ่ (ฟอนต์ 4 ขยาย 2 เท่า)
    char text[12];
    snprintf(text, sizeof(text), "%d%%", percent);
    tft.setTextSize(2);
    drawText(text, LIGHT_CARD_X + 8, LIGHT_CARD_Y + 22, 4, COLOR_TEXT, COLOR_CARD, 130);
    tft.setTextSize(1);

    // คำบอกระดับ (DARK / NORMAL / BRIGHT) สีตามระดับ
    drawText(lightLabel(percent), LIGHT_CARD_X + 170, LIGHT_CARD_Y + 36, 4, color, COLOR_CARD, 120);

    // ค่า raw ตัวเล็กมุมขวาบน
    snprintf(text, sizeof(text), "raw %d", lastRaw);
    drawText(text, LIGHT_CARD_X + 210, LIGHT_CARD_Y + 4, 2, COLOR_DIM, COLOR_CARD, 80);

    // แถบ: ส่วนที่เติมสี + ส่วนที่ว่าง
    const int16_t filled = static_cast<int16_t>((static_cast<long>(percent) * BAR_W) / 100);
    tft.fillRect(BAR_X, BAR_Y, filled, BAR_H, color);
    tft.fillRect(BAR_X + filled, BAR_Y, BAR_W - filled, BAR_H, COLOR_BG);
}

// อัปเดตส่วน WiFi และ uptime (ทุก STATUS_REFRESH_MS หรือเมื่อสถานะเปลี่ยน)
void updateStatusCards(uint32_t now)
{
    const int16_t x = WIFI_CARD_X + 8;
    const int16_t y = WIFI_CARD_Y;
    const uint16_t stateColor = wifiStateColor(wifiState);
    char text[48];

    // จุดสี + ชื่อสถานะ
    tft.fillCircle(x + 6, y + 30, 6, stateColor);
    drawText(wifiStateName(wifiState), x + 20, y + 22, 2, stateColor, COLOR_CARD, 200);

    // ชื่อ WiFi
    if (strlen(WIFI_SSID) > 0) {
        snprintf(text, sizeof(text), "SSID: %.24s", WIFI_SSID);
    } else {
        snprintf(text, sizeof(text), "Set WIFI_SSID in include/secrets.h");
    }
    drawText(text, x, y + 42, 2, COLOR_TEXT, COLOR_CARD, 280);

    // IP และความแรงสัญญาณ
    int bars = 0;
    if (wifiState == WifiState::Connected) {
        const int rssi = WiFi.RSSI();
        bars = rssiToBars(rssi);
        snprintf(text, sizeof(text), "IP: %s   %d dBm", WiFi.localIP().toString().c_str(), rssi);
    } else {
        snprintf(text, sizeof(text), "IP: --");
    }
    drawText(text, x, y + 62, 2, COLOR_TEXT, COLOR_CARD, 280);
    drawSignalBars(WIFI_CARD_X + WIFI_CARD_W - 50, y + 6, bars, stateColor);

    // uptime ที่ท้ายจอ
    const uint32_t seconds = now / 1000;
    snprintf(text, sizeof(text), "uptime %02lu:%02lu:%02lu", static_cast<unsigned long>(seconds / 3600),
             static_cast<unsigned long>((seconds / 60) % 60), static_cast<unsigned long>(seconds % 60));
    drawText(text, 8, FOOTER_Y, 2, COLOR_DIM, COLOR_BG, 150);
}

// จุดกะพริบมุมล่างขวา: ถ้ากะพริบแปลว่า loop() ยังวิ่งอยู่
void updateHeartbeat()
{
    heartbeatOn = !heartbeatOn;
    tft.fillCircle(SCREEN_W - 16, FOOTER_Y + 8, 5, heartbeatOn ? COLOR_GREEN : COLOR_BORDER);
}

// ---------------------------------------------------------------------
// Calibration (ย่อมาจาก Lab 2)
// ---------------------------------------------------------------------

void startCalibration()
{
    phase = Phase::Calibrating;
    phaseStartedAt = millis();
    calMin = static_cast<int>(lroundf(smoothedRaw));
    calMax = calMin;
    lastBlinkAt = phaseStartedAt;
    blinkOn = true;
    cyd::ledSet(true, true, true);
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
        usingDefaultRange = false;
        Serial.println("[CAL] OK: using the range measured in this room.");
    } else {
        rawBright = app::DEFAULT_RAW_BRIGHT;
        rawDark = app::DEFAULT_RAW_DARK;
        usingDefaultRange = true;
        Serial.printf("[CAL] range too small: using defaults bright=%d dark=%d (press EN/RESET to try again)\n",
                      rawBright, rawDark);
    }

    phase = Phase::Running;
    drawMainScreenStatic();
    updateStatusLed();
}

}  // namespace

void setup()
{
    Serial.begin(app::SERIAL_BAUD);
    delay(200);

    Serial.println();
    Serial.println("==========================================");
    Serial.println("Hello World from ESP32-2432S028 (CYD)!");
    Serial.println("Level 1 / Lab 4 : TFT Display");
    Serial.println("==========================================");

    cyd::ledInit();
    cyd::enableBacklight();  // เปิดไฟหลังจอ (TFT_eSPI ก็เปิดให้ตอน tft.init() เพราะกำหนด TFT_BL ไว้ สั่งซ้ำให้ชัดเจน)

    tft.init();
    tft.setRotation(cyd::DISPLAY_ROTATION);  // 1 = แนวนอน 320x240
    tft.setTextDatum(TL_DATUM);              // พิกัดข้อความ = มุมซ้ายบนของข้อความ
    tft.fillScreen(COLOR_BG);

    // LDR: ใช้ช่วงวัดแคบ (0 dB) เหมือน Lab 2 เพื่อแยกมืด-สว่างได้ชัด
    analogSetPinAttenuation(cyd::LDR_PIN, ADC_0db);

    WiFi.persistent(false);
    WiFi.mode(WIFI_STA);
    WiFi.setAutoReconnect(false);

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
        // 2a) หน้าจอ calibrate: LED ขาวกะพริบ + แถบความคืบหน้า
        if (now - lastBlinkAt >= 250) {
            lastBlinkAt = now;
            blinkOn = !blinkOn;
            cyd::ledSet(blinkOn, blinkOn, blinkOn);
        }
        if (now - lastLightDrawAt >= app::LIGHT_REFRESH_MS) {
            lastLightDrawAt = now;
            updateCalibrationScreen(now);
        }
        if (now - phaseStartedAt >= app::CALIBRATION_MS) {
            finishCalibration();
            // เริ่มต่อ WiFi หลังจาก calibrate เสร็จ (ถ้าตั้งค่า SSID ไว้แล้ว)
            if (strlen(WIFI_SSID) > 0) {
                startWifiConnect();
            } else {
                setWifiState(WifiState::NotConfigured);
            }
        }
        return;
    }

    // 2b) หน้าหลัก
    updateWifi(now);

    if (now - lastLightDrawAt >= app::LIGHT_REFRESH_MS) {
        lastLightDrawAt = now;
        updateLightCard();
    }

    if (now - lastStatusDrawAt >= app::STATUS_REFRESH_MS) {
        lastStatusDrawAt = now;
        updateStatusCards(now);
    }

    if (now - lastHeartbeatAt >= app::HEARTBEAT_MS) {
        lastHeartbeatAt = now;
        updateHeartbeat();
    }

    if (now - lastSerialAt >= app::SERIAL_PRINT_MS) {
        lastSerialAt = now;
        Serial.printf("[LDR] raw=%4d smooth=%6.1f light=%3d%%%s   [WIFI] %s\n", lastRaw, smoothedRaw, lightPercent(),
                      usingDefaultRange ? " (default range)" : "", wifiStateName(wifiState));
    }
}
