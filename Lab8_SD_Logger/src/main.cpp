// Level 1 / Lab 8 : SD Logger
//
// ต่อยอดจาก Lab 7 (NTP Clock): บันทึกข้อมูลลงไฟล์ CSV บน microSD card พร้อม "เวลาจริง" จาก NTP
//   - ทุก LOG_INTERVAL_MS (ค่าเริ่มต้น 10 วินาที) บันทึก 1 แถว: เวลาจริง, uptime, REAL/SIM, อุณหภูมิ, ความชื้น, พัดลม
//   - ทุกครั้งที่เปิดเครื่องสร้างไฟล์ใหม่ log_0001.csv, log_0002.csv ... ไม่เขียนทับไฟล์เก่า
//   - ยังไม่ sync เวลา -> บันทึกคำว่า NOT_SYNCED (ไม่ใส่เวลาปี 1970) คอลัมน์ source บอกว่าค่าเป็น REAL หรือ SIM
//   - ไม่มีการ์ด / การ์ดผิดพลาด -> โปรแกรมไม่ค้าง ทุกอย่างอื่นทำงานต่อ แล้วลองเมาท์ใหม่เป็นระยะ
//   - เปิด-เขียน-ปิดไฟล์ทุกครั้งที่บันทึก เพื่อให้ข้อมูลไม่เสียหายเมื่อไฟดับ
//   - คำสั่ง e = หยุดบันทึกและถอดการ์ดได้อย่างปลอดภัย
//
// ขาที่ใช้ (ค่าจริงอยู่ใน include/board_config.h):
//   microSD         = CS GPIO5, MOSI GPIO23, SCK GPIO18, MISO GPIO19  (SPI ชุดแยกจากจอ ใช้ VSPI)
//   DHT11 DATA      = GPIO27  (ห้ามใช้ GPIO21 เพราะเป็นขาไฟหลังจอ)
//   LED น้ำเงิน (B) = GPIO17  -> PWM (LEDC ช่อง 0) เป็นพัดลมจำลอง
//   LED แดง   (R)   = GPIO4   -> digitalWrite บอกว่าเซนเซอร์มีปัญหา
//   LED เขียว (G)   = GPIO16  -> digitalWrite ติด = sync เวลาจาก NTP สำเร็จแล้ว
//   LED เป็น active-LOW (ส่ง LOW = ติด) จึงกลับค่า duty ก่อนเขียน
//   ปุ่ม BOOT       = GPIO0   (สลับ AUTO / MANUAL)
//
// คำสั่ง Serial:  m = AUTO/MANUAL, + - 0-9 = พัดลม (MANUAL), s = แหล่งข้อมูล REAL/SIM, n = ขอเวลา NTP ใหม่, t = เวลาตอนนี้
//                i = ข้อมูลการ์ด, l = รายชื่อไฟล์, p = แสดงท้ายไฟล์ log, e = ถอดการ์ดอย่างปลอดภัย, r = เมาท์ใหม่/เริ่มบันทึกต่อ,
//                x แล้ว y = ลบไฟล์ log_####.csv ทั้งหมดของ Lab นี้, h = วิธีใช้

#include <Arduino.h>
#include <DHT.h>
#include <FS.h>
#include <SD.h>
#include <SPI.h>
#include <TFT_eSPI.h>
#include <WiFi.h>
#include <esp_sntp.h>
#include <time.h>

#include "app_config.h"
#include "board_config.h"

// ถ้ายังไม่มีไฟล์ secrets.h โปรแกรมก็ยัง build ได้ แต่จะไม่มีเวลาจริง (ไฟล์ log ยังบันทึกได้ โดยเวลาเป็น NOT_SYNCED)
#if __has_include("secrets.h")
#include "secrets.h"
#else
inline constexpr char WIFI_SSID[] = "";
inline constexpr char WIFI_PASSWORD[] = "";
#endif

namespace {

// ---------------------------------------------------------------------
// สีและตำแหน่งบนจอ
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

// การ์ดนาฬิกา / อุณหภูมิ-ความชื้น / พัดลม (บรรทัดเดียว) / SD card (สองบรรทัด)
constexpr int16_t CLOCK_CARD_X = 8, CLOCK_CARD_Y = 32, CLOCK_CARD_W = 304, CLOCK_CARD_H = 62;
constexpr int16_t TEMP_CARD_X = 8, TEMP_CARD_Y = 98, TEMP_CARD_W = 148, TEMP_CARD_H = 50;
constexpr int16_t HUM_CARD_X = 164, HUM_CARD_Y = 98, HUM_CARD_W = 148, HUM_CARD_H = 50;
constexpr int16_t FAN_CARD_X = 8, FAN_CARD_Y = 152, FAN_CARD_W = 304, FAN_CARD_H = 26;
constexpr int16_t SD_CARD_X = 8, SD_CARD_Y = 181, SD_CARD_W = 304, SD_CARD_H = 47;

enum class Mode : uint8_t { Auto, Manual };
enum class WifiState : uint8_t { NotConfigured, Connecting, Connected, Failed };
enum class NtpState : uint8_t { Idle, Syncing, Synced, Failed };
// สถานะ SD card: NoCard = ไม่พบการ์ด/เมาท์ไม่สำเร็จ, Ready = บันทึกได้, WriteError = เขียนไม่สำเร็จ, Ejected = สั่งหยุดแล้ว (ถอดได้)
enum class SdState : uint8_t { NoCard, Ready, WriteError, Ejected };

TFT_eSPI tft;
DHT dht(cyd::DHT11_PIN, DHT11);
SPIClass sdSpi(VSPI);  // SD card ใช้ SPI ชุดแยก (จอ TFT ใช้ HSPI)

Mode mode = Mode::Auto;

// ---- WiFi / NTP ----
WifiState wifiState = WifiState::NotConfigured;
uint32_t wifiStateSince = 0;
NtpState ntpState = NtpState::Idle;
uint32_t ntpStartedAt = 0;
uint32_t syncCount = 0;         // sync สำเร็จมาแล้วกี่ครั้ง
uint32_t lastSyncAtMs = 0;      // sync ล่าสุดตอนที่ millis() เท่าไร
volatile bool syncEvent = false;  // callback ของ SNTP ตั้งค่านี้ แล้ว loop() มาจัดการต่อ (callback ทำงานคนละ task)

// ---- SD card และไฟล์ log ----
SdState sdState = SdState::NoCard;
uint32_t sdLastTryAt = 0;       // ลองเมาท์ครั้งล่าสุดเมื่อไร
bool sdReportedNoCard = false;  // พิมพ์เตือน "ไม่พบการ์ด" ไปแล้วหรือยัง (กันพิมพ์ซ้ำทุกรอบ)
uint8_t sdCardType = CARD_NONE;
uint64_t sdCardBytes = 0;
char logPath[24] = "";          // เช่น "/log_0003.csv"
uint32_t logRows = 0;           // แถวที่บันทึกสำเร็จในไฟล์ปัจจุบัน
uint32_t logSkipped = 0;        // แถวที่ถึงเวลาบันทึกแต่บันทึกไม่ได้ (ไม่มีการ์ด/มีข้อผิดพลาด)
uint32_t logFailures = 0;       // ครั้งที่เขียนไม่สำเร็จ
uint32_t lastWriteMicros = 0;   // การบันทึกครั้งล่าสุดใช้เวลากี่ µs
uint32_t maxWriteMicros = 0;
bool deletePending = false;     // รอยืนยันคำสั่งลบไฟล์ (x แล้ว y)
uint32_t deleteAskedAt = 0;

// ---- ผลการอ่าน DHT11 ----
float sensorTempC = NAN;       // ค่าล่าสุดที่อ่านสำเร็จ
float sensorHumidity = NAN;
bool haveReading = false;      // เคยอ่านสำเร็จอย่างน้อย 1 ครั้งหรือยัง
uint8_t consecutiveFails = 0;
bool sensorError = false;      // อ่านพลาดติดกัน >= DHT_FAIL_LIMIT
uint32_t readCount = 0;
uint32_t okCount = 0;
uint32_t failCount = 0;
uint32_t lastReadMicros = 0;   // อ่านครั้งล่าสุดใช้เวลากี่ µs (วัดจริง)

// ---- ค่าจำลอง (SIM) ----
// simChoice = สิ่งที่ผู้ใช้เลือกด้วย s: Off = ใช้เซนเซอร์จริง (และ fallback เป็น SIM เมื่อเซนเซอร์เสียถ้าเปิดไว้)
enum class SimChoice : uint8_t { Off, Random, Sweep };
// DataSource = แหล่งที่มาของค่าที่ "ใช้อยู่จริง" ขณะนี้ (ใช้แสดงป้าย REAL / SIM และบันทึกในคอลัมน์ source)
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
bool syncLedOn = false;

// ---- จับเวลา ----
uint32_t lastReadAt = 0;
uint32_t lastFanAt = 0;
uint32_t lastDisplayAt = 0;
uint32_t lastStatusAt = 0;
uint32_t lastHeartbeatAt = 0;
uint32_t lastSerialAt = 0;
uint32_t lastTimePrintAt = 0;
uint32_t lastLogAt = 0;
bool heartbeatOn = true;

// สิ่งที่วาดบนจอไปล่าสุด (ไว้เทียบว่าเปลี่ยนไหม จะได้วาดเฉพาะส่วนที่เปลี่ยน)
constexpr int NEVER = -32768;
int drawnTimeKey = NEVER;
int drawnDateKey = NEVER;
int drawnBadgeKey = NEVER;
int drawnSyncCount = NEVER;
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
    if (label[0] != '\0') {
        drawText(label, x + 8, y + 4, 2, COLOR_DIM, COLOR_CARD);
    }
}

// ---------------------------------------------------------------------
// เวลาจริง (NTP) เหมือน Lab 7
// ---------------------------------------------------------------------

// callback ที่ไลบรารี SNTP เรียกทุกครั้งที่ได้เวลาจากเซิร์ฟเวอร์ (ทำงานใน task ของเครือข่าย จึงแค่ตั้งธงไว้)
void onTimeSync(struct timeval *tv)
{
    (void)tv;
    syncEvent = true;
}

// เวลาที่ใช้ได้จริงไหม: เคย sync สำเร็จแล้ว และเวลาในเครื่องไม่ใช่ค่าเริ่มต้น (ปี 1970)
bool timeValid()
{
    return syncCount > 0 && static_cast<uint32_t>(time(nullptr)) >= app::SANE_EPOCH;
}

// sync ล่าสุดเก่าเกินไป (ถึงแม้เคย sync แล้ว) = นาฬิกาอาจคลาดเคลื่อน
bool timeStale(uint32_t now)
{
    return ntpState == NtpState::Synced && (now - lastSyncAtMs) > app::NTP_RESYNC_MS * app::NTP_STALE_FACTOR;
}

// จัดรูปแบบเวลาท้องถิ่น (UTC+7) ตามรูปแบบ strftime
void formatLocalTime(char *out, size_t size, const char *format)
{
    const time_t t = time(nullptr);
    struct tm local;
    localtime_r(&t, &local);
    strftime(out, size, format, &local);
}

// ติดเวลาจริงให้ข้อมูล "2026-10-09 14:35:08" ถ้ายังไม่ sync คืน "NOT_SYNCED" (ใช้กับทุกแถวของไฟล์ log)
bool makeTimestamp(char *out, size_t size)
{
    if (!timeValid()) {
        snprintf(out, size, "NOT_SYNCED");
        return false;
    }
    formatLocalTime(out, size, "%Y-%m-%d %H:%M:%S");
    return true;
}

// "12s" / "5m" / "2h05m" : เวลาที่ผ่านมาแบบสั้นๆ
void formatAgo(char *out, size_t size, uint32_t ms)
{
    const uint32_t seconds = ms / 1000;
    if (seconds < 60) {
        snprintf(out, size, "%lus", static_cast<unsigned long>(seconds));
    } else if (seconds < 3600) {
        snprintf(out, size, "%lum", static_cast<unsigned long>(seconds / 60));
    } else {
        snprintf(out, size, "%luh%02lum", static_cast<unsigned long>(seconds / 3600),
                 static_cast<unsigned long>((seconds / 60) % 60));
    }
}

const char *ntpStateName(NtpState state)
{
    switch (state) {
        case NtpState::Idle: return "IDLE";
        case NtpState::Syncing: return "SYNCING";
        case NtpState::Synced: return "SYNCED";
        case NtpState::Failed: return "FAILED";
    }
    return "?";
}

void startNtp(uint32_t now)
{
    configTime(app::GMT_OFFSET_SEC, app::DAYLIGHT_OFFSET_SEC, app::NTP_SERVER_1, app::NTP_SERVER_2);
    ntpState = NtpState::Syncing;
    ntpStartedAt = now;
    Serial.printf("[NTP] requesting time from %s / %s ...\n", app::NTP_SERVER_1, app::NTP_SERVER_2);
}

void updateNtp(uint32_t now)
{
    if (syncEvent) {  // ได้รับเวลาจากเซิร์ฟเวอร์แล้ว
        syncEvent = false;
        syncCount++;
        lastSyncAtMs = now;
        ntpState = NtpState::Synced;
        char stamp[24];
        makeTimestamp(stamp, sizeof(stamp));
        Serial.printf("[NTP] synced #%lu: %s (UTC+%ld)\n", static_cast<unsigned long>(syncCount), stamp,
                      app::GMT_OFFSET_SEC / 3600);
    }

    if (wifiState != WifiState::Connected) {
        if (ntpState == NtpState::Syncing) {
            ntpState = NtpState::Idle;  // WiFi หลุดระหว่างรอ -> กลับไปรอ WiFi
        }
        return;
    }

    if (ntpState == NtpState::Idle) {
        startNtp(now);
    } else if (ntpState == NtpState::Syncing && now - ntpStartedAt >= app::NTP_TIMEOUT_MS) {
        ntpState = NtpState::Failed;
        Serial.printf("[NTP] no reply within %lu s (UDP port 123 blocked? DNS problem?) - will retry\n",
                      static_cast<unsigned long>(app::NTP_TIMEOUT_MS / 1000));
    } else if (ntpState == NtpState::Failed && now - ntpStartedAt >= app::NTP_RETRY_MS) {
        startNtp(now);
    }
}

// ---------------------------------------------------------------------
// WiFi (แบบย่อจาก Lab 3: ไม่สแกน ต่อตรงๆ แล้วลองใหม่เมื่อล้มเหลว)
// ---------------------------------------------------------------------
const char *wifiStateName(WifiState state)
{
    switch (state) {
        case WifiState::NotConfigured: return "NOT CONFIGURED";
        case WifiState::Connecting: return "CONNECTING";
        case WifiState::Connected: return "CONNECTED";
        case WifiState::Failed: return "FAILED";
    }
    return "?";
}

void setWifiState(WifiState next)
{
    Serial.printf("[WIFI] %s -> %s\n", wifiStateName(wifiState), wifiStateName(next));
    wifiState = next;
    wifiStateSince = millis();
}

void startWifiConnect()
{
    Serial.println("[WIFI] connecting ...");  // ไม่พิมพ์ชื่อ WiFi เพื่อไม่ให้หลุดเวลาแชร์ log
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
                Serial.printf("[WIFI] connected, RSSI %d dBm\n", WiFi.RSSI());
                setWifiState(WifiState::Connected);
            } else if (now - wifiStateSince >= app::CONNECT_TIMEOUT_MS) {
                WiFi.disconnect(false);
                setWifiState(WifiState::Failed);
            }
            break;
        case WifiState::Connected:
            if (WiFi.status() != WL_CONNECTED) {
                Serial.println("[WIFI] connection lost (the clock and the log keep running)");
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
// อุณหภูมิที่ "ใช้งาน" (จากเซนเซอร์จริง หรือจำลอง) เหมือน Lab 6-7
// ---------------------------------------------------------------------

// SIM sweep: คลื่นสามเหลี่ยม 0..1..0 ทุก SIM_PERIOD_MS
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
// พัดลมจำลอง = PWM บน LED น้ำเงิน (GPIO17) เหมือน Lab 6-7
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

// LED แดง (GPIO4) = เซนเซอร์มีปัญหา เปิด/ปิดด้วย digitalWrite ปกติ (ไม่ใช่ PWM)
void setErrorLed(bool on)
{
    if (on == errorLedOn) return;
    errorLedOn = on;
    // active-LOW: ติด = LOW, ดับ = HIGH
    digitalWrite(cyd::LED_PIN_R, (on == cyd::LED_ACTIVE_LOW) ? LOW : HIGH);
}

// LED เขียว (GPIO16) = เวลาจาก NTP sync สำเร็จแล้ว เปิด/ปิดด้วย digitalWrite ปกติ
void setSyncLed(bool on)
{
    if (on == syncLedOn) return;
    syncLedOn = on;
    digitalWrite(cyd::LED_PIN_G, (on == cyd::LED_ACTIVE_LOW) ? LOW : HIGH);
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
        fanOn = false;  // ไม่มีค่าที่เชื่อถือได้ -> หยุดพัดลม
    } else if (!fanOn && tempC >= app::FAN_ON_TEMP_C) {
        fanOn = true;
    } else if (fanOn && tempC <= app::FAN_OFF_TEMP_C) {
        fanOn = false;
    }
    // ระหว่าง FAN_OFF_TEMP_C กับ FAN_ON_TEMP_C: คงสถานะเดิมไว้ นี่คือ hysteresis

    applyFanDuty((fanOn && !isnan(tempC)) ? dutyForTemp(tempC) : 0);

    if (wasOn != fanOn) {
        char stamp[24];
        makeTimestamp(stamp, sizeof(stamp));
        if (isnan(tempC)) {
            Serial.printf("[FAN] %s -> %s (no valid temperature)  @ %s\n", wasOn ? "ON" : "OFF", fanOn ? "ON" : "OFF", stamp);
        } else {
            Serial.printf("[FAN] %s -> %s at %.1f C  @ %s\n", wasOn ? "ON" : "OFF", fanOn ? "ON" : "OFF", tempC, stamp);
        }
    }
}

// ---------------------------------------------------------------------
// อ่าน DHT11
// ---------------------------------------------------------------------
void readDht()
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
// SD card และไฟล์ log
// ---------------------------------------------------------------------
const char *sdStateName(SdState state)
{
    switch (state) {
        case SdState::NoCard: return "NO CARD";
        case SdState::Ready: return "READY";
        case SdState::WriteError: return "ERROR";
        case SdState::Ejected: return "EJECTED";
    }
    return "?";
}

const char *sdTypeName(uint8_t type)
{
    switch (type) {
        case CARD_MMC: return "MMC";
        case CARD_SD: return "SDSC";
        case CARD_SDHC: return "SDHC";
        default: return "?";
    }
}

// เอาเฉพาะชื่อไฟล์ (ตัด "/" ข้างหน้า) เพราะไลบรารีบางรุ่นคืนชื่อพร้อมพาธ
const char *baseName(const char *path)
{
    const char *slash = strrchr(path, '/');
    return slash ? slash + 1 : path;
}

// ชื่อไฟล์ log ที่โปรแกรมนี้สร้าง ต้องเป็นรูปแบบ log_####.csv เท่านั้น (ใช้กรองก่อนลบ เพื่อไม่แตะไฟล์อื่นในการ์ด)
bool isLogFileName(const char *name, int *number)
{
    int n = 0;
    char tail = 0;
    if (strlen(name) == 12 && sscanf(name, "log_%4d.csv%c", &n, &tail) == 1) {
        if (number) *number = n;
        return true;
    }
    return false;
}

// หาเลขไฟล์ log ที่ใหญ่สุดที่มีอยู่ในการ์ด (เพื่อสร้างไฟล์ใหม่เลขถัดไป ไม่เขียนทับของเก่า)
int highestLogNumber()
{
    int highest = 0;
    File root = SD.open("/");
    if (!root) return 0;
    File entry = root.openNextFile();
    while (entry) {
        int n = 0;
        if (!entry.isDirectory() && isLogFileName(baseName(entry.name()), &n) && n > highest) {
            highest = n;
        }
        entry.close();
        entry = root.openNextFile();
    }
    root.close();
    return highest;
}

// สร้างไฟล์ log ใหม่ (เลขถัดไป) พร้อมบรรทัดหัวตาราง CSV
bool createLogFile()
{
    const int number = highestLogNumber() + 1;
    snprintf(logPath, sizeof(logPath), "/log_%04d.csv", number);
    File file = SD.open(logPath, FILE_WRITE);
    if (!file) {
        Serial.printf("[SD] cannot create %s\n", logPath);
        logPath[0] = '\0';
        return false;
    }
    file.println("timestamp,uptime_s,source,temp_c,hum_pct,fan_duty_pct,mode");
    file.close();
    logRows = 0;
    Serial.printf("[SD] created %s\n", logPath);
    return true;
}

// เมาท์การ์ด แล้วสร้างไฟล์ log ใหม่ (คืน true เมื่อพร้อมบันทึก)
bool mountSd()
{
    const uint32_t startedAt = millis();
    // ไม่ฟอร์แมตการ์ดเด็ดขาด (พารามิเตอร์ format_if_empty เป็น false โดยปริยาย)
    const bool mounted = SD.begin(cyd::SD_PIN_CS, sdSpi, app::SD_SPI_HZ);
    sdCardType = mounted ? SD.cardType() : CARD_NONE;
    if (!mounted || sdCardType == CARD_NONE) {
        if (mounted) SD.end();
        sdState = SdState::NoCard;
        if (!sdReportedNoCard) {
            Serial.printf("[SD] no card found (insert a FAT32 microSD card) - retrying every %lu s  (mount attempt took %lu ms)\n",
                          static_cast<unsigned long>(app::SD_RETRY_MS / 1000), static_cast<unsigned long>(millis() - startedAt));
            sdReportedNoCard = true;
        }
        return false;
    }

    sdReportedNoCard = false;
    sdCardBytes = SD.cardSize();
    Serial.printf("[SD] card OK: type %s, size %.1f GB, used %.2f MB  (mount took %lu ms)\n", sdTypeName(sdCardType),
                  static_cast<double>(sdCardBytes) / 1073741824.0, static_cast<double>(SD.usedBytes()) / 1048576.0,
                  static_cast<unsigned long>(millis() - startedAt));
    if (!createLogFile()) {
        SD.end();
        sdState = SdState::WriteError;
        return false;
    }
    sdState = SdState::Ready;
    return true;
}

// บันทึก 1 แถว: เปิดไฟล์แบบต่อท้าย -> เขียน -> ปิด (ปิดทุกครั้งเพื่อความปลอดภัยของข้อมูลเมื่อไฟดับ)
void appendLogRow(uint32_t now)
{
    const DataSource source = currentSource();
    const float tempC = effectiveTemp(now);
    const float humidity = effectiveHumidity(now);
    if (isnan(tempC) || isnan(humidity)) {
        return;  // ยังไม่มีค่าที่ใช้ได้ ไม่บันทึกแถวว่าง
    }
    if (sdState != SdState::Ready) {
        logSkipped++;
        return;
    }

    char stamp[24];
    makeTimestamp(stamp, sizeof(stamp));  // ยังไม่ sync -> "NOT_SYNCED" (ไม่ใส่เวลาปี 1970)
    char line[96];
    snprintf(line, sizeof(line), "%s,%lu,%s,%.1f,%.0f,%d,%s", stamp, static_cast<unsigned long>(now / 1000), sourceName(source),
             tempC, humidity, fanDuty, mode == Mode::Auto ? "AUTO" : "MANUAL");

    const uint32_t startedAt = micros();
    File file = SD.open(logPath, FILE_APPEND);
    bool ok = static_cast<bool>(file);
    if (ok) {
        ok = file.println(line) > 0;
        file.close();
    }
    lastWriteMicros = micros() - startedAt;
    if (lastWriteMicros > maxWriteMicros) maxWriteMicros = lastWriteMicros;

    if (ok) {
        logRows++;
        Serial.printf("[LOG] #%lu %s  (write took %lu us)\n", static_cast<unsigned long>(logRows), line,
                      static_cast<unsigned long>(lastWriteMicros));
    } else {
        logFailures++;
        logSkipped++;
        sdState = SdState::WriteError;
        SD.end();  // ปล่อยการ์ด แล้วให้ลูปลองเมาท์ใหม่ (เช่นถอดการ์ดโดยไม่ได้สั่ง e)
        sdLastTryAt = now;
        Serial.println("[SD] write FAILED (card removed or full?) - logging paused, will try to remount");
    }
}

// ลองเมาท์ใหม่เป็นระยะเมื่อไม่มีการ์ด/มีข้อผิดพลาด (ไม่ลองเมื่อผู้ใช้สั่งถอดแล้ว)
void updateSd(uint32_t now)
{
    if (sdState == SdState::NoCard || sdState == SdState::WriteError) {
        if (now - sdLastTryAt >= app::SD_RETRY_MS) {
            sdLastTryAt = now;
            mountSd();
        }
    }
}

void printSdInfo()
{
    Serial.printf("[SD] state=%s", sdStateName(sdState));
    if (sdState == SdState::Ready) {
        const double total = static_cast<double>(SD.totalBytes()) / 1048576.0;
        const double used = static_cast<double>(SD.usedBytes()) / 1048576.0;
        Serial.printf("  type=%s card=%.1f GB  fs total=%.0f MB used=%.2f MB free=%.0f MB", sdTypeName(sdCardType),
                      static_cast<double>(sdCardBytes) / 1073741824.0, total, used, total - used);
    }
    Serial.printf("\n[SD] file=%s rows=%lu skipped=%lu write failures=%lu  last write=%lu us (max %lu us)\n",
                  logPath[0] ? logPath : "-", static_cast<unsigned long>(logRows), static_cast<unsigned long>(logSkipped),
                  static_cast<unsigned long>(logFailures), static_cast<unsigned long>(lastWriteMicros),
                  static_cast<unsigned long>(maxWriteMicros));
}

void listSdFiles()
{
    if (sdState != SdState::Ready) {
        Serial.println("[SD] card is not ready");
        return;
    }
    File root = SD.open("/");
    if (!root) {
        Serial.println("[SD] cannot open the root folder");
        return;
    }
    Serial.println("[SD] files in the card root:");
    int count = 0;
    File entry = root.openNextFile();
    while (entry) {
        Serial.printf("[SD]   %-24s %s %lu bytes\n", baseName(entry.name()), entry.isDirectory() ? "<dir>" : "     ",
                      static_cast<unsigned long>(entry.size()));
        count++;
        entry.close();
        entry = root.openNextFile();
    }
    root.close();
    Serial.printf("[SD] %d item(s)\n", count);
}

// แสดงท้ายไฟล์ log ปัจจุบัน (อ่านย้อนหลังประมาณ 360 ไบต์)
void printLogTail()
{
    if (sdState != SdState::Ready || logPath[0] == '\0') {
        Serial.println("[SD] no log file is open");
        return;
    }
    File file = SD.open(logPath, FILE_READ);
    if (!file) {
        Serial.println("[SD] cannot open the log file");
        return;
    }
    const size_t size = file.size();
    const size_t window = 360;
    const size_t start = size > window ? size - window : 0;
    file.seek(start);
    char buffer[window + 1];
    const size_t got = file.readBytes(buffer, window);
    buffer[got] = '\0';
    file.close();

    const char *text = buffer;
    if (start > 0) {  // เริ่มกลางบรรทัด -> ข้ามไปบรรทัดถัดไป
        const char *newline = strchr(buffer, '\n');
        text = newline ? newline + 1 : buffer;
    }
    Serial.printf("[SD] tail of %s (%lu bytes):\n%s", logPath, static_cast<unsigned long>(size), text);
}

// สั่งหยุดบันทึกและปล่อยการ์ด เพื่อให้ถอดออกได้อย่างปลอดภัย
void ejectSd()
{
    if (sdState == SdState::Ready) {
        SD.end();
    }
    sdState = SdState::Ejected;
    Serial.println("[SD] ejected: logging stopped, it is safe to remove the card.  (r = remount and resume logging)");
}

void remountSd()
{
    if (sdState == SdState::Ready) {
        SD.end();
    }
    sdReportedNoCard = false;
    sdLastTryAt = millis();
    sdState = SdState::NoCard;
    if (!mountSd()) {
        Serial.println("[SD] remount failed (will keep retrying)");
    }
}

// ลบเฉพาะไฟล์ log_####.csv (ที่โปรแกรมนี้สร้าง) แล้วสร้างไฟล์ใหม่ ไม่แตะไฟล์อื่นในการ์ด
void deleteLogFiles()
{
    if (sdState != SdState::Ready) {
        Serial.println("[SD] card is not ready");
        return;
    }
    char names[40][16];
    int count = 0;
    File root = SD.open("/");
    if (!root) return;
    File entry = root.openNextFile();
    while (entry && count < 40) {
        const char *name = baseName(entry.name());
        if (!entry.isDirectory() && isLogFileName(name, nullptr)) {
            snprintf(names[count++], sizeof(names[0]), "%s", name);
        }
        entry.close();
        entry = root.openNextFile();
    }
    if (entry) entry.close();
    root.close();

    int removed = 0;
    for (int i = 0; i < count; i++) {
        char path[24];
        snprintf(path, sizeof(path), "/%s", names[i]);
        if (SD.remove(path)) removed++;
    }
    Serial.printf("[SD] removed %d of %d log file(s) (log_####.csv only)\n", removed, count);
    logRows = 0;
    createLogFile();
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
    Serial.println("[HELP] fan:  m = AUTO/MANUAL,  + / - = duty +-10% (MANUAL),  0-9 = duty 0..90% (MANUAL),  s = data source REAL/SIM");
    Serial.println("[HELP] time: n = ask NTP again,  t = print the time now");
    Serial.println("[HELP] SD:   i = card info,  l = list files,  p = show the end of the log,  e = eject (safe to remove the card),");
    Serial.println("[HELP]       r = remount and resume logging,  x then y = delete log_####.csv files made by this lab");
    Serial.println("[HELP] h = help.   BOOT button on the board = AUTO/MANUAL");
}

void printTime(uint32_t now)
{
    char stamp[24];
    if (makeTimestamp(stamp, sizeof(stamp))) {
        char ago[16];
        formatAgo(ago, sizeof(ago), now - lastSyncAtMs);
        Serial.printf("[TIME] %s (UTC+%ld)  sync #%lu, last sync %s ago%s\n", stamp, app::GMT_OFFSET_SEC / 3600,
                      static_cast<unsigned long>(syncCount), ago, timeStale(now) ? "  STALE" : "");
    } else {
        Serial.printf("[TIME] NOT SYNCED  (wifi=%s ntp=%s)\n", wifiStateName(wifiState), ntpStateName(ntpState));
    }
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

    Serial.printf("[STATUS] src=%-10s temp=%s C hum=%s %% | mode=%s fan=%s duty=%3d%% | sd=%s rows=%lu%s\n", sourceName(source),
                  tempText, humText, modeName(mode), fanDuty > 0 ? "RUN " : "STOP", fanDuty, sdStateName(sdState),
                  static_cast<unsigned long>(logRows), sensorError ? "  SENSOR ERROR" : "");
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

void handleSerialInput(uint32_t now)
{
    while (Serial.available() > 0) {
        const int c = Serial.read();
        if (c == '\n' || c == '\r') {
            continue;  // Enter ไม่นับเป็นคำสั่ง (และไม่ยกเลิกการรอยืนยันการลบ)
        }
        if (deletePending && c != 'y' && c != 'Y') {
            deletePending = false;
            Serial.println("[SD] delete cancelled");
        }

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
        } else if (c == 'n' || c == 'N') {
            if (wifiState == WifiState::Connected) {
                startNtp(now);  // ขอเวลาใหม่ทันที
            } else {
                Serial.println("[NTP] WiFi is not connected: cannot ask for the time now");
            }
        } else if (c == 't' || c == 'T') {
            printTime(now);
        } else if (c == 'i' || c == 'I') {
            printSdInfo();
        } else if (c == 'l' || c == 'L') {
            listSdFiles();
        } else if (c == 'p' || c == 'P') {
            printLogTail();
        } else if (c == 'e' || c == 'E') {
            ejectSd();
        } else if (c == 'r' || c == 'R') {
            remountSd();
        } else if (c == 'x' || c == 'X') {
            if (sdState == SdState::Ready) {
                deletePending = true;
                deleteAskedAt = now;
                Serial.printf("[SD] delete ALL log_####.csv files made by this lab? press y within %u s (any other key cancels)\n",
                              static_cast<unsigned>(app::SD_DELETE_CONFIRM_S));
            } else {
                Serial.println("[SD] card is not ready");
            }
        } else if (c == 'y' || c == 'Y') {
            if (deletePending && now - deleteAskedAt <= app::SD_DELETE_CONFIRM_S * 1000UL) {
                deletePending = false;
                deleteLogFiles();
            }
        } else if (c == 'h' || c == 'H' || c == '?') {
            printHelp();
        }
        // ตัวอักษรอื่นข้ามไป
    }
    if (deletePending && now - deleteAskedAt > app::SD_DELETE_CONFIRM_S * 1000UL) {
        deletePending = false;
        Serial.println("[SD] delete cancelled (no confirmation)");
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
    drawHeader("Lab 8 - SD Logger");

    drawCard(CLOCK_CARD_X, CLOCK_CARD_Y, CLOCK_CARD_W, CLOCK_CARD_H, "");

    drawCard(TEMP_CARD_X, TEMP_CARD_Y, TEMP_CARD_W, TEMP_CARD_H, "TEMP");
    drawCard(HUM_CARD_X, HUM_CARD_Y, HUM_CARD_W, HUM_CARD_H, "HUMIDITY");
    drawText("C", TEMP_CARD_X + 62, TEMP_CARD_Y + 20, 4, COLOR_DIM, COLOR_CARD);
    drawText("%", HUM_CARD_X + 44, HUM_CARD_Y + 20, 4, COLOR_DIM, COLOR_CARD);

    drawCard(FAN_CARD_X, FAN_CARD_Y, FAN_CARD_W, FAN_CARD_H, "FAN");
    drawCard(SD_CARD_X, SD_CARD_Y, SD_CARD_W, SD_CARD_H, "SD");

    drawnTimeKey = NEVER;
    drawnDateKey = NEVER;
    drawnBadgeKey = NEVER;
    drawnSyncCount = NEVER;
    drawnTempKey = NEVER;
    drawnHumKey = NEVER;
    drawnDuty = NEVER;
    drawnMode = NEVER;
    drawnSource = NEVER;
}

// ป้ายสถานะนาฬิกา: ข้อความ + สี + รหัส (ไว้เทียบว่าเปลี่ยนไหม)
struct Badge {
    const char *text;
    uint16_t color;
    int key;
};

Badge clockBadge(uint32_t now)
{
    if (wifiState == WifiState::NotConfigured) return {"NO WIFI", COLOR_RED, 1};
    if (ntpState == NtpState::Synced) return timeStale(now) ? Badge{"STALE", COLOR_YELLOW, 2} : Badge{"SYNCED", COLOR_GREEN, 3};
    if (ntpState == NtpState::Syncing) return {"SYNCING", COLOR_BLUE, 4};
    if (ntpState == NtpState::Failed) return {"NO SYNC", COLOR_RED, 5};
    if (wifiState == WifiState::Failed) return {"NO WIFI", COLOR_RED, 1};
    return {"WIFI...", COLOR_BLUE, 6};  // กำลังต่อ WiFi
}

// อัปเดตการ์ดนาฬิกา วาดใหม่เฉพาะส่วนที่เปลี่ยน (เวลาเปลี่ยนทุกวินาที)
void updateClockCard(uint32_t now)
{
    char text[32];
    const bool valid = timeValid();
    const time_t t = time(nullptr);

    const int timeKey = valid ? static_cast<int>(t % 86400) : -1;
    if (timeKey != drawnTimeKey) {
        if (valid) {
            formatLocalTime(text, sizeof(text), "%H:%M:%S");
        } else {
            snprintf(text, sizeof(text), "--:--:--");  // ยังไม่ sync: ไม่แสดงเวลาปี 1970
        }
        tft.setTextSize(2);
        drawText(text, CLOCK_CARD_X + 8, CLOCK_CARD_Y + 4, 4, valid ? (timeStale(now) ? COLOR_YELLOW : COLOR_TEXT) : COLOR_DIM,
                 COLOR_CARD, 200);
        tft.setTextSize(1);
        drawnTimeKey = timeKey;
    }

    // คีย์ของวันที่คำนวณจาก "เวลาท้องถิ่น" (UTC+7) ไม่ใช่ UTC ไม่เช่นนั้นวันที่บนจอจะเปลี่ยนตอน 07:00 แทนที่จะเป็นเที่ยงคืน
    const int dateKey = valid ? static_cast<int>((static_cast<long>(t) + app::GMT_OFFSET_SEC + app::DAYLIGHT_OFFSET_SEC) / 86400) : -1;
    if (dateKey != drawnDateKey) {
        if (valid) {
            formatLocalTime(text, sizeof(text), "%d/%m/%y");
        } else {
            text[0] = '\0';
        }
        drawText(text, CLOCK_CARD_X + 212, CLOCK_CARD_Y + 24, 2, COLOR_DIM, COLOR_CARD, 84);
        drawnDateKey = dateKey;
    }

    const Badge badge = clockBadge(now);
    if (badge.key != drawnBadgeKey) {
        drawText(badge.text, CLOCK_CARD_X + 212, CLOCK_CARD_Y + 6, 2, badge.color, COLOR_CARD, 84);
        drawnBadgeKey = badge.key;
    }

    if (static_cast<int>(syncCount) != drawnSyncCount) {
        snprintf(text, sizeof(text), "sync #%lu", static_cast<unsigned long>(syncCount));
        drawText(text, CLOCK_CARD_X + 212, CLOCK_CARD_Y + 42, 2, COLOR_DIM, COLOR_CARD, 84);
        drawnSyncCount = static_cast<int>(syncCount);
    }
}

// อัปเดตการ์ดอุณหภูมิ ความชื้น และพัดลม (เหมือน Lab 7 แต่ย่อให้เล็กลง)
void updateSensorAndFan(uint32_t now)
{
    char text[40];

    // ---- ป้ายบอกแหล่งที่มาของค่า: REAL (เขียว) / SIM (เหลือง) / N/A (เทา) ----
    const DataSource source = currentSource();
    if (static_cast<int>(source) != drawnSource) {
        const char *badge = (source == DataSource::Real) ? "REAL" : (isSimulated(source) ? "SIM" : "N/A");
        const uint16_t badgeColor = (source == DataSource::Real) ? COLOR_GREEN : (isSimulated(source) ? COLOR_YELLOW : COLOR_DIM);
        drawText(badge, TEMP_CARD_X + 104, TEMP_CARD_Y + 4, 2, badgeColor, COLOR_CARD, 40);
        drawText(badge, HUM_CARD_X + 104, HUM_CARD_Y + 4, 2, badgeColor, COLOR_CARD, 40);
        drawnSource = static_cast<int>(source);
    }

    // ---- อุณหภูมิ ----
    const float tempC = effectiveTemp(now);
    const int tempKey = isnan(tempC) ? -1 : static_cast<int>(lroundf(tempC * 10.0f));
    if (tempKey != drawnTempKey) {
        uint16_t color = COLOR_TEXT;
        if (isnan(tempC)) {
            snprintf(text, sizeof(text), "--.-");
            color = sensorError ? COLOR_RED : COLOR_DIM;
        } else {
            snprintf(text, sizeof(text), "%.1f", tempC);
            color = (tempC >= app::FAN_ON_TEMP_C) ? COLOR_ORANGE : COLOR_TEXT;
        }
        drawText(text, TEMP_CARD_X + 8, TEMP_CARD_Y + 20, 4, color, COLOR_CARD, 52);
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
        drawText(text, HUM_CARD_X + 8, HUM_CARD_Y + 20, 4, color, COLOR_CARD, 36);
        drawnHumKey = humKey;
    }

    // ---- พัดลม (บรรทัดเดียว) ----
    if (static_cast<int>(mode) != drawnMode) {
        drawText(modeName(mode), FAN_CARD_X + 224, FAN_CARD_Y + 5, 2, mode == Mode::Auto ? COLOR_GREEN : COLOR_BLUE, COLOR_CARD, 70);
        drawnMode = static_cast<int>(mode);
    }
    if (fanDuty != drawnDuty) {
        snprintf(text, sizeof(text), "%d%%", fanDuty);
        drawText(text, FAN_CARD_X + 52, FAN_CARD_Y + 5, 2, COLOR_TEXT, COLOR_CARD, 44);
        drawText(fanDuty > 0 ? "RUNNING" : "STOPPED", FAN_CARD_X + 104, FAN_CARD_Y + 5, 2,
                 fanDuty > 0 ? COLOR_GREEN : COLOR_DIM, COLOR_CARD, 76);
        drawnDuty = fanDuty;
    }
}

// การ์ด SD card (สองบรรทัด) อัปเดตทุก STATUS_REFRESH_MS: สถานะการ์ด ชนิด/ขนาด ชื่อไฟล์ จำนวนแถว เวลาที่ใช้เขียน
void updateSdCard()
{
    char text[48];
    uint16_t color = COLOR_DIM;
    switch (sdState) {
        case SdState::Ready: color = COLOR_GREEN; break;
        case SdState::NoCard: color = COLOR_RED; break;
        case SdState::WriteError: color = COLOR_RED; break;
        case SdState::Ejected: color = COLOR_YELLOW; break;
    }
    drawText(sdStateName(sdState), SD_CARD_X + 36, SD_CARD_Y + 4, 2, color, COLOR_CARD, 84);

    if (sdState == SdState::Ready) {
        snprintf(text, sizeof(text), "%s %.1fGB", sdTypeName(sdCardType), static_cast<double>(sdCardBytes) / 1073741824.0);
    } else if (sdState == SdState::Ejected) {
        snprintf(text, sizeof(text), "safe to remove");
    } else {
        snprintf(text, sizeof(text), "insert FAT32 card");
    }
    drawText(text, SD_CARD_X + 128, SD_CARD_Y + 4, 2, COLOR_DIM, COLOR_CARD, 170);

    drawText(logPath[0] ? baseName(logPath) : "-", SD_CARD_X + 8, SD_CARD_Y + 25, 2, COLOR_TEXT, COLOR_CARD, 112);
    snprintf(text, sizeof(text), "rows %lu", static_cast<unsigned long>(logRows));
    drawText(text, SD_CARD_X + 124, SD_CARD_Y + 25, 2, COLOR_TEXT, COLOR_CARD, 90);
    snprintf(text, sizeof(text), "wr %lums", static_cast<unsigned long>((lastWriteMicros + 500) / 1000));
    drawText(text, SD_CARD_X + 222, SD_CARD_Y + 25, 2, COLOR_DIM, COLOR_CARD, 76);
}

// ข้อความ WiFi ที่มุมขวาของแถบหัว (แทนบรรทัดล่างจอของ Lab 7 เพราะที่เต็มแล้ว)
void updateHeaderStatus()
{
    char text[24];
    uint16_t color = COLOR_TEXT;
    switch (wifiState) {
        case WifiState::NotConfigured:
            snprintf(text, sizeof(text), "no WiFi cfg");
            color = COLOR_YELLOW;
            break;
        case WifiState::Connecting:
            snprintf(text, sizeof(text), "WiFi ...");
            break;
        case WifiState::Failed:
            snprintf(text, sizeof(text), "WiFi retry");
            color = COLOR_RED;
            break;
        case WifiState::Connected:
            snprintf(text, sizeof(text), "WiFi %ddBm", WiFi.RSSI());
            break;
    }
    drawText(text, 168, 6, 2, color, COLOR_HEADER, 124);
}

void updateHeartbeat()
{
    heartbeatOn = !heartbeatOn;
    tft.fillCircle(SCREEN_W - 14, HEADER_H / 2, 5, heartbeatOn ? COLOR_GREEN : COLOR_HEADER);
}

}  // namespace

void setup()
{
    Serial.begin(app::SERIAL_BAUD);
    delay(200);

    Serial.println();
    Serial.println("==========================================");
    Serial.println("Hello World from ESP32-2432S028 (CYD)!");
    Serial.println("Level 1 / Lab 8 : SD Logger");
    Serial.println("==========================================");
    Serial.printf("[DHT11] data pin = GPIO%u (do NOT use GPIO21: it is the TFT backlight)\n", cyd::DHT11_PIN);
    Serial.printf("[SD] pins: CS=GPIO%u MOSI=GPIO%u SCK=GPIO%u MISO=GPIO%u\n", cyd::SD_PIN_CS, cyd::SD_PIN_MOSI, cyd::SD_PIN_CLK,
                  cyd::SD_PIN_MISO);

    cyd::ledInit();
    pinMode(cyd::BOOT_BUTTON_PIN, INPUT_PULLUP);

    tft.init();  // TFT_eSPI เปิดไฟหลังจอให้ตรงนี้
    tft.setRotation(cyd::DISPLAY_ROTATION);
    tft.setTextDatum(TL_DATUM);
    tft.fillScreen(COLOR_BG);

    pwmBegin();
    dht.begin();
    lastReadAt = millis();  // DHT11 ต้องรอให้เซนเซอร์พร้อมก่อน จึงให้อ่านครั้งแรกหลังผ่านไป DHT_READ_INTERVAL_MS

    // SD card: เริ่ม SPI ชุดแยกด้วยขาของการ์ด แล้วลองเมาท์ (ไม่มีการ์ดก็ไม่ค้าง)
    sdSpi.begin(cyd::SD_PIN_CLK, cyd::SD_PIN_MISO, cyd::SD_PIN_MOSI, cyd::SD_PIN_CS);
    sdLastTryAt = millis();
    mountSd();

    // WiFi + NTP
    WiFi.persistent(false);
    WiFi.mode(WIFI_STA);
    WiFi.setAutoReconnect(false);
    sntp_set_time_sync_notification_cb(onTimeSync);  // ให้ไลบรารีแจ้งเมื่อได้เวลา
    sntp_set_sync_interval(app::NTP_RESYNC_MS);      // ขอเวลาใหม่เป็นระยะ
    if (strlen(WIFI_SSID) > 0) {
        startWifiConnect();
    } else {
        Serial.println("[WIFI] include/secrets.h not found (or WIFI_SSID is empty): no real time, the log will say NOT_SYNCED");
    }

    lastLogAt = millis();
    drawMainScreenStatic();
    printHelp();
}

void loop()
{
    const uint32_t now = millis();

    // 1) อ่านเซนเซอร์เป็นระยะ
    if (now - lastReadAt >= app::DHT_READ_INTERVAL_MS) {
        lastReadAt = now;
        readDht();
    }

    // 2) ปุ่มและคำสั่ง Serial
    if (bootButtonPressed(now)) {
        toggleMode();
    }
    handleSerialInput(now);

    // 3) WiFi และเวลา NTP
    updateWifi(now);
    updateNtp(now);
    setSyncLed(ntpState == NtpState::Synced);

    // 4) แหล่งข้อมูล: สุ่มค่า SIM (ถ้ากำลังใช้) และแจ้งเมื่อแหล่งเปลี่ยน (REAL <-> SIM)
    const DataSource source = currentSource();
    if (source == DataSource::SimRandom && now - lastSimAt >= app::SIM_UPDATE_MS) {
        lastSimAt = now;
        updateSimulationRandom();
    }
    if (source != lastReportedSource) {
        Serial.printf("[SRC] data source: %s -> %s\n", sourceName(lastReportedSource), sourceName(source));
        lastReportedSource = source;
    }

    // 5) ตรรกะพัดลม + LED แดงแจ้งปัญหาเซนเซอร์
    if (now - lastFanAt >= app::FAN_UPDATE_MS) {
        lastFanAt = now;
        if (mode == Mode::Auto) {
            updateFanAuto(now);
        }
        setErrorLed(sensorError && simChoice == SimChoice::Off);
    }

    // 6) SD card: ลองเมาท์ใหม่เมื่อจำเป็น และบันทึก 1 แถวทุก LOG_INTERVAL_MS
    updateSd(now);
    if (now - lastLogAt >= app::LOG_INTERVAL_MS) {
        lastLogAt = now;
        appendLogRow(now);
    }

    // 7) วาดจอ (เฉพาะส่วนที่เปลี่ยน)
    if (now - lastDisplayAt >= app::DISPLAY_REFRESH_MS) {
        lastDisplayAt = now;
        updateClockCard(now);
        updateSensorAndFan(now);
    }
    if (now - lastStatusAt >= app::STATUS_REFRESH_MS) {
        lastStatusAt = now;
        updateSdCard();
        updateHeaderStatus();
    }
    if (now - lastHeartbeatAt >= app::HEARTBEAT_MS) {
        lastHeartbeatAt = now;
        updateHeartbeat();
    }

    if (now - lastSerialAt >= app::SERIAL_PRINT_MS) {
        lastSerialAt = now;
        printStatus();
    }
    if (now - lastTimePrintAt >= app::TIME_PRINT_MS) {
        lastTimePrintAt = now;
        printTime(now);
    }
}
