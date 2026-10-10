// Level 1 / Lab 11 : MQTT Farm (ส่งข้อมูลและรับคำสั่งผ่าน MQTT)
//
// ต่อยอดจาก Lab 10 (Smart Farm Controller) โดยเพิ่ม MQTT ไปยัง broker สาธารณะ (ค่าเริ่มต้น broker.hivemq.com:1883 ไม่มีรหัสผ่าน)
//   ส่งออก (publish)  : <root>/status (online/offline, retained, LWT), <root>/telemetry (ค่าเซนเซอร์+สถานะทุก 10 วินาที, retained),
//                       <root>/event (ปั๊ม/พัดลม/ไฟเริ่ม-หยุด ฯลฯ), <root>/ack (ผลของคำสั่ง)   root = cydfarm/<รหัสจาก MAC>
//   รับเข้า (subscribe): <root>/cmd/fan|light|pump  = ON | OFF | AUTO ,  <root>/cmd/estop = หยุดฉุกเฉิน
//   ความปลอดภัย      : คำสั่งถูกรับเฉพาะเมื่อกดปุ่ม REMOTE CMD บนจอ (ปิดทุกครั้งที่เปิดเครื่อง และหมดอายุเองใน 10 นาที)
//                       ปั๊มยังติด watchdog 30 วินาทีและช่วงพัก 20 วินาทีเหมือนเดิม  broker สาธารณะ = ใครก็อ่านได้ ห้ามใช้กับของจริง
//   ต้องมี WiFi (include/secrets.h) ถึงจะใช้ MQTT ได้ ไม่มี WiFi ตัวควบคุมและจอสัมผัสยังทำงานปกติ
//
// ส่วนที่เหลือเหมือน Lab 10: ตัวควบคุมโรงเรือนจำลอง
//   เซนเซอร์  : อุณหภูมิ/ความชื้น (DHT11), แสง (LDR บนบอร์ด), ความชื้นดิน (จำลองในโปรแกรม)
//   อุปกรณ์   : พัดลม (LED น้ำเงิน GPIO17, PWM), ไฟปลูก (LED เขียว GPIO16), ปั๊มน้ำ (LED แดง GPIO4)  ทุกสี active-LOW
//   หน้าจอ    : DASH (ภาพรวม) / CTRL (AUTO-MANUAL ของแต่ละอุปกรณ์) / SETUP (ปรับเกณฑ์และตารางเวลา 2 หน้า) / SYSTEM
//
// กติกาการควบคุม (โหมด AUTO)
//   พัดลม : อุณหภูมิ >= ON เปิด, <= OFF ปิด (hysteresis), ความเร็วเพิ่มตามอุณหภูมิ เซนเซอร์เสีย = fail-safe หมุน 50%
//   ไฟปลูก: เปิดเมื่ออยู่ในช่วงเวลา (เช่น 06:00-18:00) และแสงต่ำกว่าเกณฑ์, ดับเมื่อแสงสูงกว่าเกณฑ์ + hysteresis
//           ถ้ายังไม่มีเวลาจริง (NTP) จะไม่ใช้ช่วงเวลา ใช้แต่แสงอย่างเดียว
//   ปั๊ม  : เริ่มเมื่อถึงเวลารดน้ำ 2 รอบ (HH:00) หรือดินแห้งกว่าเกณฑ์ ความปลอดภัย: ทำงานต่อเนื่องได้ไม่เกิน 30 วินาที
//           และพักอย่างน้อย 20 วินาทีหลังหยุดทุกครั้ง (ทั้งโหมด AUTO และ MANUAL)
//   MANUAL: ผู้ใช้สั่ง ON/OFF เอง (ปั๊มยังติดข้อจำกัดความปลอดภัย) ปุ่ม BOOT = หยุดฉุกเฉิน (ทุกอุปกรณ์ MANUAL + OFF)
//
// เซนเซอร์เสีย/NTP ไม่ sync จะแสดงบนจอ (ป้าย ERR / NO SYNC) แทนไฟ LED ดวงแดง/เขียวของ Lab 6-9
// ค่าที่ปรับจากหน้า SETUP ถูกเก็บถาวรด้วย Preferences ส่วนโหมดกลับเป็น AUTO ทุกครั้งที่เปิดเครื่อง (ปลอดภัยไว้ก่อน)
//
// จอสัมผัส/Calibrate เหมือน Lab 9 (XPT2046 บน VSPI) จึงยังไม่รวม SD card (ขา SPI ชุดเดียวกัน)
//
// คำสั่ง Serial: f l p = สลับ ON/OFF แบบ MANUAL ของพัดลม/ไฟ/ปั๊ม, a = ทุกอุปกรณ์กลับเป็น AUTO,
//   s = สลับ REAL/SIM ของอุณหภูมิ+ความชื้น, e = สลับแสง REAL/SIM, d = ทำให้ดินแห้ง (ทดสอบ), n t c k h
//   r = เปิด/ปิดการรับคำสั่งทาง MQTT (REMOTE CMD), m = พิมพ์สถานะ MQTT

#include <Arduino.h>
#include <DHT.h>
#include <PubSubClient.h>
#include <Preferences.h>
#include <SPI.h>
#include <TFT_eSPI.h>
#include <WiFi.h>
#include <esp_sntp.h>
#include <esp_system.h>
#include <time.h>

#include "app_config.h"
#include "board_config.h"

// ถ้ายังไม่มีไฟล์ secrets.h โปรแกรมก็ยัง build ได้ แต่จะไม่มีเวลาจริง (ส่วนอื่นรวมถึงจอสัมผัสใช้ได้ปกติ)
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
constexpr uint16_t COLOR_BUTTON = rgb565(34, 48, 68);        // ปุ่มปกติ
constexpr uint16_t COLOR_PRESSED = rgb565(80, 115, 170);     // ปุ่มขณะถูกกด
constexpr uint16_t COLOR_ACTIVE = rgb565(28, 100, 170);      // แท็บ/ปุ่มที่เลือกอยู่
constexpr uint16_t COLOR_ACTIVE_GREEN = rgb565(20, 110, 70); // ปุ่มที่เปิดอยู่ (ON)

constexpr int16_t SCREEN_W = 320;
constexpr int16_t SCREEN_H = 240;
constexpr int16_t HEADER_H = 28;

struct Rect {
    int16_t x, y, w, h;
};

constexpr bool contains(const Rect &r, int x, int y)
{
    return x >= r.x && x < r.x + r.w && y >= r.y && y < r.y + r.h;
}

// แถบแท็บด้านล่าง (5 ปุ่ม สูง 36 พิกเซล)
constexpr Rect TAB_RECTS[5] = {{4, 202, 59, 36}, {67, 202, 59, 36}, {130, 202, 59, 36}, {193, 202, 59, 36}, {256, 202, 59, 36}};
constexpr const char *TAB_NAMES[5] = {"DASH", "CTRL", "SETUP", "MQTT", "SYS"};

// หน้า DASH
constexpr Rect CLOCK_CARD = {8, 32, 304, 34};
constexpr Rect TEMP_CARD = {8, 70, 74, 56};
constexpr Rect HUM_CARD = {85, 70, 74, 56};
constexpr Rect LIGHT_CARD = {162, 70, 74, 56};
constexpr Rect SOIL_CARD = {239, 70, 74, 56};
constexpr Rect FAN_TILE = {8, 130, 100, 68};
constexpr Rect LIGHT_TILE = {110, 130, 100, 68};
constexpr Rect PUMP_TILE = {212, 130, 100, 68};

// หน้า CTRL: แถวละ 1 อุปกรณ์ (พัดลม, ไฟปลูก, ปั๊ม)
constexpr Rect CTRL_ROWS[3] = {{8, 34, 304, 52}, {8, 89, 304, 52}, {8, 144, 304, 52}};
constexpr Rect CTRL_MODE_BTN[3] = {{128, 42, 88, 36}, {128, 97, 88, 36}, {128, 152, 88, 36}};
constexpr Rect CTRL_MAN_BTN[3] = {{222, 42, 88, 36}, {222, 97, 88, 36}, {222, 152, 88, 36}};

// หน้า SETUP: 5 แถว แต่ละแถวมีปุ่ม - และ +
constexpr int16_t SETUP_ROW_Y[5] = {34, 66, 98, 130, 162};
constexpr Rect SET_MINUS_BTN[5] = {{200, 34, 52, 28}, {200, 66, 52, 28}, {200, 98, 52, 28}, {200, 130, 52, 28}, {200, 162, 52, 28}};
constexpr Rect SET_PLUS_BTN[5] = {{260, 34, 52, 28}, {260, 66, 52, 28}, {260, 98, 52, 28}, {260, 130, 52, 28}, {260, 162, 52, 28}};

// หน้า SYSTEM
constexpr Rect BL_SLIDER_AREA = {8, 52, 304, 34};
constexpr Rect BL_SLIDER_TRACK = {24, 65, 272, 8};
constexpr Rect SYNC_BTN = {8, 88, 148, 32};
constexpr Rect TEST_BTN = {164, 88, 148, 32};
constexpr Rect LDR_DARK_BTN = {8, 124, 148, 32};
constexpr Rect LDR_BRIGHT_BTN = {164, 124, 148, 32};
constexpr Rect RECAL_BTN = {8, 160, 304, 30};

// หน้า MQTT
constexpr Rect REMOTE_BTN = {8, 154, 304, 40};

// หน้า TOUCH TEST
constexpr Rect PAINT_BOX = {8, 92, 304, 106};
constexpr Rect BACK_BTN = {8, 202, 148, 36};
constexpr Rect CLEAR_BTN = {164, 202, 148, 36};

enum class Page : uint8_t { Dashboard, Control, Setup, Mqtt, System, Test, Calibrate };
enum class Widget : uint8_t {
    None,
    TabDash, TabCtrl, TabSetup, TabMqtt, TabSystem,
    ClockCard, TempCard, HumCard, LightCard, SoilCard, FanTile, LightTile, PumpTile,
    FanModeBtn, FanManBtn, LightModeBtn, LightManBtn, PumpModeBtn, PumpManBtn,
    SetMinus0, SetMinus1, SetMinus2, SetMinus3, SetMinus4,
    SetPlus0, SetPlus1, SetPlus2, SetPlus3, SetPlus4,
    BlSlider, SyncBtn, TestBtn, LdrDarkBtn, LdrBrightBtn, RecalBtn, RemoteBtn,
    TestPaint, TestBack, TestClear
};
enum class Mode : uint8_t { Auto, Manual };
enum class WifiState : uint8_t { NotConfigured, Connecting, Connected, Failed };
enum class NtpState : uint8_t { Idle, Syncing, Synced, Failed };
enum class SimChoice : uint8_t { Off, Random, Sweep };
enum class DataSource : uint8_t { None, Real, SimRandom, SimSweep };
enum class PumpReason : uint8_t { None, Schedule, Soil, Manual };
// ค่าที่ปรับได้จากหน้า SETUP
enum class Setting : uint8_t { None, FanOn, FanOff, LightThr, LightFrom, LightTo, SoilDry, Water1, Water2, RunSec };

TFT_eSPI tft;
DHT dht(cyd::DHT11_PIN, DHT11);
SPIClass touchSpi(VSPI);  // SPI ชุดสำหรับจอสัมผัส (จอ TFT ใช้ HSPI)
Preferences prefs;        // เก็บค่า calibrate ความสว่างจอ และเกณฑ์ต่างๆ แบบถาวร (ไม่หายเมื่อไฟดับ)

Page page = Page::Dashboard;
int setupSub = 0;  // หน้า SETUP หน้าย่อยที่เท่าไร (0 หรือ 1) แตะแท็บ SETUP ซ้ำเพื่อสลับ

// ---- WiFi / NTP (เหมือน Lab 7) ----
WifiState wifiState = WifiState::NotConfigured;
uint32_t wifiStateSince = 0;
NtpState ntpState = NtpState::Idle;
uint32_t ntpStartedAt = 0;
uint32_t syncCount = 0;
uint32_t lastSyncAtMs = 0;
volatile bool syncEvent = false;

// ---- DHT11 / ค่าจำลอง (เหมือน Lab 6-7) ----
float sensorTempC = NAN;
float sensorHumidity = NAN;
bool haveReading = false;
uint8_t consecutiveFails = 0;
bool sensorError = false;
uint32_t readCount = 0;
uint32_t okCount = 0;
uint32_t failCount = 0;
uint32_t lastReadMicros = 0;

SimChoice simChoice = SimChoice::Off;
float simTemp = app::SIM_TEMP_START_C;
float simHum = app::SIM_HUM_START;
uint32_t lastSimAt = 0;
DataSource lastReportedSource = DataSource::None;

// ---- LDR (แสง) และดินชื้น (จำลอง) ----
int ldrRaw = 0;
int ldrRawDark = app::LDR_RAW_DARK;      // raw ที่ถือว่ามืดสุด (ปรับได้จากหน้า SYSTEM แล้วเก็บถาวร)
int ldrRawBright = app::LDR_RAW_BRIGHT;  // raw ที่ถือว่าสว่างเต็มที่
float ldrSmooth = NAN;
bool lightSim = false;  // true = ใช้แสงจำลอง (ไล่ขึ้น-ลงเหมือนกลางวัน-กลางคืน) แทน LDR
uint32_t lastLdrAt = 0;
float soilPercent = app::SOIL_START_PERCENT;
uint32_t lastSoilAt = 0;

// ---- อุปกรณ์ 3 ตัว ----
struct Device {
    Mode mode = Mode::Auto;
    bool manualOn = false;  // สถานะที่ผู้ใช้สั่งเมื่ออยู่โหมด MANUAL
};
Device fanDev, lightDev, pumpDev;

int fanDuty = 0;
uint32_t fanValue = 0;
bool fanAutoOn = false;   // หน่วยความจำ hysteresis ของพัดลม
bool fanFailsafe = false; // เซนเซอร์อุณหภูมิเสียและกำลังใช้ค่า fail-safe
bool lightOn = false;
bool pumpOn = false;
PumpReason pumpReason = PumpReason::None;
uint32_t pumpStartedAt = 0;
uint32_t pumpRunLimitMs = 0;
bool pumpLocked = false;      // กำลังพักปั๊ม (cooldown)
uint32_t pumpLockUntil = 0;
long slotDoneDay[2] = {-1, -1};  // รอบรดน้ำที่ทำไปแล้วของ "วันที่" นั้น (เก็บใน RAM: รีเซ็ตบอร์ดแล้วรดซ้ำได้ถ้ายังอยู่ในช่วงผ่อนผัน)

// ค่าตั้งที่ปรับจากหน้า SETUP
struct Settings {
    int fanOn = app::FAN_ON_TEMP_C;
    int fanOff = app::FAN_OFF_TEMP_C;
    int lightThr = app::LIGHT_THRESHOLD_PERCENT;
    int lightFrom = app::LIGHT_FROM_HOUR;
    int lightTo = app::LIGHT_TO_HOUR;
    int soilDry = app::SOIL_DRY_PERCENT;
    int water1 = app::WATER_HOUR_1;
    int water2 = app::WATER_HOUR_2;
    int runSec = app::PUMP_RUN_SEC;
};
Settings cfg;
bool settingsDirty = false;

int backlightPercent = app::BACKLIGHT_DEFAULT_PERCENT;

// ---- จอสัมผัส ----
// ค่าแปลงจากค่าดิบ (0-4095) เป็นพิกัดจอ: screen = a + b * raw  (แยกแกน x กับ y) swapXY = แกนดิบสลับกับแกนจอ
struct TouchCalibration {
    bool valid = false;
    bool swapXY = false;
    float xa = 0.0f, xb = 0.0f;
    float ya = 0.0f, yb = 0.0f;
};
TouchCalibration cal;

struct RawTouch {
    int16_t x = 0, y = 0;
    int16_t z = 0;  // แรงกด (ยิ่งมากยิ่งกดแรง) น้อยกว่า TOUCH_Z_MIN = ไม่ได้แตะ
};

struct TouchState {
    bool down = false;        // กำลังแตะอยู่ (ผ่านการกันสัญญาณรบกวนแล้ว)
    bool pressEdge = false;   // เพิ่ง "กดลง" ในรอบนี้
    bool releaseEdge = false; // เพิ่ง "ปล่อย" ในรอบนี้
    bool touchingNow = false; // ตัวอย่างล่าสุดมีแรงกดพอหรือไม่
    int16_t rawX = 0, rawY = 0, z = 0;
    int x = 0, y = 0;         // พิกัดจอ (กรองแล้ว)
    int lastX = 0, lastY = 0; // พิกัดล่าสุดตอนยังแตะอยู่
    uint32_t downSince = 0;
    uint8_t pressCount = 0, releaseCount = 0;
    bool smoothReady = false;
};
TouchState touch;
uint32_t lastTouchPollAt = 0;

Widget pressTarget = Widget::None;    // ปุ่มที่นิ้วกดลงตอนแรก
Widget pressedWidget = Widget::None;  // ปุ่มที่ "ดูเหมือนถูกกดอยู่" (นิ้วยังอยู่ในกรอบ)
uint32_t repeatAt = 0;
bool longPressDone = false;
int drawnLongPress = -1;
int lastPaintX = -1, lastPaintY = -1;

// calibrate
int calStep = 0;
int calAccepted = 0;
int calSamples = 0;
long calSumX = 0, calSumY = 0;
bool calWaitRelease = false;
int calRawX[4] = {0, 0, 0, 0};
int calRawY[4] = {0, 0, 0, 0};
uint32_t calMessageUntil = 0;
bool calResultOk = false;

// ---- MQTT (ตัวแปรอยู่ตรงนี้ ฟังก์ชันอยู่ในส่วน "MQTT" ด้านล่าง) ----
enum class MqttState : uint8_t { NoWifi, Waiting, Connected };  // ไม่มี WiFi / รอลองเชื่อมต่อ / เชื่อมต่อแล้ว

WiFiClient mqttNet;
PubSubClient mqtt(mqttNet);
MqttState mqttState = MqttState::NoWifi;
char mqttRoot[40] = "";       // topic หลักของบอร์ดนี้ เช่น cydfarm/a1b2c3
char mqttClientId[32] = "";
uint32_t mqttNextAttemptAt = 0;
uint32_t mqttBackoffMs = app::MQTT_RECONNECT_MIN_MS;
uint32_t lastTelemetryAt = 0;
uint32_t mqttTx = 0;          // จำนวนข้อความที่ส่งออกสำเร็จ
uint32_t mqttRx = 0;          // จำนวนข้อความที่รับเข้า (คำสั่ง)
uint32_t mqttDropped = 0;     // จำนวนเหตุการณ์ที่ส่งไม่ได้เพราะยังไม่เชื่อมต่อ
uint32_t mqttConnectFails = 0;
int mqttLastRc = 0;           // รหัสสถานะล่าสุดของ PubSubClient (0 = ok, ติดลบ = ปัญหาเครือข่าย)
uint32_t mqttLastBlockMs = 0; // เวลาที่การเชื่อมต่อครั้งล่าสุดทำให้โปรแกรมหยุดรอ
uint32_t mqttMaxBlockMs = 0;  // นานสุดที่เคยหยุดรอ
bool remoteEnabled = false;   // รับคำสั่งทาง MQTT หรือไม่ (ปิดทุกครั้งที่เปิดเครื่อง)
uint32_t remoteUntil = 0;
char lastCmdText[64] = "";    // คำสั่งล่าสุดและผล (แสดงในหน้า MQTT)
int lastCmdSeq = 0;
char mqttLine[64] = "";

void publishEvent(const char *name, const char *detail);  // ใช้ในตรรกะควบคุม นิยามอยู่ในส่วน "MQTT"

// ---- จับเวลา ----
uint32_t lastReadAt = 0;
uint32_t lastControlAt = 0;
uint32_t lastDisplayAt = 0;
uint32_t lastStatusAt = 0;
uint32_t lastHeartbeatAt = 0;
uint32_t lastSerialAt = 0;
uint32_t lastTimePrintAt = 0;
bool heartbeatOn = true;

// สิ่งที่วาดบนจอไปล่าสุด (วาดเฉพาะส่วนที่เปลี่ยน)
constexpr int NEVER = -32768;
int drawnTimeKey = NEVER;
int drawnDateKey = NEVER;
int drawnBadgeKey = NEVER;
int drawnSyncCount = NEVER;
int drawnTempKey = NEVER;
int drawnHumKey = NEVER;
int drawnLightKey = NEVER;
int drawnSoilKey = NEVER;
int drawnTempTag = NEVER;
int drawnHumTag = NEVER;
int drawnLightTag = NEVER;
int drawnSoilTag = NEVER;
int drawnTile[3] = {NEVER, NEVER, NEVER};
int drawnCtrl[3] = {NEVER, NEVER, NEVER};
int drawnSetup[5] = {NEVER, NEVER, NEVER, NEVER, NEVER};
int drawnBacklight = NEVER;
int drawnHeaderTime = NEVER;
int drawnTestKey = NEVER;
int drawnMqttDot = NEVER;
int drawnMqttState = NEVER;
int drawnMqttCounts = NEVER;
int drawnMqttCmd = NEVER;
int drawnMqttBlock = NEVER;
int drawnRemote = NEVER;

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

void drawCard(const Rect &r, const char *label)
{
    tft.fillRoundRect(r.x, r.y, r.w, r.h, 6, COLOR_CARD);
    tft.drawRoundRect(r.x, r.y, r.w, r.h, 6, COLOR_BORDER);
    if (label[0] != '\0') {
        drawText(label, r.x + 6, r.y + 3, 2, COLOR_DIM, COLOR_CARD);
    }
}

// ปุ่มสี่เหลี่ยมมุมมน ข้อความกึ่งกลาง
void drawButton(const Rect &r, const char *label, uint16_t fill, uint16_t border, uint16_t textColor, uint8_t font = 2)
{
    tft.fillRoundRect(r.x, r.y, r.w, r.h, 6, fill);
    tft.drawRoundRect(r.x, r.y, r.w, r.h, 6, border);
    tft.setTextColor(textColor, fill);
    tft.setTextPadding(0);
    tft.setTextDatum(MC_DATUM);
    tft.drawString(label, r.x + r.w / 2, r.y + r.h / 2, font);
    tft.setTextDatum(TL_DATUM);
}

// ---------------------------------------------------------------------
// เวลาจริง (NTP) เหมือน Lab 7
// ---------------------------------------------------------------------
void onTimeSync(struct timeval *tv)
{
    (void)tv;
    syncEvent = true;
}

bool timeValid()
{
    return syncCount > 0 && static_cast<uint32_t>(time(nullptr)) >= app::SANE_EPOCH;
}

bool timeStale(uint32_t now)
{
    return ntpState == NtpState::Synced && (now - lastSyncAtMs) > app::NTP_RESYNC_MS * app::NTP_STALE_FACTOR;
}

void formatLocalTime(char *out, size_t size, const char *format)
{
    const time_t t = time(nullptr);
    struct tm local;
    localtime_r(&t, &local);
    strftime(out, size, format, &local);
}

bool makeTimestamp(char *out, size_t size)
{
    if (!timeValid()) {
        snprintf(out, size, "NOT_SYNCED");
        return false;
    }
    formatLocalTime(out, size, "%Y-%m-%d %H:%M:%S");
    return true;
}

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

// ขอเวลา NTP ทันที (ใช้กับปุ่มบนจอและคำสั่ง n)
void requestNtpNow(uint32_t now)
{
    if (wifiState == WifiState::Connected) {
        startNtp(now);
    } else {
        Serial.println("[NTP] WiFi is not connected: cannot ask for the time now");
    }
}

void updateNtp(uint32_t now)
{
    if (syncEvent) {
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
            ntpState = NtpState::Idle;
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
// WiFi (แบบย่อจาก Lab 3)
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
                Serial.println("[WIFI] connection lost (the clock keeps running)");
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
float sweepFraction(uint32_t now)
{
    const float half = static_cast<float>(app::SIM_PERIOD_MS) / 2.0f;
    const float t = static_cast<float>(now % app::SIM_PERIOD_MS);
    return (t < half) ? (t / half) : ((static_cast<float>(app::SIM_PERIOD_MS) - t) / half);
}

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

DataSource currentSource()
{
    if (simChoice == SimChoice::Random) return DataSource::SimRandom;
    if (simChoice == SimChoice::Sweep) return DataSource::SimSweep;
    if (sensorValid()) return DataSource::Real;
    if (sensorError && app::SIM_WHEN_NO_SENSOR) return DataSource::SimRandom;
    return DataSource::None;
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
        case DataSource::SimSweep:
            return app::SIM_HUM_MAX - sweepFraction(now) * (app::SIM_HUM_MAX - app::SIM_HUM_MIN);
        case DataSource::None: break;
    }
    return NAN;
}


// ---------------------------------------------------------------------
// เอาต์พุต: พัดลม (PWM) ไฟปลูก ปั๊ม และไฟหลังจอ
// ---------------------------------------------------------------------
void writeFanPwm(uint32_t value)
{
    // active-LOW: ต้องส่ง (ค่าสูงสุด - value)
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
    ledcSetup(app::FAN_CHANNEL, app::PWM_FREQUENCY_HZ, app::PWM_RESOLUTION_BITS);
    ledcAttachPin(cyd::LED_PIN_B, app::FAN_CHANNEL);  // GPIO17 (LED น้ำเงิน) = พัดลมจำลอง
    applyFanDuty(0);
}

// ความสว่างจอ: PWM ที่ GPIO21 (ขาไฟหลังจอ) ไม่กลับค่าเพราะไฟหลังจอเป็น active-HIGH
void applyBacklight(int percent)
{
    backlightPercent = constrain(percent, app::BACKLIGHT_MIN_PERCENT, 100);
    const uint32_t value = (static_cast<uint32_t>(backlightPercent) * app::PWM_MAX_VALUE + 50) / 100;
    ledcWrite(app::BACKLIGHT_CHANNEL, value);
}

void backlightBegin()
{
    // ต้องทำหลัง tft.init() (ไลบรารีจอเปิดไฟหลังจอด้วย digitalWrite ไว้ก่อน) เหมือน Lab 5
    ledcSetup(app::BACKLIGHT_CHANNEL, app::PWM_FREQUENCY_HZ, app::PWM_RESOLUTION_BITS);
    ledcAttachPin(cyd::TFT_PIN_BACKLIGHT, app::BACKLIGHT_CHANNEL);
    applyBacklight(backlightPercent);
}

void setLedPin(uint8_t pin, bool on)
{
    digitalWrite(pin, (on == cyd::LED_ACTIVE_LOW) ? LOW : HIGH);
}

void setLightOutput(bool on)
{
    lightOn = on;
    setLedPin(cyd::LED_PIN_G, on);  // LED เขียว = ไฟปลูก
}

void setPumpOutput(bool on)
{
    pumpOn = on;
    setLedPin(cyd::LED_PIN_R, on);  // LED แดง = ปั๊มน้ำ
}

// ---------------------------------------------------------------------
// เซนเซอร์แสง (LDR) และดินชื้น (จำลอง)
// ---------------------------------------------------------------------
void readLdr()
{
    long sum = 0;
    for (uint8_t i = 0; i < app::LDR_SAMPLES; i++) {
        sum += analogRead(cyd::LDR_PIN);
    }
    ldrRaw = static_cast<int>(sum / app::LDR_SAMPLES);
    if (isnan(ldrSmooth)) {
        ldrSmooth = static_cast<float>(ldrRaw);
    } else {
        ldrSmooth += (static_cast<float>(ldrRaw) - ldrSmooth) * app::LDR_SMOOTH_ALPHA;
    }
}

// แสงรอบตัว 0-100% (มืด = 0, สว่าง = 100) จาก LDR หรือค่าจำลอง ยังไม่มีค่า = NAN
float ambientPercent(uint32_t now)
{
    if (lightSim) return sweepFraction(now) * 100.0f;
    if (isnan(ldrSmooth)) return NAN;
    // raw แปรผกผันกับแสง: ใส่ช่วงขาเข้า "มืดก่อน สว่างทีหลัง"
    const long pct = map(lroundf(ldrSmooth), ldrRawDark, ldrRawBright, 0, 100);
    return static_cast<float>(constrain(pct, 0L, 100L));
}

// จำค่า raw ปัจจุบันของ LDR เป็นปลาย "มืด" หรือ "สว่าง" (ครอบมือบังเซนเซอร์ = มืด, ส่องไฟ/ที่สว่างปกติ = สว่าง)
void captureLdrEnd(bool dark)
{
    if (isnan(ldrSmooth)) return;
    const int value = static_cast<int>(lroundf(ldrSmooth));
    const int other = dark ? ldrRawBright : ldrRawDark;
    const bool orderOk = dark ? (value > other) : (value < other);  // raw แปรผกผันกับแสง: มืด > สว่าง
    if (!orderOk || abs(value - other) < app::LDR_MIN_SPAN) {
        Serial.printf("[LDR] refused: %s raw=%d but the other end is %d (need dark > bright by at least %d)\n", dark ? "DARK" : "BRIGHT", value,
                      other, app::LDR_MIN_SPAN);
        return;
    }
    (dark ? ldrRawDark : ldrRawBright) = value;
    prefs.putUShort("ldrd", static_cast<uint16_t>(ldrRawDark));
    prefs.putUShort("ldrb", static_cast<uint16_t>(ldrRawBright));
    Serial.printf("[LDR] %s end set to raw=%d (dark=%d bright=%d) saved\n", dark ? "DARK" : "BRIGHT", value, ldrRawDark, ldrRawBright);
}

// ดินแห้งลงเรื่อยๆ และเปียกขึ้นเมื่อปั๊มทำงาน (ระบบป้อนกลับแบบง่าย)
void updateSoil(float dtSeconds)
{
    const float rate = pumpOn ? app::SOIL_WET_RATE : -app::SOIL_DRY_RATE;
    soilPercent = constrain(soilPercent + rate * dtSeconds, 0.0f, 100.0f);
}

// ---------------------------------------------------------------------
// ค่าตั้งที่ปรับจากหน้า SETUP
// ---------------------------------------------------------------------
int soilWetTarget()
{
    return min(95, cfg.soilDry + app::SOIL_HYSTERESIS_PERCENT);
}

int fanOnMax()
{
    return static_cast<int>(fminf(app::THRESHOLD_MAX_C, app::FAN_FULL_TEMP_C - 1.0f));
}

// บังคับให้ค่าที่โหลดมาสมเหตุสมผล (เช่น เกณฑ์ปิด < เกณฑ์เปิด)
void normalizeSettings()
{
    cfg.fanOn = constrain(cfg.fanOn, static_cast<int>(app::THRESHOLD_MIN_C) + 1, fanOnMax());
    cfg.fanOff = constrain(cfg.fanOff, static_cast<int>(app::THRESHOLD_MIN_C), cfg.fanOn - 1);
    cfg.lightThr = constrain(cfg.lightThr, 5, 95);
    cfg.lightFrom = constrain(cfg.lightFrom, 0, 23);
    cfg.lightTo = constrain(cfg.lightTo, cfg.lightFrom + 1, 24);
    cfg.soilDry = constrain(cfg.soilDry, 10, 75);
    cfg.water1 = constrain(cfg.water1, 0, 23);
    cfg.water2 = constrain(cfg.water2, 0, 23);
    cfg.runSec = constrain(cfg.runSec, app::PUMP_RUN_MIN_SEC, app::PUMP_MAX_RUN_SEC);
}

const char *settingName(Setting s)
{
    switch (s) {
        case Setting::FanOn: return "FAN ON >=";
        case Setting::FanOff: return "FAN OFF <=";
        case Setting::LightThr: return "LIGHT <";
        case Setting::LightFrom: return "LIGHT FROM";
        case Setting::LightTo: return "LIGHT TO";
        case Setting::SoilDry: return "SOIL DRY <";
        case Setting::Water1: return "WATER 1";
        case Setting::Water2: return "WATER 2";
        case Setting::RunSec: return "RUN TIME";
        case Setting::None: break;
    }
    return "";
}

int settingValue(Setting s)
{
    switch (s) {
        case Setting::FanOn: return cfg.fanOn;
        case Setting::FanOff: return cfg.fanOff;
        case Setting::LightThr: return cfg.lightThr;
        case Setting::LightFrom: return cfg.lightFrom;
        case Setting::LightTo: return cfg.lightTo;
        case Setting::SoilDry: return cfg.soilDry;
        case Setting::Water1: return cfg.water1;
        case Setting::Water2: return cfg.water2;
        case Setting::RunSec: return cfg.runSec;
        case Setting::None: break;
    }
    return 0;
}

void settingText(Setting s, char *out, size_t size)
{
    const int v = settingValue(s);
    switch (s) {
        case Setting::FanOn:
        case Setting::FanOff: snprintf(out, size, "%d C", v); break;
        case Setting::LightThr:
        case Setting::SoilDry: snprintf(out, size, "%d %%", v); break;
        case Setting::LightFrom:
        case Setting::LightTo:
        case Setting::Water1:
        case Setting::Water2: snprintf(out, size, "%02d:00", v); break;
        case Setting::RunSec: snprintf(out, size, "%d s", v); break;
        case Setting::None: out[0] = '\0'; break;
    }
}

// แต่ละหน้าย่อยของ SETUP มี 5 แถว
Setting settingAtRow(int sub, int row)
{
    static const Setting table[2][5] = {
        {Setting::FanOn, Setting::FanOff, Setting::LightThr, Setting::LightFrom, Setting::LightTo},
        {Setting::SoilDry, Setting::Water1, Setting::Water2, Setting::RunSec, Setting::None},
    };
    return table[sub & 1][row];
}

// เปลี่ยนค่าทีละ 1 ขั้น (เกณฑ์เปอร์เซ็นต์และเวลารดทีละ 5) รักษาความสัมพันธ์ของค่า เช่น ปิด < เปิด
void adjustSetting(Setting s, int direction)
{
    const int before = settingValue(s);
    switch (s) {
        case Setting::FanOn: cfg.fanOn = constrain(cfg.fanOn + direction, cfg.fanOff + 1, fanOnMax()); break;
        case Setting::FanOff: cfg.fanOff = constrain(cfg.fanOff + direction, static_cast<int>(app::THRESHOLD_MIN_C), cfg.fanOn - 1); break;
        case Setting::LightThr: cfg.lightThr = constrain(cfg.lightThr + 5 * direction, 5, 95); break;
        case Setting::LightFrom: cfg.lightFrom = constrain(cfg.lightFrom + direction, 0, cfg.lightTo - 1); break;
        case Setting::LightTo: cfg.lightTo = constrain(cfg.lightTo + direction, cfg.lightFrom + 1, 24); break;
        case Setting::SoilDry: cfg.soilDry = constrain(cfg.soilDry + 5 * direction, 10, 75); break;
        case Setting::Water1: cfg.water1 = constrain(cfg.water1 + direction, 0, 23); break;
        case Setting::Water2: cfg.water2 = constrain(cfg.water2 + direction, 0, 23); break;
        case Setting::RunSec: cfg.runSec = constrain(cfg.runSec + 5 * direction, app::PUMP_RUN_MIN_SEC, app::PUMP_MAX_RUN_SEC); break;
        case Setting::None: return;
    }
    if (settingValue(s) != before) {
        settingsDirty = true;
        char text[16];
        settingText(s, text, sizeof(text));
        Serial.printf("[CFG] %s = %s\n", settingName(s), text);
    }
}

// ---------------------------------------------------------------------
// ตรรกะควบคุม 3 อุปกรณ์
// ---------------------------------------------------------------------
const char *deviceModeName(const Device &d)
{
    return d.mode == Mode::Auto ? "AUTO" : "MANUAL";
}

const char *pumpReasonName(PumpReason r)
{
    switch (r) {
        case PumpReason::Schedule: return "SCHEDULE";
        case PumpReason::Soil: return "SOIL DRY";
        case PumpReason::Manual: return "MANUAL";
        case PumpReason::None: break;
    }
    return "-";
}

// พัดลม: duty เพิ่มตามอุณหภูมิจาก FAN_MIN_DUTY ที่เกณฑ์เปิด ไปถึง 100% ที่ FAN_FULL_TEMP_C
int dutyForTemp(float tempC)
{
    const float span = app::FAN_FULL_TEMP_C - static_cast<float>(cfg.fanOn);
    const float fraction = constrain((tempC - static_cast<float>(cfg.fanOn)) / span, 0.0f, 1.0f);
    return app::FAN_MIN_DUTY_PERCENT + static_cast<int>(lroundf(fraction * (100 - app::FAN_MIN_DUTY_PERCENT)));
}

void updateFan(uint32_t now)
{
    const int before = fanDuty;
    const bool wasFailsafe = fanFailsafe;
    const float tempC = effectiveTemp(now);
    int duty = 0;
    fanFailsafe = false;

    if (fanDev.mode == Mode::Manual) {
        duty = fanDev.manualOn ? app::MANUAL_FAN_PERCENT : 0;
    } else if (isnan(tempC)) {
        fanAutoOn = false;
        if (sensorError) {  // เซนเซอร์เสียและไม่ได้ใช้ SIM: หมุนที่ค่า fail-safe ดีกว่าปล่อยให้โรงเรือนร้อน
            fanFailsafe = true;
            duty = app::SENSOR_FAIL_FAN_PERCENT;
        }
    } else {
        if (!fanAutoOn && tempC >= static_cast<float>(cfg.fanOn)) {
            fanAutoOn = true;
        } else if (fanAutoOn && tempC <= static_cast<float>(cfg.fanOff)) {
            fanAutoOn = false;
        }
        duty = fanAutoOn ? dutyForTemp(tempC) : 0;
    }
    applyFanDuty(duty);

    if ((before > 0) != (fanDuty > 0) || fanFailsafe != wasFailsafe) {
        publishEvent(fanDuty > 0 ? "fan_on" : "fan_off", fanFailsafe ? "FAILSAFE" : deviceModeName(fanDev));
        char stamp[24];
        makeTimestamp(stamp, sizeof(stamp));
        if (fanFailsafe) {
            Serial.printf("[FAN] %d%% FAIL-SAFE (no valid temperature, sensor error)  @ %s\n", fanDuty, stamp);
        } else if (isnan(tempC)) {
            Serial.printf("[FAN] %s (%s)  @ %s\n", fanDuty > 0 ? "ON" : "OFF", deviceModeName(fanDev), stamp);
        } else {
            Serial.printf("[FAN] %s %d%% (%s) at %.1f C  @ %s\n", fanDuty > 0 ? "ON" : "OFF", fanDuty, deviceModeName(fanDev), tempC, stamp);
        }
    }
}

// ไฟปลูกอยู่ในช่วงเวลาที่อนุญาตหรือไม่ (ยังไม่มีเวลาจริง = ไม่จำกัดช่วงเวลา ใช้แต่แสง)
bool inLightWindow()
{
    if (!timeValid()) return true;
    const time_t t = time(nullptr);
    struct tm local;
    localtime_r(&t, &local);
    return local.tm_hour >= cfg.lightFrom && local.tm_hour < cfg.lightTo;
}

void updateLight(uint32_t now)
{
    bool want = lightOn;
    if (lightDev.mode == Mode::Manual) {
        want = lightDev.manualOn;
    } else {
        const float ambient = ambientPercent(now);
        const bool window = inLightWindow();
        if (isnan(ambient)) {
            want = false;
        } else if (!lightOn) {
            want = window && ambient < static_cast<float>(cfg.lightThr);
        } else {  // กำลังเปิดอยู่: ดับเมื่อออกนอกช่วงเวลา หรือแสงสูงกว่าเกณฑ์ + hysteresis
            want = window && ambient < static_cast<float>(cfg.lightThr + app::LIGHT_HYSTERESIS_PERCENT);
        }
    }
    if (want != lightOn) {
        setLightOutput(want);
        publishEvent(want ? "light_on" : "light_off", deviceModeName(lightDev));
        char stamp[24];
        makeTimestamp(stamp, sizeof(stamp));
        Serial.printf("[LIGHT] %s (%s) ambient=%.0f%% window=%s  @ %s\n", want ? "ON" : "OFF", deviceModeName(lightDev), ambientPercent(now),
                      inLightWindow() ? "yes" : "no", stamp);
    }
}

bool pumpResting(uint32_t now)
{
    return pumpLocked && static_cast<int32_t>(pumpLockUntil - now) > 0;
}

void startPump(uint32_t now, PumpReason reason, uint32_t limitMs)
{
    pumpReason = reason;
    pumpStartedAt = now;
    pumpRunLimitMs = limitMs;
    setPumpOutput(true);
    publishEvent("pump_on", pumpReasonName(reason));
    char stamp[24];
    makeTimestamp(stamp, sizeof(stamp));
    Serial.printf("[PUMP] ON  reason=%s limit=%lu s soil=%.0f%%  @ %s\n", pumpReasonName(reason), static_cast<unsigned long>(limitMs / 1000),
                  soilPercent, stamp);
}

void stopPump(uint32_t now, const char *why)
{
    const uint32_t ranMs = now - pumpStartedAt;
    setPumpOutput(false);
    publishEvent("pump_off", why);
    pumpLocked = true;
    pumpLockUntil = now + static_cast<uint32_t>(app::PUMP_COOLDOWN_SEC) * 1000UL;
    char stamp[24];
    makeTimestamp(stamp, sizeof(stamp));
    Serial.printf("[PUMP] OFF (%s) ran %lu s, rest %d s, soil=%.0f%%  @ %s\n", why, static_cast<unsigned long>(ranMs / 1000), app::PUMP_COOLDOWN_SEC,
                  soilPercent, stamp);
    pumpReason = PumpReason::None;
}

// รอบรดน้ำที่ "ถึงเวลาและยังไม่ได้ทำของวันนี้" คืนลำดับ 0/1 หรือ -1 ถ้าไม่มี (dayKey = เลขวันตามเวลาท้องถิ่น)
int dueWaterSlot(long &dayKey)
{
    if (!timeValid()) return -1;
    const time_t t = time(nullptr);
    struct tm local;
    localtime_r(&t, &local);
    dayKey = (static_cast<long>(t) + app::GMT_OFFSET_SEC + app::DAYLIGHT_OFFSET_SEC) / 86400;
    const int minutes = local.tm_hour * 60 + local.tm_min;
    const int hours[2] = {cfg.water1, cfg.water2};
    for (int i = 0; i < 2; i++) {
        const int start = hours[i] * 60;
        if (slotDoneDay[i] != dayKey && minutes >= start && minutes < start + app::WATER_SLOT_GRACE_MIN) {
            return i;
        }
    }
    return -1;
}

void updatePump(uint32_t now)
{
    if (pumpLocked && !pumpResting(now)) {
        pumpLocked = false;
    }

    // ด่านที่ 1 (สำคัญที่สุด): ปั๊มทำงานต่อเนื่องเกิน PUMP_MAX_RUN_SEC ไม่ได้ ไม่ว่าจะสั่งมาจากไหน
    if (pumpOn && now - pumpStartedAt >= static_cast<uint32_t>(app::PUMP_MAX_RUN_SEC) * 1000UL) {
        stopPump(now, "MAX RUN watchdog");
        if (pumpDev.mode == Mode::Manual) pumpDev.manualOn = false;
        return;
    }

    if (pumpDev.mode == Mode::Manual) {
        if (pumpOn) {
            if (!pumpDev.manualOn) {
                stopPump(now, "manual OFF");
            } else {
                pumpReason = PumpReason::Manual;  // รับช่วงต่อจากรอบที่ AUTO เริ่มไว้
            }
        } else if (pumpDev.manualOn) {
            if (pumpResting(now)) {
                pumpDev.manualOn = false;  // กำลังพักปั๊ม: ปฏิเสธคำสั่ง
                Serial.printf("[PUMP] refused: resting for %ld more s\n", static_cast<long>((pumpLockUntil - now) / 1000UL) + 1);
            } else {
                startPump(now, PumpReason::Manual, static_cast<uint32_t>(app::PUMP_MAX_RUN_SEC) * 1000UL);
            }
        }
        return;
    }

    // โหมด AUTO
    if (pumpOn) {
        const uint32_t elapsed = now - pumpStartedAt;
        if (pumpReason == PumpReason::Manual) {
            stopPump(now, "back to AUTO");
        } else if (pumpReason == PumpReason::Schedule && elapsed >= pumpRunLimitMs) {
            stopPump(now, "schedule finished");
        } else if (pumpReason == PumpReason::Soil && soilPercent >= static_cast<float>(soilWetTarget())) {
            stopPump(now, "soil wet enough");
        }
    } else if (!pumpResting(now)) {
        long dayKey = 0;
        const int slot = dueWaterSlot(dayKey);
        if (slot >= 0) {
            const int hours[2] = {cfg.water1, cfg.water2};
            for (int i = 0; i < 2; i++) {
                if (hours[i] == hours[slot]) slotDoneDay[i] = dayKey;  // รอบที่เวลาเดียวกันถือว่ารดแล้วพร้อมกัน
            }
            startPump(now, PumpReason::Schedule, static_cast<uint32_t>(cfg.runSec) * 1000UL);
        } else if (soilPercent < static_cast<float>(cfg.soilDry)) {
            startPump(now, PumpReason::Soil, static_cast<uint32_t>(app::PUMP_MAX_RUN_SEC) * 1000UL);
        }
    }
}

void updateControl(uint32_t now)
{
    updateFan(now);
    updateLight(now);
    updatePump(now);
}

// เปลี่ยนโหมดของอุปกรณ์ AUTO <-> MANUAL (ไป MANUAL ให้คงสถานะปัจจุบันไว้ก่อน)
void toggleDeviceMode(Device &d, bool currentlyOn, const char *name)
{
    if (d.mode == Mode::Auto) {
        d.mode = Mode::Manual;
        d.manualOn = currentlyOn;
    } else {
        d.mode = Mode::Auto;
    }
    Serial.printf("[MODE] %s -> %s\n", name, deviceModeName(d));
    char detail[40];
    snprintf(detail, sizeof(detail), "%s %s", name, deviceModeName(d));
    publishEvent("mode", detail);
}

// สั่ง ON/OFF แบบ MANUAL (ถ้ายังอยู่ AUTO จะสลับเป็น MANUAL ให้เอง)
void toggleDeviceManual(Device &d, bool currentlyOn, const char *name)
{
    if (d.mode == Mode::Auto) {
        d.mode = Mode::Manual;
        d.manualOn = !currentlyOn;
    } else {
        d.manualOn = !d.manualOn;
    }
    Serial.printf("[MODE] %s -> MANUAL %s\n", name, d.manualOn ? "ON" : "OFF");
    char detail[40];
    snprintf(detail, sizeof(detail), "%s MANUAL %s", name, d.manualOn ? "ON" : "OFF");
    publishEvent("mode", detail);
}

void setAllAuto()
{
    fanDev.mode = lightDev.mode = pumpDev.mode = Mode::Auto;
    Serial.println("[MODE] all devices -> AUTO");
    publishEvent("all_auto", "all devices AUTO");
}

// หยุดฉุกเฉิน: ทุกอุปกรณ์เป็น MANUAL + OFF (ปุ่ม BOOT)
void emergencyStop()
{
    fanDev.mode = lightDev.mode = pumpDev.mode = Mode::Manual;
    fanDev.manualOn = lightDev.manualOn = pumpDev.manualOn = false;
    Serial.println("[MODE] EMERGENCY STOP: all devices MANUAL + OFF");
    publishEvent("estop", "all devices MANUAL OFF");
}

// ---------------------------------------------------------------------
// MQTT: ส่งข้อมูลขึ้น broker และรับคำสั่งกลับมา
//   หลักการ: MQTT เป็นแบบ publish/subscribe ผ่านคนกลาง (broker) ไม่ต้องรู้ IP ของบอร์ด
//   ข้อควรจำ: การเชื่อมต่อ (connect) ใช้เวลาและอาจรอ timeout ตัวควบคุมจึงลองใหม่แบบห่างขึ้นเรื่อยๆ (backoff)
//   และเราวัดเวลาที่โปรแกรมหยุดรอไว้แสดงในหน้า MQTT
// ---------------------------------------------------------------------
void mqttTopic(char *out, size_t size, const char *suffix)
{
    snprintf(out, size, "%s/%s", mqttRoot, suffix);
}

bool mqttSend(const char *suffix, const char *payload, bool retained)
{
    char topic[72];
    mqttTopic(topic, sizeof(topic), suffix);
    const bool ok = mqtt.publish(topic, payload, retained);
    if (ok) {
        mqttTx++;
    } else {
        Serial.printf("[MQTT] publish to %s failed (message too large or connection lost)\n", suffix);
    }
    return ok;
}

// แจ้งเหตุการณ์ (ปั๊มเริ่ม/หยุด ฯลฯ) ไปที่ <root>/event ถ้ายังไม่ได้เชื่อมต่อจะนับเป็น dropped (ไม่เก็บคิว)
void publishEvent(const char *name, const char *detail)
{
    if (!mqtt.connected()) {
        mqttDropped++;
        return;
    }
    char stamp[24];
    makeTimestamp(stamp, sizeof(stamp));
    char json[200];
    snprintf(json, sizeof(json), "{\"event\":\"%s\",\"detail\":\"%s\",\"ts\":\"%s\",\"uptime_s\":%lu}", name, detail, stamp,
             static_cast<unsigned long>(millis() / 1000UL));
    mqttSend("event", json, false);
}

// ส่งค่าเซนเซอร์และสถานะทั้งหมดเป็น JSON ไปที่ <root>/telemetry (retained = ผู้ที่เพิ่ง subscribe เห็นค่าล่าสุดทันที)
void publishTelemetry(uint32_t now)
{
    char stamp[24];
    makeTimestamp(stamp, sizeof(stamp));
    const float tempC = effectiveTemp(now);
    const float humidity = effectiveHumidity(now);
    const float ambient = ambientPercent(now);
    char t[12], h[12], l[12];
    if (isnan(tempC)) snprintf(t, sizeof(t), "null"); else snprintf(t, sizeof(t), "%.1f", tempC);
    if (isnan(humidity)) snprintf(h, sizeof(h), "null"); else snprintf(h, sizeof(h), "%.0f", humidity);
    if (isnan(ambient)) snprintf(l, sizeof(l), "null"); else snprintf(l, sizeof(l), "%.0f", ambient);
    const long restSeconds = pumpResting(now) ? static_cast<long>((pumpLockUntil - now) / 1000UL + 1) : 0;

    char json[420];
    const int n = snprintf(json, sizeof(json),
                           "{\"ts\":\"%s\",\"uptime_s\":%lu,\"temp\":%s,\"hum\":%s,\"light\":%s,\"soil\":%.0f,\"src\":\"%s\","
                           "\"fan\":{\"duty\":%d,\"mode\":\"%s\",\"failsafe\":%s},\"lamp\":{\"on\":%s,\"mode\":\"%s\"},"
                           "\"pump\":{\"on\":%s,\"mode\":\"%s\",\"rest_s\":%ld},\"remote\":%s,\"rssi\":%d}",
                           stamp, static_cast<unsigned long>(now / 1000UL), t, h, l, soilPercent, sourceName(currentSource()), fanDuty,
                           deviceModeName(fanDev), fanFailsafe ? "true" : "false", lightOn ? "true" : "false", deviceModeName(lightDev),
                           pumpOn ? "true" : "false", deviceModeName(pumpDev), restSeconds, remoteEnabled ? "true" : "false", WiFi.RSSI());
    if (n <= 0 || n >= static_cast<int>(sizeof(json))) {
        Serial.println("[MQTT] telemetry JSON too long: not sent");
        return;
    }
    mqttSend("telemetry", json, true);
}

void setRemote(bool on, uint32_t now)
{
    remoteEnabled = on;
    remoteUntil = now + app::REMOTE_ENABLE_MS;
    if (on) {
        Serial.printf("[MQTT] remote commands ENABLED for %lu min (they will switch off by themselves)\n",
                      static_cast<unsigned long>(app::REMOTE_ENABLE_MS / 60000UL));
    } else {
        Serial.println("[MQTT] remote commands DISABLED");
    }
    publishEvent("remote", on ? "enabled" : "disabled");
}

void updateRemoteExpiry(uint32_t now)
{
    if (remoteEnabled && static_cast<int32_t>(now - remoteUntil) >= 0) {
        setRemote(false, now);
    }
}

// คัดลอกข้อความที่มาจากเครือข่ายอย่างปลอดภัย: ตัดความยาว และแทนอักขระแปลกๆ ด้วย '?' (ห้ามเชื่อข้อมูลจากภายนอก)
void copySafe(char *out, size_t size, const char *src, size_t length, bool upper)
{
    size_t n = 0;
    for (size_t i = 0; i < length && n + 1 < size; i++) {
        char c = src[i];
        if (upper && c >= 'a' && c <= 'z') c = static_cast<char>(c - 'a' + 'A');
        const bool ok = (c >= 'A' && c <= 'Z') || (c >= 'a' && c <= 'z') || (c >= '0' && c <= '9') || c == '_' || c == '-';
        out[n++] = ok ? c : '?';
    }
    out[n] = '\0';
}

// ตอบผลคำสั่งที่ <root>/ack และจดไว้แสดงบนหน้าจอ
void mqttReply(const char *cmd, const char *data, const char *result)
{
    char json[160];
    snprintf(json, sizeof(json), "{\"cmd\":\"%s\",\"payload\":\"%s\",\"result\":\"%s\"}", cmd, data, result);
    mqttSend("ack", json, false);
    snprintf(lastCmdText, sizeof(lastCmdText), "%s %s: %s", cmd, data, result);
    lastCmdSeq++;
    Serial.printf("[MQTT] command %s\n", lastCmdText);
}

// มีข้อความเข้ามาจาก topic ที่ subscribe ไว้ (<root>/cmd/#)
// ข้อควรระวัง: topic และ payload ชี้เข้าบัฟเฟอร์ของไลบรารี การ publish ตอบกลับจะเขียนทับ จึงคัดลอกออกมาก่อน
void mqttCallback(char *topic, uint8_t *payload, unsigned int length)
{
    char prefix[56];
    snprintf(prefix, sizeof(prefix), "%s/cmd/", mqttRoot);
    const size_t prefixLen = strlen(prefix);
    if (strncmp(topic, prefix, prefixLen) != 0) return;

    char cmd[16];
    char data[16];
    copySafe(cmd, sizeof(cmd), topic + prefixLen, strlen(topic + prefixLen), false);
    copySafe(data, sizeof(data), reinterpret_cast<const char *>(payload), length, true);
    mqttRx++;
    const uint32_t now = millis();

    if (!remoteEnabled) {
        mqttReply(cmd, data, "rejected: REMOTE is OFF");
        return;
    }

    int index = -1;
    if (strcmp(cmd, "fan") == 0) index = 0;
    else if (strcmp(cmd, "light") == 0) index = 1;
    else if (strcmp(cmd, "pump") == 0) index = 2;

    if (index >= 0) {
        Device &d = (index == 0) ? fanDev : (index == 1 ? lightDev : pumpDev);
        if (strcmp(data, "AUTO") == 0) {
            d.mode = Mode::Auto;
        } else if (strcmp(data, "ON") == 0 || strcmp(data, "OFF") == 0) {
            const bool wantOn = strcmp(data, "ON") == 0;
            if (index == 2 && wantOn && pumpResting(now)) {
                char refusal[48];
                snprintf(refusal, sizeof(refusal), "refused: pump resting %lu s", static_cast<unsigned long>((pumpLockUntil - now) / 1000UL + 1));
                mqttReply(cmd, data, refusal);
                return;
            }
            d.mode = Mode::Manual;
            d.manualOn = wantOn;
        } else {
            mqttReply(cmd, data, "rejected: use ON/OFF/AUTO");
            return;
        }
        updateControl(now);
        mqttReply(cmd, data, "accepted");
        publishEvent("remote_cmd", lastCmdText);
    } else if (strcmp(cmd, "estop") == 0) {
        emergencyStop();
        updateControl(now);
        mqttReply(cmd, data, "accepted: all MANUAL OFF");
    } else {
        mqttReply(cmd, data, "rejected: unknown command");
    }
}

void mqttBegin()
{
    uint8_t mac[6] = {0, 0, 0, 0, 0, 0};
    esp_read_mac(mac, ESP_MAC_WIFI_STA);
    snprintf(mqttRoot, sizeof(mqttRoot), "%s/%02x%02x%02x", app::MQTT_TOPIC_PREFIX, mac[3], mac[4], mac[5]);
    snprintf(mqttClientId, sizeof(mqttClientId), "cydfarm-%02x%02x%02x", mac[3], mac[4], mac[5]);  // ห้ามซ้ำกับใครบน broker เดียวกัน
    mqtt.setServer(app::MQTT_HOST, app::MQTT_PORT);
    mqtt.setCallback(mqttCallback);
    mqtt.setKeepAlive(app::MQTT_KEEPALIVE_S);
    mqtt.setSocketTimeout(app::MQTT_SOCKET_TIMEOUT_S);
    mqttNet.setTimeout(app::MQTT_SOCKET_TIMEOUT_S);  // รอการเชื่อมต่อ TCP นานสุดกี่วินาที (ค่าเริ่มต้นของ WiFiClient คือ 3)
    mqtt.setBufferSize(512);
    Serial.printf("[MQTT] broker %s:%u (no password, public)  client id %s\n", app::MQTT_HOST, app::MQTT_PORT, mqttClientId);
    Serial.printf("[MQTT] root topic: %s   (publishes: status telemetry event ack  |  subscribes: cmd/fan cmd/light cmd/pump cmd/estop)\n", mqttRoot);
}

const char *mqttStateName(uint32_t now)
{
    switch (mqttState) {
        case MqttState::NoWifi: return "NO WIFI";
        case MqttState::Waiting: return (static_cast<int32_t>(mqttNextAttemptAt - now) > 0) ? "RETRY" : "CONNECTING";
        case MqttState::Connected: return "CONNECTED";
    }
    return "?";
}

// ลองเชื่อมต่อ broker หนึ่งครั้ง (ฟังก์ชันนี้ "หยุดรอ" จนกว่าจะสำเร็จหรือ timeout จึงวัดเวลาไว้)
void mqttConnect(uint32_t now)
{
    char willTopic[72];
    mqttTopic(willTopic, sizeof(willTopic), "status");
    const uint32_t startedAt = millis();
    const bool ok = mqtt.connect(mqttClientId, nullptr, nullptr, willTopic, 0, true, "offline");  // LWT: broker ประกาศ offline แทนเมื่อบอร์ดหลุด
    mqttLastBlockMs = millis() - startedAt;
    if (mqttLastBlockMs > mqttMaxBlockMs) mqttMaxBlockMs = mqttLastBlockMs;
    mqttLastRc = mqtt.state();

    if (ok) {
        char sub[72];
        mqttTopic(sub, sizeof(sub), "cmd/#");
        mqtt.subscribe(sub);
        mqttState = MqttState::Connected;
        mqttBackoffMs = app::MQTT_RECONNECT_MIN_MS;
        mqttSend("status", "online", true);
        lastTelemetryAt = now - app::MQTT_TELEMETRY_MS;  // ส่งค่าแรกทันที
        Serial.printf("[MQTT] connected (took %lu ms), subscribed to %s\n", static_cast<unsigned long>(mqttLastBlockMs), sub);
    } else {
        mqttConnectFails++;
        mqttNextAttemptAt = millis() + mqttBackoffMs;
        Serial.printf("[MQTT] connect failed rc=%d (took %lu ms), retry in %lu s\n", mqttLastRc, static_cast<unsigned long>(mqttLastBlockMs),
                      static_cast<unsigned long>(mqttBackoffMs / 1000UL));
        mqttBackoffMs = min<uint32_t>(mqttBackoffMs * 2, app::MQTT_RECONNECT_MAX_MS);
    }
}

void updateMqtt(uint32_t now)
{
    if (wifiState != WifiState::Connected) {
        if (mqttState != MqttState::NoWifi) {
            Serial.println("[MQTT] WiFi lost: MQTT stopped");
            mqtt.disconnect();
            mqttState = MqttState::NoWifi;
        }
        return;
    }
    if (mqttState == MqttState::NoWifi) {
        mqttState = MqttState::Waiting;
        mqttNextAttemptAt = now + 1000;  // รอให้ WiFi/DNS พร้อมสักครู่
    }

    if (mqttState == MqttState::Connected && !mqtt.connected()) {
        Serial.printf("[MQTT] connection lost (rc=%d), will reconnect\n", mqtt.state());
        mqttState = MqttState::Waiting;
        mqttNextAttemptAt = now + app::MQTT_RECONNECT_MIN_MS;
    }

    if (mqttState == MqttState::Waiting) {
        if (static_cast<int32_t>(now - mqttNextAttemptAt) >= 0) {
            mqttConnect(now);
        }
        return;
    }

    mqtt.loop();  // รับข้อความ (เรียก mqttCallback) และส่ง ping ตาม keep-alive
    if (now - lastTelemetryAt >= app::MQTT_TELEMETRY_MS) {
        lastTelemetryAt = now;
        publishTelemetry(now);
    }
}

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
// จอสัมผัส XPT2046: อ่านค่าดิบผ่าน SPI
// ---------------------------------------------------------------------

// เลือกค่าเฉลี่ยของ "สองค่าที่ใกล้กันที่สุด" จากสามค่า (ตัดค่าสุ่มที่เพี้ยน)
int16_t bestTwoAverage(int16_t a, int16_t b, int16_t c)
{
    const int16_t dab = abs(a - b);
    const int16_t dac = abs(a - c);
    const int16_t dbc = abs(b - c);
    if (dab <= dac && dab <= dbc) return (a + b) >> 1;
    if (dac <= dab && dac <= dbc) return (a + c) >> 1;
    return (b + c) >> 1;
}

// อ่านชิป XPT2046 หนึ่งรอบ คืน true ถ้ามีการแตะ (แรงกด >= TOUCH_Z_MIN)
// ลำดับคำสั่ง: Z1, Z2 -> คำนวณแรงกด; ถ้ากดอยู่ อ่านแกน X และ Y หลายครั้งแล้วเลือกค่าที่ใกล้กัน
// (ผลของแต่ละคำสั่งจะมาในรอบส่งถัดไป จึงอ่านเหลื่อมกัน 1 คำสั่ง)
bool readRawTouch(RawTouch &out)
{
    int16_t data[6] = {0, 0, 0, 0, 0, 0};

    touchSpi.beginTransaction(SPISettings(cyd::TOUCH_SPI_FREQUENCY, MSBFIRST, SPI_MODE0));
    digitalWrite(cyd::TOUCH_PIN_CS, LOW);
    touchSpi.transfer(0xB1);                                 // เริ่มวัด Z1
    const int16_t z1 = static_cast<int16_t>(touchSpi.transfer16(0xC1) >> 3);  // ได้ผล Z1, เริ่มวัด Z2
    int z = z1 + 4095;
    const int16_t z2 = static_cast<int16_t>(touchSpi.transfer16(0x91) >> 3);  // ได้ผล Z2, เริ่มวัดแกนแรก
    z -= z2;                                                  // แรงกด = Z1 + 4095 - Z2
    if (z >= app::TOUCH_Z_MIN) {
        touchSpi.transfer16(0x91);                            // ค่าแรกมักมีสัญญาณรบกวน ทิ้งไป
        data[0] = static_cast<int16_t>(touchSpi.transfer16(0xD1) >> 3);
        data[1] = static_cast<int16_t>(touchSpi.transfer16(0x91) >> 3);
        data[2] = static_cast<int16_t>(touchSpi.transfer16(0xD1) >> 3);
        data[3] = static_cast<int16_t>(touchSpi.transfer16(0x91) >> 3);
    }
    data[4] = static_cast<int16_t>(touchSpi.transfer16(0xD0) >> 3);  // คำสั่งสุดท้ายสั่งให้ชิปพักการวัด
    data[5] = static_cast<int16_t>(touchSpi.transfer16(0) >> 3);
    digitalWrite(cyd::TOUCH_PIN_CS, HIGH);
    touchSpi.endTransaction();

    out.z = static_cast<int16_t>(z < 0 ? 0 : z);
    if (z < app::TOUCH_Z_MIN) {
        return false;
    }
    out.x = bestTwoAverage(data[0], data[2], data[4]);
    out.y = bestTwoAverage(data[1], data[3], data[5]);
    return true;
}

// ---------------------------------------------------------------------
// การ calibrate: แปลงค่าดิบเป็นพิกัดจอ
// ---------------------------------------------------------------------
void mapRawToScreen(int16_t rawX, int16_t rawY, int &sx, int &sy)
{
    const float u = cal.swapXY ? rawY : rawX;  // แกนดิบที่ใช้เป็นแกน x ของจอ
    const float v = cal.swapXY ? rawX : rawY;
    sx = constrain(static_cast<int>(lroundf(cal.xa + cal.xb * u)), 0, SCREEN_W - 1);
    sy = constrain(static_cast<int>(lroundf(cal.ya + cal.yb * v)), 0, SCREEN_H - 1);
}

// เป้า 4 มุม (บนซ้าย, บนขวา, ล่างขวา, ล่างซ้าย) ห่างขอบ CAL_MARGIN พิกเซล
void calTarget(int index, int &tx, int &ty)
{
    tx = (index == 0 || index == 3) ? app::CAL_MARGIN : SCREEN_W - 1 - app::CAL_MARGIN;
    ty = (index == 0 || index == 1) ? app::CAL_MARGIN : SCREEN_H - 1 - app::CAL_MARGIN;
}

// คำนวณค่า calibrate จากค่าดิบของเป้าทั้ง 4 คืน false ถ้าข้อมูลไม่สมเหตุสมผล (เช่นแตะผิดมุม)
bool computeCalibration()
{
    // แกนดิบไหนเปลี่ยนมากกว่าเมื่อเลื่อนจากมุมบนซ้ายไปบนขวา (แกน x ของจอ) -> ถ้าเป็นแกน Y ดิบ แปลว่าแกนสลับกัน
    const int dxTop = abs(calRawX[1] - calRawX[0]);
    const int dyTop = abs(calRawY[1] - calRawY[0]);
    TouchCalibration c;
    c.swapXY = dyTop > dxTop;

    auto rawU = [&](int i) { return static_cast<float>(c.swapXY ? calRawY[i] : calRawX[i]); };
    auto rawV = [&](int i) { return static_cast<float>(c.swapXY ? calRawX[i] : calRawY[i]); };
    const float leftU = (rawU(0) + rawU(3)) / 2.0f;
    const float rightU = (rawU(1) + rawU(2)) / 2.0f;
    const float topV = (rawV(0) + rawV(1)) / 2.0f;
    const float bottomV = (rawV(2) + rawV(3)) / 2.0f;
    const float spanU = rightU - leftU;
    const float spanV = bottomV - topV;
    if (fabsf(spanU) < app::CAL_MIN_SPAN || fabsf(spanV) < app::CAL_MIN_SPAN) {
        Serial.printf("[CAL] raw span too small (u=%.0f v=%.0f): did you touch the crosses?\n", spanU, spanV);
        return false;
    }

    int leftX, topY, rightX, bottomY;
    calTarget(0, leftX, topY);
    calTarget(2, rightX, bottomY);
    c.xb = static_cast<float>(rightX - leftX) / spanU;  // พิกเซลต่อหน่วยดิบ (เครื่องหมายลบ = แกนกลับด้าน)
    c.xa = static_cast<float>(leftX) - c.xb * leftU;
    c.yb = static_cast<float>(bottomY - topY) / spanV;
    c.ya = static_cast<float>(topY) - c.yb * topV;

    // ตรวจย้อนกลับ: เอาค่าดิบของแต่ละเป้ามาแปลงเป็นพิกัดจอ ต้องใกล้เป้าจริง
    float worst = 0.0f;
    for (int i = 0; i < 4; i++) {
        int tx, ty;
        calTarget(i, tx, ty);
        const float sx = c.xa + c.xb * rawU(i);
        const float sy = c.ya + c.yb * rawV(i);
        const float err = fmaxf(fabsf(sx - tx), fabsf(sy - ty));
        if (err > worst) worst = err;
    }
    Serial.printf("[CAL] swapXY=%s  x: %.4f*u%+.1f  y: %.4f*v%+.1f  worst target error %.1f px\n", c.swapXY ? "yes" : "no", c.xb, c.xa,
                  c.yb, c.ya, worst);
    if (worst > app::CAL_MAX_ERROR_PX) {
        Serial.println("[CAL] points are not consistent with a rectangle: please try again");
        return false;
    }
    c.valid = true;
    cal = c;
    return true;
}

void saveCalibration()
{
    prefs.putBool("calok", cal.valid);
    prefs.putBool("swap", cal.swapXY);
    prefs.putFloat("xa", cal.xa);
    prefs.putFloat("xb", cal.xb);
    prefs.putFloat("ya", cal.ya);
    prefs.putFloat("yb", cal.yb);
    Serial.println("[CAL] saved to flash (Preferences)");
}

// ---------------------------------------------------------------------
// ค่าที่เก็บถาวร (Preferences namespace "lab11"): calibrate, ความสว่างจอ, เกณฑ์และตารางเวลา
// ---------------------------------------------------------------------
void saveSettings()
{
    prefs.putUChar("fon", static_cast<uint8_t>(cfg.fanOn));
    prefs.putUChar("foff", static_cast<uint8_t>(cfg.fanOff));
    prefs.putUChar("lthr", static_cast<uint8_t>(cfg.lightThr));
    prefs.putUChar("lfrom", static_cast<uint8_t>(cfg.lightFrom));
    prefs.putUChar("lto", static_cast<uint8_t>(cfg.lightTo));
    prefs.putUChar("sdry", static_cast<uint8_t>(cfg.soilDry));
    prefs.putUChar("w1", static_cast<uint8_t>(cfg.water1));
    prefs.putUChar("w2", static_cast<uint8_t>(cfg.water2));
    prefs.putUChar("prun", static_cast<uint8_t>(cfg.runSec));
    settingsDirty = false;
    Serial.println("[CFG] saved to flash (Preferences)");
}

void loadSettings()
{
    prefs.begin("lab11", false);
    if (prefs.getBool("calok", false)) {
        cal.valid = true;
        cal.swapXY = prefs.getBool("swap", false);
        cal.xa = prefs.getFloat("xa", 0.0f);
        cal.xb = prefs.getFloat("xb", 0.0f);
        cal.ya = prefs.getFloat("ya", 0.0f);
        cal.yb = prefs.getFloat("yb", 0.0f);
        Serial.printf("[CAL] loaded from flash: swapXY=%s x: %.4f*u%+.1f  y: %.4f*v%+.1f\n", cal.swapXY ? "yes" : "no", cal.xb, cal.xa,
                      cal.yb, cal.ya);
    } else {
        Serial.println("[CAL] no saved calibration: the calibration screen will start");
    }
    backlightPercent = prefs.getUChar("bl", app::BACKLIGHT_DEFAULT_PERCENT);

    ldrRawDark = prefs.getUShort("ldrd", app::LDR_RAW_DARK);
    ldrRawBright = prefs.getUShort("ldrb", app::LDR_RAW_BRIGHT);
    Serial.printf("[LDR] range: dark=raw %d, bright=raw %d (%s)\n", ldrRawDark, ldrRawBright,
                  prefs.isKey("ldrd") ? "saved by SET DARK/BRIGHT" : "defaults from app_config.h");
    cfg.fanOn = prefs.getUChar("fon", cfg.fanOn);
    cfg.fanOff = prefs.getUChar("foff", cfg.fanOff);
    cfg.lightThr = prefs.getUChar("lthr", cfg.lightThr);
    cfg.lightFrom = prefs.getUChar("lfrom", cfg.lightFrom);
    cfg.lightTo = prefs.getUChar("lto", cfg.lightTo);
    cfg.soilDry = prefs.getUChar("sdry", cfg.soilDry);
    cfg.water1 = prefs.getUChar("w1", cfg.water1);
    cfg.water2 = prefs.getUChar("w2", cfg.water2);
    cfg.runSec = prefs.getUChar("prun", cfg.runSec);
    normalizeSettings();
    Serial.printf("[CFG] fan %d/%d C, light <%d%% %02d-%02d h, soil <%d%%, water %02d:00 + %02d:00 for %d s\n", cfg.fanOn, cfg.fanOff,
                  cfg.lightThr, cfg.lightFrom, cfg.lightTo, cfg.soilDry, cfg.water1, cfg.water2, cfg.runSec);
}

// ---------------------------------------------------------------------
// อ่านจอสัมผัส: กันสัญญาณรบกวน (debounce) และสร้างเหตุการณ์ กดลง / กำลังกด / ปล่อย
// ---------------------------------------------------------------------
void pollTouch(uint32_t now)
{
    touch.pressEdge = false;
    touch.releaseEdge = false;
    if (now - lastTouchPollAt < app::TOUCH_POLL_MS) return;
    lastTouchPollAt = now;

    RawTouch raw;
    const bool touching = readRawTouch(raw);
    touch.touchingNow = touching;
    touch.z = raw.z;
    if (touching) {
        touch.rawX = raw.x;
        touch.rawY = raw.y;
        if (touch.pressCount < 255) touch.pressCount++;
        touch.releaseCount = 0;
    } else {
        if (touch.releaseCount < 255) touch.releaseCount++;
        touch.pressCount = 0;
    }

    if (!touch.down && touch.pressCount >= app::TOUCH_PRESS_SAMPLES) {  // ต้องเห็นการแตะติดกันหลายครั้งถึงนับว่ากดลง
        touch.down = true;
        touch.pressEdge = true;
        touch.downSince = now;
        touch.smoothReady = false;
    }

    if (touch.down && touching && cal.valid) {
        int sx, sy;
        mapRawToScreen(raw.x, raw.y, sx, sy);
        if (!touch.smoothReady) {
            touch.x = sx;
            touch.y = sy;
            touch.smoothReady = true;
        } else {  // ตัวกรอง EMA ให้พิกัดนิ่งขึ้นขณะลากนิ้ว
            touch.x += static_cast<int>(lroundf(app::TOUCH_SMOOTH_ALPHA * static_cast<float>(sx - touch.x)));
            touch.y += static_cast<int>(lroundf(app::TOUCH_SMOOTH_ALPHA * static_cast<float>(sy - touch.y)));
        }
        touch.lastX = touch.x;
        touch.lastY = touch.y;
    }

    if (touch.down && touch.releaseCount >= app::TOUCH_RELEASE_SAMPLES) {  // ไม่เห็นการแตะติดกันหลายครั้ง = ปล่อยแล้ว
        touch.down = false;
        touch.releaseEdge = true;
    }
}


// ---------------------------------------------------------------------
// ชื่อสำหรับ Serial
// ---------------------------------------------------------------------
const char *widgetName(Widget w)
{
    switch (w) {
        case Widget::None: return "-";
        case Widget::TabDash: return "TabDash";
        case Widget::TabCtrl: return "TabCtrl";
        case Widget::TabSetup: return "TabSetup";
        case Widget::TabMqtt: return "TabMqtt";
        case Widget::TabSystem: return "TabSystem";
        case Widget::ClockCard: return "ClockCard";
        case Widget::TempCard: return "TempCard";
        case Widget::HumCard: return "HumCard";
        case Widget::LightCard: return "LightCard";
        case Widget::SoilCard: return "SoilCard";
        case Widget::FanTile: return "FanTile";
        case Widget::LightTile: return "LightTile";
        case Widget::PumpTile: return "PumpTile";
        case Widget::FanModeBtn: return "FanModeBtn";
        case Widget::FanManBtn: return "FanManBtn";
        case Widget::LightModeBtn: return "LightModeBtn";
        case Widget::LightManBtn: return "LightManBtn";
        case Widget::PumpModeBtn: return "PumpModeBtn";
        case Widget::PumpManBtn: return "PumpManBtn";
        case Widget::SetMinus0: return "SetMinus0";
        case Widget::SetMinus1: return "SetMinus1";
        case Widget::SetMinus2: return "SetMinus2";
        case Widget::SetMinus3: return "SetMinus3";
        case Widget::SetMinus4: return "SetMinus4";
        case Widget::SetPlus0: return "SetPlus0";
        case Widget::SetPlus1: return "SetPlus1";
        case Widget::SetPlus2: return "SetPlus2";
        case Widget::SetPlus3: return "SetPlus3";
        case Widget::SetPlus4: return "SetPlus4";
        case Widget::BlSlider: return "BlSlider";
        case Widget::SyncBtn: return "SyncBtn";
        case Widget::TestBtn: return "TestBtn";
        case Widget::LdrDarkBtn: return "LdrDarkBtn";
        case Widget::LdrBrightBtn: return "LdrBrightBtn";
        case Widget::RecalBtn: return "RecalBtn";
        case Widget::RemoteBtn: return "RemoteBtn";
        case Widget::TestPaint: return "TestPaint";
        case Widget::TestBack: return "TestBack";
        case Widget::TestClear: return "TestClear";
    }
    return "?";
}

const char *pageName(Page p)
{
    switch (p) {
        case Page::Dashboard: return "DASH";
        case Page::Control: return "CTRL";
        case Page::Setup: return "SETUP";
        case Page::Mqtt: return "MQTT";
        case Page::System: return "SYSTEM";
        case Page::Test: return "TEST";
        case Page::Calibrate: return "CALIBRATE";
    }
    return "?";
}

// ---------------------------------------------------------------------
// ตัวช่วยเกี่ยวกับอุปกรณ์ (ลำดับ 0 = พัดลม, 1 = ไฟปลูก, 2 = ปั๊ม)
// ---------------------------------------------------------------------
Device &deviceAt(int i)
{
    return i == 0 ? fanDev : (i == 1 ? lightDev : pumpDev);
}

const char *deviceLabel(int i)
{
    return i == 0 ? "FAN" : (i == 1 ? "LIGHT" : "PUMP");
}

bool deviceIsOn(int i)
{
    return i == 0 ? fanDuty > 0 : (i == 1 ? lightOn : pumpOn);
}

Widget deviceWidget(int i, bool manualButton)
{
    return static_cast<Widget>(static_cast<int>(Widget::FanModeBtn) + 2 * i + (manualButton ? 1 : 0));
}

// ค่าที่เปลี่ยนเมื่อต้องวาดอุปกรณ์ใหม่ (โหมด + สถานะ + วินาทีของปั๊ม)
int deviceKey(int i, uint32_t now)
{
    int state = 0;
    if (i == 0) {
        state = (fanFailsafe ? 1000 : 0) + fanDuty;
    } else if (i == 1) {
        state = lightOn ? 1 : 0;
    } else if (pumpOn) {
        state = 1000 + static_cast<int>((now - pumpStartedAt) / 1000UL);
    } else if (pumpResting(now)) {
        state = 2000 + static_cast<int>((pumpLockUntil - now) / 1000UL);
    }
    return static_cast<int>(deviceAt(i).mode) * 10000 + state;
}

// ---------------------------------------------------------------------
// ตรวจว่าจุดที่แตะอยู่บนปุ่มไหน (hit test)
// ---------------------------------------------------------------------
Widget hitTest(Page p, int x, int y)
{
    if (p == Page::Dashboard || p == Page::Control || p == Page::Setup || p == Page::Mqtt || p == Page::System) {
        for (int i = 0; i < 5; i++) {
            if (contains(TAB_RECTS[i], x, y)) return static_cast<Widget>(static_cast<int>(Widget::TabDash) + i);
        }
    }
    switch (p) {
        case Page::Dashboard:
            if (contains(CLOCK_CARD, x, y)) return Widget::ClockCard;
            if (contains(TEMP_CARD, x, y)) return Widget::TempCard;
            if (contains(HUM_CARD, x, y)) return Widget::HumCard;
            if (contains(LIGHT_CARD, x, y)) return Widget::LightCard;
            if (contains(SOIL_CARD, x, y)) return Widget::SoilCard;
            if (contains(FAN_TILE, x, y)) return Widget::FanTile;
            if (contains(LIGHT_TILE, x, y)) return Widget::LightTile;
            if (contains(PUMP_TILE, x, y)) return Widget::PumpTile;
            break;
        case Page::Control:
            for (int i = 0; i < 3; i++) {
                if (contains(CTRL_MODE_BTN[i], x, y)) return deviceWidget(i, false);
                if (contains(CTRL_MAN_BTN[i], x, y)) return deviceWidget(i, true);
            }
            break;
        case Page::Setup:
            for (int r = 0; r < 5; r++) {
                if (settingAtRow(setupSub, r) == Setting::None) continue;
                if (contains(SET_MINUS_BTN[r], x, y)) return static_cast<Widget>(static_cast<int>(Widget::SetMinus0) + r);
                if (contains(SET_PLUS_BTN[r], x, y)) return static_cast<Widget>(static_cast<int>(Widget::SetPlus0) + r);
            }
            break;
        case Page::Mqtt:
            if (contains(REMOTE_BTN, x, y)) return Widget::RemoteBtn;
            break;
        case Page::System:
            if (contains(BL_SLIDER_AREA, x, y)) return Widget::BlSlider;
            if (contains(SYNC_BTN, x, y)) return Widget::SyncBtn;
            if (contains(TEST_BTN, x, y)) return Widget::TestBtn;
            if (contains(LDR_DARK_BTN, x, y)) return Widget::LdrDarkBtn;
            if (contains(LDR_BRIGHT_BTN, x, y)) return Widget::LdrBrightBtn;
            if (contains(RECAL_BTN, x, y)) return Widget::RecalBtn;
            break;
        case Page::Test:
            if (contains(BACK_BTN, x, y)) return Widget::TestBack;
            if (contains(CLEAR_BTN, x, y)) return Widget::TestClear;
            if (contains(PAINT_BOX, x, y)) return Widget::TestPaint;
            break;
        case Page::Calibrate:
            break;
    }
    return Widget::None;
}

// ปุ่ม - / + ของหน้า SETUP: คืนแถว (0-4) หรือ -1 ถ้าไม่ใช่ และ direction = -1 (ลด) / +1 (เพิ่ม)
int setupRowOf(Widget w, int &direction)
{
    const int v = static_cast<int>(w);
    if (v >= static_cast<int>(Widget::SetMinus0) && v <= static_cast<int>(Widget::SetMinus4)) {
        direction = -1;
        return v - static_cast<int>(Widget::SetMinus0);
    }
    if (v >= static_cast<int>(Widget::SetPlus0) && v <= static_cast<int>(Widget::SetPlus4)) {
        direction = +1;
        return v - static_cast<int>(Widget::SetPlus0);
    }
    return -1;
}

// ---------------------------------------------------------------------
// วาดปุ่ม/แท็บ/แถบเลื่อน (แต่ละปุ่มดูเปลี่ยนไปเมื่อถูกกด)
// ---------------------------------------------------------------------
void drawCardBorder(const Rect &r, bool pressed)
{
    tft.drawRoundRect(r.x, r.y, r.w, r.h, 6, pressed ? COLOR_TEXT : COLOR_BORDER);
    tft.drawRoundRect(r.x + 1, r.y + 1, r.w - 2, r.h - 2, 5, pressed ? COLOR_TEXT : COLOR_CARD);
}

void drawTab(int index)
{
    const Widget w = static_cast<Widget>(static_cast<int>(Widget::TabDash) + index);
    const Page tabPage = static_cast<Page>(index);
    const bool active = page == tabPage;
    const bool pressed = pressedWidget == w;
    drawButton(TAB_RECTS[index], TAB_NAMES[index], pressed ? COLOR_PRESSED : (active ? COLOR_ACTIVE : COLOR_BUTTON),
               active ? COLOR_TEXT : COLOR_BORDER, COLOR_TEXT);
}

void drawTabBar()
{
    for (int i = 0; i < 5; i++) drawTab(i);
}

// แถบเลื่อน: พื้นที่ area (ล้างก่อน) รางยาว track เติมสีตามเปอร์เซ็นต์ และปุ่มวงกลมบอกตำแหน่ง
void drawSlider(const Rect &area, const Rect &track, int percent, uint16_t color)
{
    tft.fillRect(area.x, area.y, area.w, area.h, COLOR_BG);
    tft.fillRoundRect(track.x, track.y, track.w, track.h, 4, COLOR_BORDER);
    const int16_t filled = static_cast<int16_t>(percent * track.w / 100);
    if (filled > 0) {
        tft.fillRoundRect(track.x, track.y, filled, track.h, 4, color);
    }
    const int16_t thumbX = track.x + filled;
    const int16_t thumbY = track.y + track.h / 2;
    tft.fillCircle(thumbX, thumbY, 11, COLOR_TEXT);
    tft.drawCircle(thumbX, thumbY, 11, color);
}

int sliderPercentFromX(const Rect &track, int x)
{
    return constrain((x - track.x) * 100 / track.w, 0, 100);
}

void drawRecalButton(int progressPercent)
{
    const bool pressed = pressedWidget == Widget::RecalBtn;
    drawButton(RECAL_BTN, "RECALIBRATE  (hold 1 s)", pressed ? COLOR_PRESSED : COLOR_BUTTON, COLOR_BORDER, COLOR_TEXT);
    if (progressPercent > 0) {
        const int16_t w = static_cast<int16_t>((RECAL_BTN.w - 8) * progressPercent / 100);
        tft.fillRect(RECAL_BTN.x + 4, RECAL_BTN.y + RECAL_BTN.h - 9, w, 6, COLOR_GREEN);
    }
}

// ปุ่มโหมด AUTO/MANUAL ของหน้า CTRL
void drawModeButton(int i)
{
    const bool pressed = pressedWidget == deviceWidget(i, false);
    const bool autoMode = deviceAt(i).mode == Mode::Auto;
    drawButton(CTRL_MODE_BTN[i], autoMode ? "AUTO" : "MANUAL", pressed ? COLOR_PRESSED : (autoMode ? COLOR_ACTIVE_GREEN : COLOR_ACTIVE),
               autoMode ? COLOR_GREEN : COLOR_BLUE, COLOR_TEXT);
}

// ปุ่ม ON/OFF ของหน้า CTRL (แสดงสถานะจริง ข้อความหรี่เมื่ออยู่ AUTO เพราะอุปกรณ์ทำตามกติกา แตะเพื่อสลับเป็น MANUAL)
void drawManualButton(int i)
{
    const bool pressed = pressedWidget == deviceWidget(i, true);
    const bool on = deviceIsOn(i);
    const bool autoMode = deviceAt(i).mode == Mode::Auto;
    drawButton(CTRL_MAN_BTN[i], on ? "ON" : "OFF", pressed ? COLOR_PRESSED : (on ? COLOR_ACTIVE_GREEN : COLOR_BUTTON),
               on ? COLOR_GREEN : COLOR_BORDER, autoMode ? COLOR_DIM : COLOR_TEXT);
}

// ปุ่ม REMOTE CMD ในหน้า MQTT: เปิดรับคำสั่งทาง MQTT ชั่วคราว (หมดอายุเอง) สีส้มเตือนเมื่อเปิดอยู่
void drawRemoteButton(uint32_t now)
{
    const bool pressed = pressedWidget == Widget::RemoteBtn;
    if (remoteEnabled) {
        int32_t left = static_cast<int32_t>(remoteUntil - now) / 1000;
        if (left < 0) left = 0;
        char text[40];
        snprintf(text, sizeof(text), "REMOTE ON  %ld:%02ld  (tap = OFF)", static_cast<long>(left / 60), static_cast<long>(left % 60));
        drawButton(REMOTE_BTN, text, pressed ? COLOR_PRESSED : rgb565(110, 55, 20), COLOR_ORANGE, COLOR_TEXT);
    } else {
        drawButton(REMOTE_BTN, "REMOTE OFF  (tap to allow)", pressed ? COLOR_PRESSED : COLOR_BUTTON, COLOR_BORDER, COLOR_TEXT);
    }
}

// วาด widget ที่ต้องเปลี่ยนรูปตามสถานะ "ถูกกด"
void redrawWidget(Widget w)
{
    const bool pressed = pressedWidget == w;
    const int v = static_cast<int>(w);
    int direction = 0;
    const int row = setupRowOf(w, direction);
    if (row >= 0) {
        if (settingAtRow(setupSub, row) != Setting::None) {
            drawButton(direction < 0 ? SET_MINUS_BTN[row] : SET_PLUS_BTN[row], direction < 0 ? "-" : "+",
                       pressed ? COLOR_PRESSED : COLOR_BUTTON, COLOR_BORDER, COLOR_TEXT, 4);
        }
        return;
    }
    if (v >= static_cast<int>(Widget::FanModeBtn) && v <= static_cast<int>(Widget::PumpManBtn)) {
        const int i = (v - static_cast<int>(Widget::FanModeBtn)) / 2;
        if ((v - static_cast<int>(Widget::FanModeBtn)) % 2 == 0) {
            drawModeButton(i);
        } else {
            drawManualButton(i);
        }
        return;
    }
    switch (w) {
        case Widget::TabDash: drawTab(0); break;
        case Widget::TabCtrl: drawTab(1); break;
        case Widget::TabSetup: drawTab(2); break;
        case Widget::TabMqtt: drawTab(3); break;
        case Widget::TabSystem: drawTab(4); break;
        case Widget::ClockCard: drawCardBorder(CLOCK_CARD, pressed); break;
        case Widget::TempCard: drawCardBorder(TEMP_CARD, pressed); break;
        case Widget::HumCard: drawCardBorder(HUM_CARD, pressed); break;
        case Widget::LightCard: drawCardBorder(LIGHT_CARD, pressed); break;
        case Widget::SoilCard: drawCardBorder(SOIL_CARD, pressed); break;
        case Widget::FanTile: drawCardBorder(FAN_TILE, pressed); break;
        case Widget::LightTile: drawCardBorder(LIGHT_TILE, pressed); break;
        case Widget::PumpTile: drawCardBorder(PUMP_TILE, pressed); break;
        case Widget::SyncBtn: drawButton(SYNC_BTN, "SYNC NTP", pressed ? COLOR_PRESSED : COLOR_BUTTON, COLOR_BORDER, COLOR_TEXT); break;
        case Widget::TestBtn: drawButton(TEST_BTN, "TOUCH TEST", pressed ? COLOR_PRESSED : COLOR_BUTTON, COLOR_BORDER, COLOR_TEXT); break;
        case Widget::LdrDarkBtn: drawButton(LDR_DARK_BTN, "LDR SET DARK", pressed ? COLOR_PRESSED : COLOR_BUTTON, COLOR_BORDER, COLOR_TEXT); break;
        case Widget::LdrBrightBtn: drawButton(LDR_BRIGHT_BTN, "LDR SET BRIGHT", pressed ? COLOR_PRESSED : COLOR_BUTTON, COLOR_BORDER, COLOR_TEXT); break;
        case Widget::RecalBtn: drawRecalButton(0); break;
        case Widget::RemoteBtn: drawRemoteButton(millis()); break;
        case Widget::TestBack: drawButton(BACK_BTN, "BACK", pressed ? COLOR_PRESSED : COLOR_BUTTON, COLOR_BORDER, COLOR_TEXT); break;
        case Widget::TestClear: drawButton(CLEAR_BTN, "CLEAR", pressed ? COLOR_PRESSED : COLOR_BUTTON, COLOR_BORDER, COLOR_TEXT); break;
        default:
            break;
    }
}

// ---------------------------------------------------------------------
// หน้าจอ: วาดส่วนที่ไม่เปลี่ยนครั้งเดียวตอนเข้าหน้า แล้วอัปเดตเฉพาะส่วนที่เปลี่ยน
// ---------------------------------------------------------------------
void resetDrawnKeys()
{
    drawnTimeKey = NEVER;
    drawnDateKey = NEVER;
    drawnBadgeKey = NEVER;
    drawnSyncCount = NEVER;
    drawnTempKey = NEVER;
    drawnHumKey = NEVER;
    drawnLightKey = NEVER;
    drawnSoilKey = NEVER;
    drawnTempTag = NEVER;
    drawnHumTag = NEVER;
    drawnLightTag = NEVER;
    drawnSoilTag = NEVER;
    for (int i = 0; i < 3; i++) {
        drawnTile[i] = NEVER;
        drawnCtrl[i] = NEVER;
    }
    for (int r = 0; r < 5; r++) drawnSetup[r] = NEVER;
    drawnBacklight = NEVER;
    drawnHeaderTime = NEVER;
    drawnTestKey = NEVER;
    drawnMqttDot = NEVER;
    drawnMqttState = NEVER;
    drawnMqttCounts = NEVER;
    drawnMqttCmd = NEVER;
    drawnMqttBlock = NEVER;
    drawnRemote = NEVER;
}

void drawCalibrationTarget()
{
    int tx, ty;
    calTarget(calStep, tx, ty);
    tft.drawFastHLine(tx - 12, ty, 25, COLOR_GREEN);
    tft.drawFastVLine(tx, ty - 12, 25, COLOR_GREEN);
    tft.drawCircle(tx, ty, 7, COLOR_GREEN);
}

void eraseCalibrationTarget(int step)
{
    int tx, ty;
    calTarget(step, tx, ty);
    tft.fillRect(tx - 14, ty - 14, 29, 29, COLOR_BG);
}

void drawCalibrationScreen()
{
    tft.fillScreen(COLOR_BG);
    tft.setTextDatum(TC_DATUM);
    tft.setTextColor(COLOR_TEXT, COLOR_BG);
    tft.drawString("Touch calibration", SCREEN_W / 2, 70, 4);
    tft.setTextColor(COLOR_DIM, COLOR_BG);
    tft.drawString("Touch the CENTER of each green cross,", SCREEN_W / 2, 108, 2);
    tft.drawString("hold until the next cross appears.", SCREEN_W / 2, 128, 2);
    tft.drawString("(a fingernail or a stylus is more accurate)", SCREEN_W / 2, 148, 2);
    tft.setTextDatum(TL_DATUM);
    char text[24];
    snprintf(text, sizeof(text), "Point %d / 4", calStep + 1);
    drawText(text, 112, 176, 2, COLOR_YELLOW, COLOR_BG, 96);
    drawCalibrationTarget();
}

void beginCalibration()
{
    page = Page::Calibrate;
    calStep = 0;
    calAccepted = 0;
    calSamples = 0;
    calSumX = calSumY = 0;
    calWaitRelease = false;
    calMessageUntil = 0;
    pressTarget = pressedWidget = Widget::None;
    resetDrawnKeys();
    Serial.println("[CAL] touch the center of each cross (4 points)");
    drawCalibrationScreen();
}

void enterPage(Page next)
{
    if (next == Page::Calibrate) {
        beginCalibration();
        return;
    }
    Serial.printf("[UI] page -> %s%s\n", pageName(next), next == Page::Setup ? (setupSub == 0 ? " 1/2" : " 2/2") : "");
    page = next;
    pressTarget = pressedWidget = Widget::None;
    resetDrawnKeys();
    tft.fillScreen(COLOR_BG);
    drawHeader("Lab 11 - MQTT Farm");
    tft.fillCircle(214, HEADER_H / 2, 4, COLOR_GREEN);  // จุด heartbeat ในแถบหัว

    switch (next) {
        case Page::Dashboard:
            drawCard(CLOCK_CARD, "");
            drawCard(TEMP_CARD, "TEMP");
            drawCard(HUM_CARD, "HUM");
            drawCard(LIGHT_CARD, "LIGHT");
            drawCard(SOIL_CARD, "SOIL");
            drawCard(FAN_TILE, "FAN");
            drawCard(LIGHT_TILE, "LIGHT");
            drawCard(PUMP_TILE, "PUMP");
            break;
        case Page::Control:
            for (int i = 0; i < 3; i++) {
                drawCard(CTRL_ROWS[i], "");
                drawText(deviceLabel(i), 16, CTRL_ROWS[i].y + 4, 4, COLOR_TEXT, COLOR_CARD);
                drawModeButton(i);
                drawManualButton(i);
            }
            break;
        case Page::Setup:
            for (int r = 0; r < 5; r++) {
                const Setting s = settingAtRow(setupSub, r);
                if (s == Setting::None) continue;
                drawText(settingName(s), 10, SETUP_ROW_Y[r] + 6, 2, COLOR_DIM, COLOR_BG);
                redrawWidget(static_cast<Widget>(static_cast<int>(Widget::SetMinus0) + r));
                redrawWidget(static_cast<Widget>(static_cast<int>(Widget::SetPlus0) + r));
            }
            drawText(setupSub == 0 ? "page 1/2  (tap SETUP again for page 2)" : "page 2/2  (tap SETUP again for page 1)", 10, 192, 1,
                     COLOR_DIM, COLOR_BG);
            break;
        case Page::Mqtt:
            drawText("BROKER", 10, 32, 2, COLOR_DIM, COLOR_BG);
            snprintf(mqttLine, sizeof(mqttLine), "%s:%u", app::MQTT_HOST, app::MQTT_PORT);
            drawText(mqttLine, 100, 32, 2, COLOR_TEXT, COLOR_BG);
            drawText("STATE", 10, 52, 2, COLOR_DIM, COLOR_BG);
            drawText("TOPIC", 10, 72, 2, COLOR_DIM, COLOR_BG);
            drawText(mqttRoot, 100, 72, 2, COLOR_TEXT, COLOR_BG);
            drawText("TRAFFIC", 10, 92, 2, COLOR_DIM, COLOR_BG);
            drawText("BLOCKED", 10, 112, 2, COLOR_DIM, COLOR_BG);
            drawText("LAST CMD", 10, 132, 2, COLOR_DIM, COLOR_BG);
            redrawWidget(Widget::RemoteBtn);
            break;
        case Page::System:
            drawText("BACKLIGHT", 16, 36, 2, COLOR_DIM, COLOR_BG);
            redrawWidget(Widget::SyncBtn);
            redrawWidget(Widget::TestBtn);
            redrawWidget(Widget::LdrDarkBtn);
            redrawWidget(Widget::LdrBrightBtn);
            drawRecalButton(0);
            break;
        case Page::Test:
            drawText("TOUCH TEST  (draw in the box)", 16, 34, 2, COLOR_DIM, COLOR_BG);
            tft.drawRect(PAINT_BOX.x, PAINT_BOX.y, PAINT_BOX.w, PAINT_BOX.h, COLOR_BORDER);
            redrawWidget(Widget::TestBack);
            redrawWidget(Widget::TestClear);
            lastPaintX = lastPaintY = -1;
            break;
        case Page::Calibrate:
            break;
    }
    if (next == Page::Dashboard || next == Page::Control || next == Page::Setup || next == Page::Mqtt || next == Page::System) {
        drawTabBar();
    }
}

// ป้ายสถานะนาฬิกา (เหมือน Lab 7)
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
    return {"WIFI...", COLOR_BLUE, 6};
}

// เวลาที่มุมขวาของแถบหัว (ทุกหน้า)
void updateHeaderTime(uint32_t now)
{
    (void)now;
    const bool valid = timeValid();
    const int key = valid ? static_cast<int>(time(nullptr) % 86400) : -1;
    if (key == drawnHeaderTime) return;
    char text[16];
    if (valid) {
        formatLocalTime(text, sizeof(text), "%H:%M:%S");
    } else {
        snprintf(text, sizeof(text), "--:--:--");
    }
    drawText(text, 236, 6, 2, valid ? COLOR_TEXT : COLOR_DIM, COLOR_HEADER, 76);
    drawnHeaderTime = key;
}

// จุดสถานะ MQTT ที่แถบหัว (ทุกหน้า): เขียว = เชื่อมต่อแล้ว, ส้ม = รอลองใหม่, เทา = ไม่มี WiFi
void updateHeaderMqtt(uint32_t now)
{
    (void)now;
    const int key = static_cast<int>(mqttState) * 2 + (remoteEnabled ? 1 : 0);
    if (key == drawnMqttDot) return;
    const uint16_t color = mqttState == MqttState::Connected ? COLOR_GREEN : (mqttState == MqttState::Waiting ? COLOR_ORANGE : COLOR_BORDER);
    tft.fillCircle(196, HEADER_H / 2, 4, color);
    // วงแหวนสีส้มรอบจุด = กำลังรับคำสั่งทาง MQTT (REMOTE CMD เปิดอยู่)
    tft.drawCircle(196, HEADER_H / 2, 7, remoteEnabled ? COLOR_ORANGE : COLOR_HEADER);
    drawnMqttDot = key;
}

// ---- หน้า MQTT ----
void updateMqttPage(uint32_t now)
{
    char text[64];

    int retrySeconds = 0;
    if (mqttState == MqttState::Waiting) {
        const int32_t d = static_cast<int32_t>(mqttNextAttemptAt - now);
        if (d > 0) retrySeconds = d / 1000 + 1;
    }
    const int stateKey = static_cast<int>(mqttState) * 100000 + retrySeconds * 100 + (mqttLastRc + 50);
    if (stateKey != drawnMqttState) {
        uint16_t color = COLOR_DIM;
        if (mqttState == MqttState::Connected) {
            snprintf(text, sizeof(text), "CONNECTED");
            color = COLOR_GREEN;
        } else if (mqttState == MqttState::Waiting && retrySeconds > 0) {
            snprintf(text, sizeof(text), "RETRY %ds  (rc=%d)", retrySeconds, mqttLastRc);
            color = COLOR_ORANGE;
        } else if (mqttState == MqttState::Waiting) {
            snprintf(text, sizeof(text), "CONNECTING...");
            color = COLOR_BLUE;
        } else {
            snprintf(text, sizeof(text), "NO WIFI");
            color = COLOR_RED;
        }
        drawText(text, 100, 52, 2, color, COLOR_BG, 210);
        drawnMqttState = stateKey;
    }

    const int countsKey = static_cast<int>(mqttTx * 3 + mqttRx * 1001 + mqttDropped * 17);
    if (countsKey != drawnMqttCounts) {
        snprintf(text, sizeof(text), "TX %lu  RX %lu  drop %lu", static_cast<unsigned long>(mqttTx), static_cast<unsigned long>(mqttRx),
                 static_cast<unsigned long>(mqttDropped));
        drawText(text, 100, 92, 2, COLOR_TEXT, COLOR_BG, 210);
        drawnMqttCounts = countsKey;
    }

    const int blockKey = static_cast<int>(mqttLastBlockMs * 7 + mqttMaxBlockMs);
    if (blockKey != drawnMqttBlock) {
        snprintf(text, sizeof(text), "last %lu ms, max %lu ms", static_cast<unsigned long>(mqttLastBlockMs), static_cast<unsigned long>(mqttMaxBlockMs));
        drawText(text, 100, 112, 2, COLOR_TEXT, COLOR_BG, 210);
        drawnMqttBlock = blockKey;
    }

    if (lastCmdSeq != drawnMqttCmd) {
        snprintf(text, sizeof(text), "%.36s", lastCmdText[0] != '\0' ? lastCmdText : "(none yet)");
        drawText(text, 100, 137, 1, COLOR_YELLOW, COLOR_BG, 216);
        drawnMqttCmd = lastCmdSeq;
    }

    int32_t remoteLeft = remoteEnabled ? static_cast<int32_t>(remoteUntil - now) / 1000 : -1;
    if (remoteLeft < -1) remoteLeft = 0;
    if (remoteLeft != drawnRemote) {
        drawRemoteButton(now);
        drawnRemote = remoteLeft;
    }
}

// ---- หน้า DASH ----
// การ์ดเซนเซอร์: ค่า (ตัวใหญ่) และป้ายหน่วย+แหล่งที่มา (ตัวเล็ก)
void drawSensorValue(const Rect &card, const char *text, uint16_t color)
{
    drawText(text, card.x + 6, card.y + 19, 4, color, COLOR_CARD, 62);
}

void drawSensorTag(const Rect &card, const char *text, uint16_t color)
{
    drawText(text, card.x + 6, card.y + 46, 1, color, COLOR_CARD, 62);
}

// อุปกรณ์ 3 ตัวบน DASH
void drawTile(int i, uint32_t now)
{
    static const Rect *tiles[3] = {&FAN_TILE, &LIGHT_TILE, &PUMP_TILE};
    const Rect &r = *tiles[i];
    const Device &d = deviceAt(i);
    char value[16];
    char modeText[20];
    uint16_t valueColor = COLOR_DIM;

    if (i == 0) {
        if (fanDuty > 0) {
            snprintf(value, sizeof(value), "%d%%", fanDuty);
            valueColor = fanFailsafe ? COLOR_RED : COLOR_BLUE;
        } else {
            snprintf(value, sizeof(value), "OFF");
        }
    } else if (i == 1) {
        snprintf(value, sizeof(value), "%s", lightOn ? "ON" : "OFF");
        valueColor = lightOn ? COLOR_YELLOW : COLOR_DIM;
    } else if (pumpOn) {
        snprintf(value, sizeof(value), "RUN");
        valueColor = COLOR_BLUE;
    } else if (pumpResting(now)) {
        snprintf(value, sizeof(value), "REST");
        valueColor = COLOR_ORANGE;
    } else {
        snprintf(value, sizeof(value), "OFF");
    }

    const char *modeName = d.mode == Mode::Auto ? "AUTO" : "MAN";
    uint16_t modeColor = d.mode == Mode::Auto ? COLOR_GREEN : COLOR_BLUE;
    if (i == 0 && fanFailsafe) {
        snprintf(modeText, sizeof(modeText), "FAILSAFE");
        modeColor = COLOR_RED;
    } else if (i == 2 && pumpOn) {
        snprintf(modeText, sizeof(modeText), "%s %lus", modeName, static_cast<unsigned long>((now - pumpStartedAt) / 1000UL));
    } else if (i == 2 && pumpResting(now)) {
        snprintf(modeText, sizeof(modeText), "%s %lus", modeName, static_cast<unsigned long>((pumpLockUntil - now) / 1000UL + 1));
    } else {
        snprintf(modeText, sizeof(modeText), "%s", modeName);
    }
    drawText(value, r.x + 8, r.y + 22, 4, valueColor, COLOR_CARD, 84);
    drawText(modeText, r.x + 8, r.y + 49, 2, modeColor, COLOR_CARD, 84);
}

void updateDashboard(uint32_t now)
{
    char text[40];
    const bool valid = timeValid();
    const time_t t = time(nullptr);

    const int timeKey = valid ? static_cast<int>(t % 86400) : -1;
    if (timeKey != drawnTimeKey) {
        if (valid) {
            formatLocalTime(text, sizeof(text), "%H:%M:%S");
        } else {
            snprintf(text, sizeof(text), "--:--:--");
        }
        drawText(text, CLOCK_CARD.x + 8, CLOCK_CARD.y + 4, 4, valid ? (timeStale(now) ? COLOR_YELLOW : COLOR_TEXT) : COLOR_DIM, COLOR_CARD, 118);
        drawnTimeKey = timeKey;
    }

    const Badge badge = clockBadge(now);
    if (badge.key != drawnBadgeKey) {
        drawText(badge.text, CLOCK_CARD.x + 130, CLOCK_CARD.y + 9, 2, badge.color, COLOR_CARD, 84);
        drawnBadgeKey = badge.key;
    }

    // คีย์ของวันที่คำนวณจากเวลาท้องถิ่น (UTC+7) ไม่ใช่ UTC (เรียนรู้จาก Lab 7)
    const int dateKey = valid ? static_cast<int>((static_cast<long>(t) + app::GMT_OFFSET_SEC + app::DAYLIGHT_OFFSET_SEC) / 86400) : -1;
    if (dateKey != drawnDateKey) {
        if (valid) {
            formatLocalTime(text, sizeof(text), "%d/%m/%y");
        } else {
            text[0] = '\0';
        }
        drawText(text, CLOCK_CARD.x + 216, CLOCK_CARD.y + 9, 2, COLOR_DIM, COLOR_CARD, 84);
        drawnDateKey = dateKey;
    }

    // อุณหภูมิ / ความชื้น (ป้ายแหล่งที่มา REAL / SIM / ERR / N/A)
    const DataSource source = currentSource();
    const int tagKey = static_cast<int>(source) * 2 + (sensorError ? 1 : 0);
    const char *tagSource = (source == DataSource::Real) ? "REAL" : (isSimulated(source) ? "SIM" : (sensorError ? "ERR" : "N/A"));
    const uint16_t tagColor = (source == DataSource::Real) ? COLOR_GREEN : (isSimulated(source) ? COLOR_YELLOW : (sensorError ? COLOR_RED : COLOR_DIM));

    const float tempC = effectiveTemp(now);
    const int tempKey = isnan(tempC) ? -1 : static_cast<int>(lroundf(tempC * 10.0f));
    if (tempKey != drawnTempKey) {
        if (isnan(tempC)) {
            snprintf(text, sizeof(text), "--.-");
            drawSensorValue(TEMP_CARD, text, sensorError ? COLOR_RED : COLOR_DIM);
        } else {
            snprintf(text, sizeof(text), "%.1f", tempC);
            drawSensorValue(TEMP_CARD, text, tempC >= static_cast<float>(cfg.fanOn) ? COLOR_ORANGE : COLOR_TEXT);
        }
        drawnTempKey = tempKey;
    }
    if (tagKey != drawnTempTag) {
        snprintf(text, sizeof(text), "C  %s", tagSource);
        drawSensorTag(TEMP_CARD, text, tagColor);
        drawnTempTag = tagKey;
    }

    const float humidity = effectiveHumidity(now);
    const int humKey = isnan(humidity) ? -1 : static_cast<int>(lroundf(humidity));
    if (humKey != drawnHumKey) {
        if (isnan(humidity)) {
            snprintf(text, sizeof(text), "--");
            drawSensorValue(HUM_CARD, text, sensorError ? COLOR_RED : COLOR_DIM);
        } else {
            snprintf(text, sizeof(text), "%d", humKey);
            drawSensorValue(HUM_CARD, text, COLOR_TEXT);
        }
        drawnHumKey = humKey;
    }
    if (tagKey != drawnHumTag) {
        snprintf(text, sizeof(text), "%%  %s", tagSource);
        drawSensorTag(HUM_CARD, text, tagColor);
        drawnHumTag = tagKey;
    }

    // แสง (LDR หรือจำลอง): ตัวเลขเป็นสีเหลืองเมื่อต่ำกว่าเกณฑ์เปิดไฟ
    const float ambient = ambientPercent(now);
    const int lightKey = isnan(ambient) ? -1 : static_cast<int>(lroundf(ambient));
    if (lightKey != drawnLightKey) {
        if (isnan(ambient)) {
            snprintf(text, sizeof(text), "--");
            drawSensorValue(LIGHT_CARD, text, COLOR_DIM);
        } else {
            snprintf(text, sizeof(text), "%d", lightKey);
            drawSensorValue(LIGHT_CARD, text, ambient < static_cast<float>(cfg.lightThr) ? COLOR_YELLOW : COLOR_TEXT);
        }
        drawnLightKey = lightKey;
    }
    const int lightTagKey = lightSim ? 1 : 0;
    if (lightTagKey != drawnLightTag) {
        drawSensorTag(LIGHT_CARD, lightSim ? "%  SIM" : "%  LDR", lightSim ? COLOR_YELLOW : COLOR_GREEN);
        drawnLightTag = lightTagKey;
    }

    // ความชื้นดิน (จำลองเสมอ): ตัวเลขเป็นสีแดงเมื่อแห้งกว่าเกณฑ์
    const int soilKey = static_cast<int>(lroundf(soilPercent));
    if (soilKey != drawnSoilKey) {
        snprintf(text, sizeof(text), "%d", soilKey);
        drawSensorValue(SOIL_CARD, text, soilPercent < static_cast<float>(cfg.soilDry) ? COLOR_RED : COLOR_TEXT);
        drawnSoilKey = soilKey;
    }
    if (drawnSoilTag == NEVER) {
        drawSensorTag(SOIL_CARD, "%  SIM", COLOR_YELLOW);
        drawnSoilTag = 1;
    }

    for (int i = 0; i < 3; i++) {
        const int key = deviceKey(i, now);
        if (key != drawnTile[i]) {
            drawTile(i, now);
            drawnTile[i] = key;
        }
    }
}

// ---- หน้า CTRL ----
void updateControlPage(uint32_t now)
{
    char text[24];
    for (int i = 0; i < 3; i++) {
        const int key = deviceKey(i, now);
        if (key == drawnCtrl[i]) continue;
        uint16_t color = COLOR_TEXT;
        if (i == 0) {
            if (fanFailsafe) {
                snprintf(text, sizeof(text), "FAILSAFE");
                color = COLOR_RED;
            } else if (fanDuty > 0) {
                snprintf(text, sizeof(text), "ON  %d%%", fanDuty);
                color = COLOR_BLUE;
            } else {
                snprintf(text, sizeof(text), "OFF");
                color = COLOR_DIM;
            }
        } else if (i == 1) {
            snprintf(text, sizeof(text), "%s", lightOn ? "ON" : "OFF");
            color = lightOn ? COLOR_YELLOW : COLOR_DIM;
        } else if (pumpOn) {
            snprintf(text, sizeof(text), "RUN %lus", static_cast<unsigned long>((now - pumpStartedAt) / 1000UL));
            color = COLOR_BLUE;
        } else if (pumpResting(now)) {
            snprintf(text, sizeof(text), "REST %lus", static_cast<unsigned long>((pumpLockUntil - now) / 1000UL + 1));
            color = COLOR_ORANGE;
        } else {
            snprintf(text, sizeof(text), "OFF");
            color = COLOR_DIM;
        }
        drawText(text, 16, CTRL_ROWS[i].y + 30, 2, color, COLOR_CARD, 108);
        redrawWidget(deviceWidget(i, false));
        redrawWidget(deviceWidget(i, true));
        drawnCtrl[i] = key;
    }
}

// ---- หน้า SETUP ----
void updateSetupPage()
{
    char text[40];
    for (int r = 0; r < 5; r++) {
        const Setting s = settingAtRow(setupSub, r);
        if (s == Setting::None) {
            if (setupSub == 1 && r == 4) {
                const int key = soilWetTarget() * 100 + app::PUMP_MAX_RUN_SEC;
                if (key != drawnSetup[r]) {
                    snprintf(text, sizeof(text), "stop at %d%%, max %d s", soilWetTarget(), app::PUMP_MAX_RUN_SEC);
                    drawText(text, 10, SETUP_ROW_Y[r] + 6, 2, COLOR_DIM, COLOR_BG, 300);
                    drawnSetup[r] = key;
                }
            }
            continue;
        }
        const int key = settingValue(s);
        if (key != drawnSetup[r]) {
            settingText(s, text, sizeof(text));
            drawText(text, 124, SETUP_ROW_Y[r] + 6, 2, COLOR_TEXT, COLOR_BG, 70);
            drawnSetup[r] = key;
        }
    }
}

// ---- หน้า SYSTEM ----
void updateSystemPage(uint32_t now)
{
    (void)now;
    char text[48];
    if (backlightPercent != drawnBacklight) {
        drawSlider(BL_SLIDER_AREA, BL_SLIDER_TRACK, (backlightPercent - app::BACKLIGHT_MIN_PERCENT) * 100 / (100 - app::BACKLIGHT_MIN_PERCENT),
                   COLOR_YELLOW);
        snprintf(text, sizeof(text), "%d%%", backlightPercent);
        drawText(text, 240, 36, 2, COLOR_TEXT, COLOR_BG, 64);
        drawnBacklight = backlightPercent;
    }
}

void updateSystemInfo(uint32_t now)
{
    char text[64];
    char wifi[32];
    char ago[16];
    if (wifiState == WifiState::Connected) {
        if (syncCount > 0) {
            formatAgo(ago, sizeof(ago), now - lastSyncAtMs);
            snprintf(wifi, sizeof(wifi), "WiFi %ddBm #%lu %s", WiFi.RSSI(), static_cast<unsigned long>(syncCount), ago);
        } else {
            snprintf(wifi, sizeof(wifi), "WiFi %ddBm no NTP", WiFi.RSSI());
        }
    } else {
        snprintf(wifi, sizeof(wifi), "WiFi: %s", wifiStateName(wifiState));
    }
    snprintf(text, sizeof(text), "LDR %d [%d..%d]  %s", ldrRaw, ldrRawDark, ldrRawBright, wifi);
    drawText(text, 8, 193, 1, COLOR_DIM, COLOR_BG, 304);
}

// ---- หน้า TOUCH TEST ----
void updateTestPage()
{
    char text[56];
    // ข้อความอัปเดตเมื่อค่าเปลี่ยน (ใช้ค่าดิบ+สถานะเป็นคีย์)
    const int key = (touch.touchingNow ? 1 : 0) + touch.rawX * 2 + touch.rawY * 8191 + touch.z * 3;
    if (key != drawnTestKey) {
        if (touch.touchingNow) {
            snprintf(text, sizeof(text), "RAW x=%4d y=%4d  z=%4d", touch.rawX, touch.rawY, touch.z);
        } else {
            snprintf(text, sizeof(text), "RAW  -    -       z=%4d", touch.z);
        }
        drawText(text, 16, 52, 2, COLOR_TEXT, COLOR_BG, 288);
        if (touch.down) {
            snprintf(text, sizeof(text), "PIXEL x=%3d y=%3d   DOWN", touch.x, touch.y);
        } else {
            snprintf(text, sizeof(text), "PIXEL last x=%3d y=%3d  UP", touch.lastX, touch.lastY);
        }
        drawText(text, 16, 70, 2, touch.down ? COLOR_GREEN : COLOR_DIM, COLOR_BG, 288);
        drawnTestKey = key;
    }
}

// ---------------------------------------------------------------------
// การทำงานของปุ่ม
// ---------------------------------------------------------------------
void cycleSimChoice()
{
    simChoice = static_cast<SimChoice>((static_cast<uint8_t>(simChoice) + 1) % 3);
    if (simChoice == SimChoice::Off) {
        Serial.println("[SIM] off: using the real DHT11 sensor");
    } else if (simChoice == SimChoice::Random) {
        Serial.println("[SIM] random: simulated temperature/humidity (random walk)");
    } else {
        Serial.println("[SIM] sweep: simulated temperature/humidity (smooth up-down cycle)");
    }
}

void toggleLightSim()
{
    lightSim = !lightSim;
    Serial.printf("[SIM] light: %s\n", lightSim ? "simulated day/night cycle (60 s)" : "real LDR sensor");
}

void makeSoilDry()
{
    soilPercent = app::SOIL_TEST_DRY_PERCENT;
    Serial.printf("[SIM] soil set to %.0f%% (test: AUTO pump should start)\n", soilPercent);
}

// เริ่มกด (กดลง) ที่ widget
void onPress(Widget w, uint32_t now)
{
    repeatAt = now + app::REPEAT_DELAY_MS;
    longPressDone = false;
    drawnLongPress = -1;
    int direction = 0;
    const int row = setupRowOf(w, direction);
    if (row >= 0) {
        adjustSetting(settingAtRow(setupSub, row), direction);
    }
}

// กำลังกดค้าง / ลาก
void onHold(Widget w, uint32_t now)
{
    int direction = 0;
    const int row = setupRowOf(w, direction);
    if (row >= 0) {
        if (pressedWidget == w && now >= repeatAt) {  // กดค้างแล้วเพิ่ม/ลดซ้ำ (auto-repeat)
            repeatAt = now + app::REPEAT_INTERVAL_MS;
            adjustSetting(settingAtRow(setupSub, row), direction);
        }
        return;
    }
    switch (w) {
        case Widget::BlSlider: {
            const int fraction = sliderPercentFromX(BL_SLIDER_TRACK, touch.x);
            const int percent = app::BACKLIGHT_MIN_PERCENT + fraction * (100 - app::BACKLIGHT_MIN_PERCENT) / 100;
            if (percent != backlightPercent) applyBacklight(percent);
            break;
        }
        case Widget::RecalBtn:
            if (pressedWidget == w && !longPressDone) {
                const uint32_t held = now - touch.downSince;
                const int progress = static_cast<int>(min<uint32_t>(100, held * 100 / app::LONG_PRESS_MS));
                if (progress != drawnLongPress) {
                    drawRecalButton(progress);
                    drawnLongPress = progress;
                }
                if (held >= app::LONG_PRESS_MS) {
                    longPressDone = true;
                    Serial.println("[UI] long press: starting touch calibration");
                    beginCalibration();
                }
            }
            break;
        case Widget::TestPaint:
            if (touch.x != lastPaintX || touch.y != lastPaintY) {
                tft.fillCircle(touch.x, touch.y, 2, COLOR_GREEN);
                lastPaintX = touch.x;
                lastPaintY = touch.y;
            }
            break;
        default:
            break;
    }
}

// ปล่อยนิ้วบน widget เดิมที่กดลง = "คลิก" ปุ่มนั้น
void onActivate(Widget w, uint32_t now)
{
    const int v = static_cast<int>(w);
    if (v >= static_cast<int>(Widget::FanModeBtn) && v <= static_cast<int>(Widget::PumpManBtn)) {
        const int i = (v - static_cast<int>(Widget::FanModeBtn)) / 2;
        if ((v - static_cast<int>(Widget::FanModeBtn)) % 2 == 0) {
            toggleDeviceMode(deviceAt(i), deviceIsOn(i), deviceLabel(i));
        } else {
            toggleDeviceManual(deviceAt(i), deviceIsOn(i), deviceLabel(i));
        }
        updateControl(now);  // ให้อุปกรณ์ตอบสนองทันที ไม่ต้องรอรอบถัดไป
        return;
    }
    switch (w) {
        case Widget::TabDash: if (page != Page::Dashboard) enterPage(Page::Dashboard); break;
        case Widget::TabCtrl: if (page != Page::Control) enterPage(Page::Control); break;
        case Widget::TabSetup:
            if (page == Page::Setup) setupSub ^= 1;  // แตะแท็บเดิมซ้ำ = สลับหน้าย่อย
            enterPage(Page::Setup);
            break;
        case Widget::TabMqtt: if (page != Page::Mqtt) enterPage(Page::Mqtt); break;
        case Widget::TabSystem: if (page != Page::System) enterPage(Page::System); break;
        case Widget::RemoteBtn: setRemote(!remoteEnabled, now); break;
        case Widget::ClockCard: requestNtpNow(now); break;
        case Widget::TempCard:
        case Widget::HumCard: cycleSimChoice(); break;
        case Widget::LightCard: toggleLightSim(); break;
        case Widget::SoilCard: makeSoilDry(); break;
        case Widget::FanTile:
        case Widget::LightTile:
        case Widget::PumpTile: enterPage(Page::Control); break;
        case Widget::SyncBtn: requestNtpNow(now); break;
        case Widget::LdrDarkBtn: captureLdrEnd(true); break;
        case Widget::LdrBrightBtn: captureLdrEnd(false); break;
        case Widget::TestBtn: enterPage(Page::Test); break;
        case Widget::TestBack: enterPage(Page::System); break;
        case Widget::TestClear:
            tft.fillRect(PAINT_BOX.x + 1, PAINT_BOX.y + 1, PAINT_BOX.w - 2, PAINT_BOX.h - 2, COLOR_BG);
            lastPaintX = lastPaintY = -1;
            break;
        default:
            break;
    }
}

// ปล่อยนิ้ว (ไม่ว่าจะคลิกหรือไม่)
void onRelease(Widget w)
{
    int direction = 0;
    if (setupRowOf(w, direction) >= 0 && settingsDirty) {
        saveSettings();  // บันทึกตอนปล่อยนิ้ว ไม่ใช่ทุกครั้งที่ค่าเปลี่ยนขณะกดค้าง (ถนอม flash)
    }
    if (w == Widget::BlSlider) {
        prefs.putUChar("bl", static_cast<uint8_t>(backlightPercent));  // จำความสว่างไว้ตลอดไป
        Serial.printf("[UI] backlight %d%% saved\n", backlightPercent);
    }
    if (w == Widget::RecalBtn && page == Page::System) {
        drawRecalButton(0);
    }
}

// ---------------------------------------------------------------------
// ประมวลผลเหตุการณ์สัมผัสของหน้าปกติ
// ---------------------------------------------------------------------
void handleTouchEvents(uint32_t now)
{
    if (touch.pressEdge) {
        pressTarget = hitTest(page, touch.x, touch.y);
        pressedWidget = pressTarget;
        Serial.printf("[TOUCH] down  x=%3d y=%3d  raw=(%4d,%4d) z=%4d  page=%s target=%s\n", touch.x, touch.y, touch.rawX, touch.rawY,
                      touch.z, pageName(page), widgetName(pressTarget));
        onPress(pressTarget, now);
        redrawWidget(pressTarget);
    }

    if (touch.down && pressTarget != Widget::None) {
        // นิ้วเลื่อนออกนอกปุ่ม = ปุ่มดูเหมือนไม่ถูกกด (ปล่อยตรงนั้นจะไม่ทำงาน) เหมือนปุ่มบนมือถือ
        const bool stillInside = hitTest(page, touch.x, touch.y) == pressTarget;
        const Widget wanted = stillInside ? pressTarget : Widget::None;
        if (wanted != pressedWidget) {
            pressedWidget = wanted;
            redrawWidget(pressTarget);
        }
        onHold(pressTarget, now);
    }

    if (touch.releaseEdge) {
        const Widget releasedOn = hitTest(page, touch.lastX, touch.lastY);
        const bool clicked = pressTarget != Widget::None && releasedOn == pressTarget;
        const uint32_t held = now - touch.downSince;
        Serial.printf("[TOUCH] up    x=%3d y=%3d  held=%lu ms  target=%s  %s\n", touch.lastX, touch.lastY, static_cast<unsigned long>(held),
                      widgetName(pressTarget), clicked ? "click" : "(not a click)");
        const Widget target = pressTarget;
        pressedWidget = Widget::None;
        pressTarget = Widget::None;
        redrawWidget(target);
        const Page before = page;
        const int beforeSub = setupSub;
        if (clicked) onActivate(target, now);
        if (page == before && setupSub == beforeSub) onRelease(target);
    }
}

void handleCalibration(uint32_t now)
{
    if (calMessageUntil != 0) {  // กำลังแสดงข้อความผลลัพธ์
        if (now >= calMessageUntil) {
            calMessageUntil = 0;
            if (calResultOk) {
                enterPage(Page::Dashboard);
            } else {
                beginCalibration();  // ล้มเหลว: เริ่มใหม่ (ไม่ใช้ค่าที่ผิด)
            }
        }
        return;
    }

    if (calWaitRelease) {  // รอให้ปล่อยนิ้วก่อนไปเป้าถัดไป
        if (!touch.down && !touch.touchingNow) {
            calWaitRelease = false;
            eraseCalibrationTarget(calStep);
            calStep++;
            if (calStep >= 4) {
                const bool ok = computeCalibration();
                calResultOk = ok;
                tft.fillRect(0, 60, SCREEN_W, 130, COLOR_BG);
                tft.setTextDatum(TC_DATUM);
                tft.setTextColor(ok ? COLOR_GREEN : COLOR_RED, COLOR_BG);
                tft.drawString(ok ? "Calibration saved" : "Calibration FAILED", SCREEN_W / 2, 100, 4);
                tft.setTextColor(COLOR_DIM, COLOR_BG);
                tft.drawString(ok ? "Starting the panel..." : "Please try again", SCREEN_W / 2, 140, 2);
                tft.setTextDatum(TL_DATUM);
                if (ok) saveCalibration();
                calMessageUntil = now + 1500;
            } else {
                char text[24];
                snprintf(text, sizeof(text), "Point %d / 4", calStep + 1);
                drawText(text, 112, 176, 2, COLOR_YELLOW, COLOR_BG, 96);
                drawCalibrationTarget();
                calSamples = 0;
                calAccepted = 0;
                calSumX = calSumY = 0;
            }
        }
        return;
    }

    if (touch.touchingNow) {
        calSamples++;
        if (calSamples > 3) {  // ข้ามตัวอย่างแรกๆ (ยังไม่นิ่ง)
            calSumX += touch.rawX;
            calSumY += touch.rawY;
            calAccepted++;
        }
        if (calAccepted >= app::CAL_SAMPLES) {
            calRawX[calStep] = static_cast<int>(calSumX / calAccepted);
            calRawY[calStep] = static_cast<int>(calSumY / calAccepted);
            Serial.printf("[CAL] point %d raw = (%d, %d)\n", calStep + 1, calRawX[calStep], calRawY[calStep]);
            int tx, ty;
            calTarget(calStep, tx, ty);
            tft.fillCircle(tx, ty, 5, COLOR_GREEN);  // จุดทึบ = รับค่าแล้ว (ให้ปล่อยนิ้ว)
            calWaitRelease = true;
        }
    } else {  // ยกนิ้วก่อนเก็บครบ = เริ่มเก็บใหม่
        calSamples = 0;
        calAccepted = 0;
        calSumX = calSumY = 0;
    }
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

// ---------------------------------------------------------------------
// Serial และปุ่ม BOOT
// ---------------------------------------------------------------------
void printHelp()
{
    Serial.println("[HELP] devices: f = FAN on/off (MANUAL),  l = LIGHT on/off (MANUAL),  p = PUMP on/off (MANUAL),  a = all AUTO");
    Serial.println("[HELP] sim:     s = temp/humidity REAL/SIM,  e = light REAL/SIM,  d = make the soil dry (test the pump)");
    Serial.println("[HELP] mqtt:    r = allow/deny commands from MQTT (REMOTE CMD, switches off by itself),  m = print the MQTT status");
    Serial.println("[HELP] time:    n = ask NTP again,  t = print the time now");
    Serial.println("[HELP] touch:   c = calibrate the touch screen again,  k = print the calibration, touch and LDR values");
    Serial.println("[HELP] h = help.   BOOT button on the board = EMERGENCY STOP (all devices MANUAL + OFF)");
}

void printTouchInfo()
{
    Serial.printf("[TOUCH] calibration: %s", cal.valid ? "valid" : "NOT calibrated");
    if (cal.valid) {
        Serial.printf("  swapXY=%s x: %.4f*u%+.1f  y: %.4f*v%+.1f", cal.swapXY ? "yes" : "no", cal.xb, cal.xa, cal.yb, cal.ya);
    }
    Serial.printf("\n[TOUCH] last raw=(%d,%d) z=%d  touching=%s  down=%s\n", touch.rawX, touch.rawY, touch.z, touch.touchingNow ? "yes" : "no",
                  touch.down ? "yes" : "no");
    Serial.printf("[LDR] raw=%d smooth=%.1f  bright=%d dark=%d  -> light=%.0f%% (%s)\n", ldrRaw, isnan(ldrSmooth) ? -1.0f : ldrSmooth,
                  ldrRawBright, ldrRawDark, ambientPercent(millis()), lightSim ? "SIM" : "REAL");
}

void printMqttInfo()
{
    const uint32_t now = millis();
    Serial.printf("[MQTT] state=%s rc=%d  root=%s  TX=%lu RX=%lu dropped=%lu  connect fails=%lu  blocked: last %lu ms, max %lu ms  remote=%s\n",
                  mqttStateName(now), mqttLastRc, mqttRoot, static_cast<unsigned long>(mqttTx), static_cast<unsigned long>(mqttRx),
                  static_cast<unsigned long>(mqttDropped), static_cast<unsigned long>(mqttConnectFails), static_cast<unsigned long>(mqttLastBlockMs),
                  static_cast<unsigned long>(mqttMaxBlockMs), remoteEnabled ? "ON" : "off");
}

void printStatus()
{
    const uint32_t now = millis();
    const DataSource source = currentSource();
    const float tempC = effectiveTemp(now);
    const float humidity = effectiveHumidity(now);
    const float ambient = ambientPercent(now);
    char tempText[16], humText[16], lightText[16];
    if (isnan(tempC)) snprintf(tempText, sizeof(tempText), "--"); else snprintf(tempText, sizeof(tempText), "%.1f", tempC);
    if (isnan(humidity)) snprintf(humText, sizeof(humText), "--"); else snprintf(humText, sizeof(humText), "%.0f", humidity);
    if (isnan(ambient)) snprintf(lightText, sizeof(lightText), "--"); else snprintf(lightText, sizeof(lightText), "%.0f", ambient);

    Serial.printf("[STATUS] page=%s src=%-10s temp=%s C hum=%s %% light=%s %% (raw %d) soil=%.0f %% | fan=%3d%% %s light=%s %s pump=%s %s%s | bl=%d%% touch z=%d%s\n",
                  pageName(page), sourceName(source), tempText, humText, lightText, ldrRaw, soilPercent, fanDuty, deviceModeName(fanDev),
                  lightOn ? "ON " : "OFF", deviceModeName(lightDev), pumpOn ? "ON " : (pumpResting(now) ? "REST" : "OFF"),
                  deviceModeName(pumpDev), fanFailsafe ? " [FAN FAIL-SAFE]" : "", backlightPercent, touch.z, sensorError ? "  SENSOR ERROR" : "");
}

void handleSerialInput(uint32_t now)
{
    while (Serial.available() > 0) {
        const int c = Serial.read();
        if (c == 'f' || c == 'F') {
            toggleDeviceManual(fanDev, fanDuty > 0, "FAN");
            updateControl(now);
        } else if (c == 'l' || c == 'L') {
            toggleDeviceManual(lightDev, lightOn, "LIGHT");
            updateControl(now);
        } else if (c == 'p' || c == 'P') {
            toggleDeviceManual(pumpDev, pumpOn, "PUMP");
            updateControl(now);
        } else if (c == 'a' || c == 'A') {
            setAllAuto();
        } else if (c == 's' || c == 'S') {
            cycleSimChoice();
        } else if (c == 'e' || c == 'E') {
            toggleLightSim();
        } else if (c == 'd' || c == 'D') {
            makeSoilDry();
        } else if (c == 'n' || c == 'N') {
            requestNtpNow(now);
        } else if (c == 't' || c == 'T') {
            printTime(now);
        } else if (c == 'c' || c == 'C') {
            beginCalibration();
        } else if (c == 'k' || c == 'K') {
            printTouchInfo();
        } else if (c == 'r' || c == 'R') {
            setRemote(!remoteEnabled, now);
        } else if (c == 'm' || c == 'M') {
            printMqttInfo();
        } else if (c == 'h' || c == 'H' || c == '?') {
            printHelp();
        }
    }
}

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

void updateHeartbeat()
{
    heartbeatOn = !heartbeatOn;
    if (page != Page::Calibrate) {
        tft.fillCircle(214, HEADER_H / 2, 4, heartbeatOn ? COLOR_GREEN : COLOR_HEADER);
    }
}

}  // namespace

void setup()
{
    Serial.begin(app::SERIAL_BAUD);
    delay(200);

    Serial.println();
    Serial.println("==========================================");
    Serial.println("Hello World from ESP32-2432S028 (CYD)!");
    Serial.println("Level 1 / Lab 11 : MQTT Farm");
    Serial.println("==========================================");
    Serial.printf("[PINS] fan=GPIO%u (blue LED, PWM)  light=GPIO%u (green LED)  pump=GPIO%u (red LED)  LDR=GPIO%u  DHT11=GPIO%u\n",
                  cyd::LED_PIN_B, cyd::LED_PIN_G, cyd::LED_PIN_R, cyd::LDR_PIN, cyd::DHT11_PIN);

    cyd::ledInit();  // ทุกอุปกรณ์เริ่มที่ OFF
    pinMode(cyd::BOOT_BUTTON_PIN, INPUT_PULLUP);
    analogSetPinAttenuation(cyd::LDR_PIN, ADC_0db);  // เหมือน Lab 2: ขยายช่วงค่า raw ของ LDR

    loadSettings();  // ค่า calibrate ความสว่างจอ และเกณฑ์ที่เคยตั้งไว้

    tft.init();  // ไลบรารีจอเปิดไฟหลังจอ (digitalWrite) ให้ก่อน
    tft.setRotation(cyd::DISPLAY_ROTATION);
    tft.setTextDatum(TL_DATUM);
    tft.fillScreen(COLOR_BG);
    backlightBegin();  // จากนี้ GPIO21 ถูกคุมด้วย PWM (ปรับความสว่างได้)

    pwmBegin();
    dht.begin();
    lastReadAt = millis();
    readLdr();

    // จอสัมผัส: เริ่ม SPI ชุด VSPI ด้วยขาของ XPT2046
    pinMode(cyd::TOUCH_PIN_CS, OUTPUT);
    digitalWrite(cyd::TOUCH_PIN_CS, HIGH);
    touchSpi.begin(cyd::TOUCH_PIN_CLK, cyd::TOUCH_PIN_MISO, cyd::TOUCH_PIN_MOSI, cyd::TOUCH_PIN_CS);

    mqttBegin();  // สร้างชื่อ topic จาก MAC (ยังไม่เชื่อมต่อ จนกว่า WiFi พร้อม)

    // WiFi + NTP
    WiFi.persistent(false);
    WiFi.mode(WIFI_STA);
    WiFi.setAutoReconnect(false);
    sntp_set_time_sync_notification_cb(onTimeSync);
    sntp_set_sync_interval(app::NTP_RESYNC_MS);
    if (strlen(WIFI_SSID) > 0) {
        startWifiConnect();
    } else {
        Serial.println("[WIFI] include/secrets.h not found (or WIFI_SSID is empty): no real time (the schedule is disabled, the rest works)");
    }

    printHelp();
    if (cal.valid) {
        enterPage(Page::Dashboard);
    } else {
        beginCalibration();
    }
}

void loop()
{
    const uint32_t now = millis();

    // 1) จอสัมผัส (อ่านทุก TOUCH_POLL_MS) แล้วประมวลผลเหตุการณ์กด/ลาก/ปล่อย
    pollTouch(now);
    if (page == Page::Calibrate) {
        if (touch.pressEdge || touch.releaseEdge || touch.touchingNow || calWaitRelease || calMessageUntil != 0) {
            handleCalibration(now);
        }
    } else {
        handleTouchEvents(now);
    }

    // 2) อ่านเซนเซอร์เป็นระยะ (DHT11 ใช้เวลา ~25 ms ระหว่างนั้นไม่ได้อ่านจอสัมผัส)
    if (now - lastReadAt >= app::DHT_READ_INTERVAL_MS) {
        lastReadAt = now;
        readDht();
    }
    if (now - lastLdrAt >= app::LDR_READ_MS) {
        lastLdrAt = now;
        readLdr();
    }

    // 3) ปุ่ม BOOT (หยุดฉุกเฉิน) และคำสั่ง Serial
    if (bootButtonPressed(now)) {
        emergencyStop();
        updateControl(now);
    }
    handleSerialInput(now);

    // 4) WiFi และเวลา NTP
    updateWifi(now);
    updateMqtt(now);
    updateRemoteExpiry(now);
    updateNtp(now);

    // 5) แหล่งข้อมูล: สุ่มค่า SIM และแจ้งเมื่อแหล่งเปลี่ยน
    const DataSource source = currentSource();
    if (source == DataSource::SimRandom && now - lastSimAt >= app::SIM_UPDATE_MS) {
        lastSimAt = now;
        updateSimulationRandom();
    }
    if (source != lastReportedSource) {
        Serial.printf("[SRC] data source: %s -> %s\n", sourceName(lastReportedSource), sourceName(source));
        lastReportedSource = source;
    }

    // 6) ดินชื้น (จำลอง) และตรรกะควบคุมพัดลม/ไฟปลูก/ปั๊ม
    if (now - lastSoilAt >= app::SOIL_UPDATE_MS) {
        updateSoil(static_cast<float>(now - lastSoilAt) / 1000.0f);
        lastSoilAt = now;
    }
    if (now - lastControlAt >= app::CONTROL_UPDATE_MS) {
        lastControlAt = now;
        updateControl(now);
    }

    // 7) วาดจอ (เฉพาะส่วนที่เปลี่ยน ของหน้าที่เปิดอยู่)
    if (page != Page::Calibrate && now - lastDisplayAt >= app::DISPLAY_REFRESH_MS) {
        lastDisplayAt = now;
        updateHeaderTime(now);
        updateHeaderMqtt(now);
        switch (page) {
            case Page::Dashboard: updateDashboard(now); break;
            case Page::Control: updateControlPage(now); break;
            case Page::Setup: updateSetupPage(); break;
            case Page::Mqtt: updateMqttPage(now); break;
            case Page::System: updateSystemPage(now); break;
            case Page::Test: updateTestPage(); break;
            case Page::Calibrate: break;
        }
    }
    if (page == Page::System && now - lastStatusAt >= app::STATUS_REFRESH_MS) {
        lastStatusAt = now;
        updateSystemInfo(now);
    }
    if (now - lastHeartbeatAt >= app::HEARTBEAT_MS) {
        lastHeartbeatAt = now;
        updateHeartbeat();
    }

    if (now - lastSerialAt >= app::SERIAL_PRINT_MS * 5) {  // สรุปสถานะทุก 5 วินาที (ไม่ให้รบกวนบรรทัด [TOUCH])
        lastSerialAt = now;
        printStatus();
    }
    if (now - lastTimePrintAt >= app::TIME_PRINT_MS * 12) {
        lastTimePrintAt = now;
        printTime(now);
    }
}
