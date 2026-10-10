// Level 1 / Lab 3 : WiFi Connecting
//
// เชื่อมต่อ WiFi ด้วยชื่อ (SSID) และรหัสผ่านที่เรากำหนดเองในไฟล์ include/secrets.h
//   - แสดงผลทุกขั้นตอนทาง Serial Monitor (สแกนเครือข่าย, เชื่อมต่อ, ได้ IP, ความแรงสัญญาณ)
//   - ใช้ RGB LED บอกสถานะ:
//        BLUE กะพริบช้า  = กำลังสแกนหา WiFi
//        BLUE กะพริบเร็ว = กำลังเชื่อมต่อ
//        GREEN ติดค้าง   = เชื่อมต่อสำเร็จ
//        RED ติดค้าง     = เชื่อมต่อล้มเหลว (จะลองใหม่เอง)
//        RED กะพริบช้า   = เครือข่ายหลุดหลังเคยเชื่อมสำเร็จ (จะลองใหม่เอง)
//        RED กะพริบเร็ว  = ยังไม่ได้ตั้งค่า SSID/รหัสผ่านใน secrets.h
//
// แนวคิดที่ฝึกใน Lab นี้: WiFi แบบ Station, state machine หลายสถานะ, สแกนแบบ async,
// event callback, การเก็บรหัสผ่านไว้นอก git, และ millis() แทน delay()

#include <Arduino.h>
#include <WiFi.h>

#include "app_config.h"
#include "board_config.h"

// ถ้ายังไม่มีไฟล์ secrets.h โปรแกรมก็ยัง build ได้ แต่จะแจ้งเตือนผ่าน Serial และ LED แทน
#if __has_include("secrets.h")
#include "secrets.h"
constexpr bool HAS_SECRETS_FILE = true;
#else
inline constexpr char WIFI_SSID[] = "";
inline constexpr char WIFI_PASSWORD[] = "";
constexpr bool HAS_SECRETS_FILE = false;
#endif

namespace {

enum class WifiState : uint8_t { Boot, NotConfigured, Scanning, Connecting, Connected, Failed, Lost };

struct LedPattern {
    bool red;
    bool green;
    bool blue;
    uint32_t blinkMs;  // 0 = ติดค้าง, มากกว่า 0 = กะพริบ (ms ต่อครึ่งรอบ)
    const char *text;  // คำอธิบายสำหรับพิมพ์ใน Serial
};

LedPattern patternFor(WifiState state)
{
    switch (state) {
        case WifiState::NotConfigured: return {true, false, false, app::BLINK_FAST_MS, "RED blinking fast"};
        case WifiState::Scanning: return {false, false, true, app::BLINK_SLOW_MS, "BLUE blinking slow"};
        case WifiState::Connecting: return {false, false, true, app::BLINK_FAST_MS, "BLUE blinking fast"};
        case WifiState::Connected: return {false, true, false, 0, "GREEN steady"};
        case WifiState::Failed: return {true, false, false, 0, "RED steady"};
        case WifiState::Lost: return {true, false, false, app::BLINK_SLOW_MS, "RED blinking slow"};
        case WifiState::Boot: break;
    }
    return {false, false, false, 0, "OFF"};
}

const char *stateName(WifiState state)
{
    switch (state) {
        case WifiState::Boot: return "Boot";
        case WifiState::NotConfigured: return "NotConfigured";
        case WifiState::Scanning: return "Scanning";
        case WifiState::Connecting: return "Connecting";
        case WifiState::Connected: return "Connected";
        case WifiState::Failed: return "Failed";
        case WifiState::Lost: return "Lost";
    }
    return "?";
}

WifiState state = WifiState::Boot;
uint32_t stateSince = 0;
uint32_t lastBlinkAt = 0;
uint32_t lastPrintAt = 0;
uint32_t connectStartedAt = 0;
bool blinkOn = true;
uint8_t attempt = 0;

// ตัวแปรนี้ถูกเขียนจาก event callback (ทำงานคนละ task กับ loop) จึงต้องเป็น volatile
volatile uint8_t lastDisconnectReason = 0;

// ---------------------------------------------------------------------
// ข้อความอธิบายสาเหตุที่ WiFi หลุด/เชื่อมไม่ได้
// ---------------------------------------------------------------------

const char *reasonText(uint8_t reason)
{
    switch (reason) {
        case 0: return "(no reason reported)";
        case WIFI_REASON_NO_AP_FOUND: return "NO_AP_FOUND: SSID not found (typo? 5 GHz only? out of range?)";
        case WIFI_REASON_AUTH_FAIL: return "AUTH_FAIL: authentication failed (wrong password?)";
        case WIFI_REASON_4WAY_HANDSHAKE_TIMEOUT:
        case WIFI_REASON_HANDSHAKE_TIMEOUT: return "HANDSHAKE_TIMEOUT: usually a wrong password";
        case WIFI_REASON_ASSOC_FAIL: return "ASSOC_FAIL: the router refused the connection";
        case WIFI_REASON_BEACON_TIMEOUT: return "BEACON_TIMEOUT: lost contact with the router";
        case WIFI_REASON_AUTH_EXPIRE: return "AUTH_EXPIRE: authentication expired";
        default: return "other reason (see wifi_err_reason_t in ESP-IDF)";
    }
}

const char *statusText(wl_status_t status)
{
    switch (status) {
        case WL_IDLE_STATUS: return "IDLE";
        case WL_NO_SSID_AVAIL: return "NO_SSID_AVAIL";
        case WL_SCAN_COMPLETED: return "SCAN_COMPLETED";
        case WL_CONNECTED: return "CONNECTED";
        case WL_CONNECT_FAILED: return "CONNECT_FAILED";
        case WL_CONNECTION_LOST: return "CONNECTION_LOST";
        case WL_DISCONNECTED: return "DISCONNECTED";
        default: return "OTHER";
    }
}

void onWifiEvent(WiFiEvent_t event, WiFiEventInfo_t info)
{
    if (event == ARDUINO_EVENT_WIFI_STA_DISCONNECTED) {
        lastDisconnectReason = info.wifi_sta_disconnected.reason;
    }
}

// ---------------------------------------------------------------------
// ตัวช่วยแสดงผล
// ---------------------------------------------------------------------

// สร้างแถบความแรงสัญญาณ 10 ช่อง จาก RSSI (ประมาณ -90 dBm = แทบไม่มี ถึง -30 dBm = แรงมาก)
void makeRssiBar(int rssi, char *out)
{
    const int filled = static_cast<int>(constrain(map(rssi, -90, -30, 0, 10), 0L, 10L));
    for (int i = 0; i < 10; i++) {
        out[i] = (i < filled) ? '#' : '-';
    }
    out[10] = '\0';
}

const char *rssiQuality(int rssi)
{
    if (rssi >= -55) return "strong";
    if (rssi >= -70) return "good";
    if (rssi >= -80) return "weak";
    return "very weak";
}

// เปลี่ยนสถานะ + พิมพ์ว่า LED จะเป็นอย่างไร ให้ผู้เรียนเห็นความเชื่อมโยงระหว่างสถานะกับไฟ
void setState(WifiState next)
{
    const uint32_t now = millis();
    Serial.printf("[STATE] %s -> %s   (LED: %s)\n", stateName(state), stateName(next), patternFor(next).text);
    state = next;
    stateSince = now;
    lastBlinkAt = now;
    lastPrintAt = now;
    blinkOn = true;
}

// อัปเดต LED ตามสถานะปัจจุบัน (ไม่ใช้ delay: สลับติด/ดับด้วย millis)
void updateLed(uint32_t now)
{
    const LedPattern pattern = patternFor(state);
    if (pattern.blinkMs == 0) {
        cyd::ledSet(pattern.red, pattern.green, pattern.blue);
        return;
    }
    if (now - lastBlinkAt >= pattern.blinkMs) {
        lastBlinkAt = now;
        blinkOn = !blinkOn;
    }
    cyd::ledSet(blinkOn && pattern.red, blinkOn && pattern.green, blinkOn && pattern.blue);
}

// ---------------------------------------------------------------------
// ขั้นตอนของ WiFi
// ---------------------------------------------------------------------

void startScan()
{
    attempt++;
    Serial.printf("[WiFi] attempt #%u: scanning for networks ...\n", attempt);
    WiFi.scanDelete();
    WiFi.scanNetworks(true);  // true = สแกนแบบ async: สั่งแล้วไม่รอ ให้ loop() ถามผลทีหลัง
    setState(WifiState::Scanning);
}

// พิมพ์รายการ WiFi ที่เจอ และคืนค่า true ถ้าเจอชื่อ WiFi ของเรา
bool printScanResults(int count)
{
    Serial.printf("[SCAN] found %d network(s):\n", count);
    if (count == 0) {
        Serial.println("       (none) - move closer to the router?");
        return false;
    }
    Serial.println("        #  SSID                      CH   RSSI  signal       security");

    bool found = false;
    for (int i = 0; i < count; i++) {
        const String ssid = WiFi.SSID(i);
        const bool mine = (ssid == WIFI_SSID);
        if (mine) {
            found = true;
        }
        if (i < app::MAX_SCAN_RESULTS_PRINTED) {
            char bar[11];
            makeRssiBar(WiFi.RSSI(i), bar);
            Serial.printf("       %2d  %-24.24s %3d  %4d  [%s]  %s%s\n", i + 1, ssid.c_str(), WiFi.channel(i),
                          WiFi.RSSI(i), bar, (WiFi.encryptionType(i) == WIFI_AUTH_OPEN) ? "open" : "secured",
                          mine ? "   <== YOUR NETWORK" : "");
        }
    }
    if (count > app::MAX_SCAN_RESULTS_PRINTED) {
        Serial.printf("       ... and %d more\n", count - app::MAX_SCAN_RESULTS_PRINTED);
    }
    return found;
}

void beginConnect()
{
    lastDisconnectReason = 0;
    connectStartedAt = millis();
    Serial.printf("[WiFi] connecting to \"%s\" (password length: %u) ...\n", WIFI_SSID,
                  static_cast<unsigned>(strlen(WIFI_PASSWORD)));
    WiFi.begin(WIFI_SSID, strlen(WIFI_PASSWORD) > 0 ? WIFI_PASSWORD : nullptr);
    setState(WifiState::Connecting);
}

void onConnected()
{
    const float seconds = (millis() - connectStartedAt) / 1000.0f;
    char bar[11];
    makeRssiBar(WiFi.RSSI(), bar);

    Serial.printf("[WiFi] CONNECTED in %.1f s\n", seconds);
    Serial.printf("       SSID    : %s\n", WiFi.SSID().c_str());
    Serial.printf("       IP      : %s\n", WiFi.localIP().toString().c_str());
    Serial.printf("       Gateway : %s\n", WiFi.gatewayIP().toString().c_str());
    Serial.printf("       Subnet  : %s\n", WiFi.subnetMask().toString().c_str());
    Serial.printf("       DNS     : %s\n", WiFi.dnsIP().toString().c_str());
    Serial.printf("       MAC     : %s\n", WiFi.macAddress().c_str());
    Serial.printf("       Channel : %d\n", WiFi.channel());
    Serial.printf("       RSSI    : %d dBm [%s] %s\n", WiFi.RSSI(), bar, rssiQuality(WiFi.RSSI()));
    setState(WifiState::Connected);
}

void failConnect(const char *why, bool showReason)
{
    Serial.printf("[WiFi] FAILED: %s\n", why);
    if (showReason) {
        Serial.printf("[WiFi] reason code %u -> %s\n", static_cast<unsigned>(lastDisconnectReason),
                      reasonText(lastDisconnectReason));
    }
    Serial.printf("[WiFi] will retry in %lu s ...\n", static_cast<unsigned long>(app::RETRY_DELAY_MS / 1000));
    WiFi.disconnect(false);
    setState(WifiState::Failed);
}

void printLedLegend()
{
    Serial.println("LED legend:");
    Serial.println("  BLUE  blinking slow = scanning for WiFi networks");
    Serial.println("  BLUE  blinking fast = connecting to your WiFi");
    Serial.println("  GREEN steady        = connected (got an IP address)");
    Serial.println("  RED   steady        = failed (will retry)");
    Serial.println("  RED   blinking slow = connection lost (will retry)");
    Serial.println("  RED   blinking fast = WiFi name/password not set in include/secrets.h");
}

// ทดสอบ LED R -> G -> B ตอนเปิดเครื่อง เพื่อให้รู้ว่าไฟแต่ละสีใช้งานได้ก่อนเริ่ม WiFi
// (ใช้ delay ได้ที่นี่เพราะอยู่ใน setup ครั้งเดียว และยังไม่มีงานอื่นรออยู่)
void ledSelfTest()
{
    Serial.println("LED self-test: RED -> GREEN -> BLUE");
    cyd::ledSet(true, false, false);
    delay(app::SELF_TEST_STEP_MS);
    cyd::ledSet(false, true, false);
    delay(app::SELF_TEST_STEP_MS);
    cyd::ledSet(false, false, true);
    delay(app::SELF_TEST_STEP_MS);
    cyd::ledSet(false, false, false);
}

}  // namespace

void setup()
{
    Serial.begin(app::SERIAL_BAUD);
    delay(200);

    Serial.println();
    Serial.println("==========================================");
    Serial.println("Hello World from ESP32-2432S028 (CYD)!");
    Serial.println("Level 1 / Lab 3 : WiFi Connecting");
    Serial.println("==========================================");
    printLedLegend();
    Serial.println();

    cyd::ledInit();
    ledSelfTest();

    WiFi.persistent(false);         // ไม่เขียนชื่อ/รหัส WiFi ลงหน่วยความจำถาวรของชิป
    WiFi.mode(WIFI_STA);            // โหมด Station = บอร์ดเป็นลูกข่ายที่ไปต่อกับ router
    WiFi.setAutoReconnect(false);   // เราคุมการเชื่อมต่อใหม่เองใน state machine
    WiFi.onEvent(onWifiEvent);

    if (strlen(WIFI_SSID) == 0) {
        setState(WifiState::NotConfigured);
    } else {
        Serial.printf("[WiFi] target SSID: \"%s\"   MAC of this board: %s\n", WIFI_SSID, WiFi.macAddress().c_str());
        if (strlen(WIFI_PASSWORD) > 0 && strlen(WIFI_PASSWORD) < 8) {
            Serial.println("[WiFi] warning: WPA passwords are at least 8 characters - check WIFI_PASSWORD");
        }
        startScan();
    }
}

void loop()
{
    const uint32_t now = millis();
    updateLed(now);

    switch (state) {
        case WifiState::NotConfigured:
            if (now - lastPrintAt >= 5000) {
                lastPrintAt = now;
                if (!HAS_SECRETS_FILE) {
                    Serial.println("[WiFi] include/secrets.h not found. Copy include/secrets.h.example to "
                                   "include/secrets.h, fill in WIFI_SSID and WIFI_PASSWORD, then upload again.");
                } else {
                    Serial.println("[WiFi] WIFI_SSID in include/secrets.h is empty. Fill in your WiFi name and "
                                   "password, then upload again.");
                }
            }
            break;

        case WifiState::Scanning: {
            const int count = WiFi.scanComplete();
            if (count < 0) {
                // -1 = ยังสแกนอยู่, -2 = ไลบรารีบอกว่า "ล้มเหลว"  แต่ไลบรารีตัดสินว่าล้มเหลวเร็วไป
                // (ถ้าสแกนนานเกิน 6 วินาที ทั้งที่สแกนยังเดินต่อและเสร็จทีหลังได้)
                // เราจึงรอเองได้นานถึง SCAN_TIMEOUT_MS (LED ยังกะพริบได้เพราะไม่ได้หยุดรอ)
                if (now - stateSince >= app::SCAN_TIMEOUT_MS) {
                    failConnect("the network scan did not finish in time", false);
                }
                break;
            }
            const bool found = printScanResults(count);
            WiFi.scanDelete();
            if (found) {
                beginConnect();
            } else {
                Serial.printf("[WiFi] \"%s\" was NOT found in the scan. Check the spelling (upper/lower case, spaces) "
                              "and that the router uses 2.4 GHz.\n",
                              WIFI_SSID);
                failConnect("SSID not found", false);
            }
            break;
        }

        case WifiState::Connecting: {
            const wl_status_t status = WiFi.status();
            if (status == WL_CONNECTED) {
                onConnected();
            } else if (status == WL_CONNECT_FAILED) {
                failConnect("the router refused the connection (wrong password?)", true);
            } else if (now - connectStartedAt >= app::CONNECT_TIMEOUT_MS) {
                failConnect("timeout - no connection in time", true);
            } else if (now - lastPrintAt >= app::CONNECTING_PRINT_MS) {
                lastPrintAt = now;
                Serial.printf("[WiFi] connecting ... %lu s (status: %s)\n",
                              static_cast<unsigned long>((now - connectStartedAt) / 1000), statusText(status));
            }
            break;
        }

        case WifiState::Connected:
            if (WiFi.status() != WL_CONNECTED) {
                Serial.printf("[WiFi] connection LOST (reason code %u -> %s)\n",
                              static_cast<unsigned>(lastDisconnectReason), reasonText(lastDisconnectReason));
                Serial.printf("[WiFi] will retry in %lu s ...\n", static_cast<unsigned long>(app::RETRY_DELAY_MS / 1000));
                setState(WifiState::Lost);
            } else if (now - lastPrintAt >= app::STATUS_PRINT_MS) {
                lastPrintAt = now;
                char bar[11];
                makeRssiBar(WiFi.RSSI(), bar);
                Serial.printf("[WiFi] online  IP %s   RSSI %d dBm [%s] %s\n", WiFi.localIP().toString().c_str(),
                              WiFi.RSSI(), bar, rssiQuality(WiFi.RSSI()));
            }
            break;

        case WifiState::Failed:
        case WifiState::Lost:
            if (now - stateSince >= app::RETRY_DELAY_MS) {
                startScan();
            }
            break;

        case WifiState::Boot:
            break;
    }
}
