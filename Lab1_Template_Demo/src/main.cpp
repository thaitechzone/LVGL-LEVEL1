// Level 1 / Lab 1 : Template Demo
//
// โปรแกรมแรกของบอร์ด ESP32-2432S028 (CYD)
//   1) พิมพ์ "Hello World" ออกทาง Serial Monitor
//   2) ให้ RGB LED บนบอร์ดแสดงสี แดง -> เขียว -> น้ำเงิน ตามลำดับ แล้ววนซ้ำ
//      พร้อมพิมพ์ชื่อสีและเลข GPIO ที่กำลังติดออกทาง Serial ให้ตรงกับสีที่เห็น
//
// แนวคิดที่ฝึกใน Lab นี้: setup()/loop(), Serial, digitalWrite ผ่านฟังก์ชันช่วย,
// และการจับเวลาด้วย millis() แทน delay() (โปรแกรมจึงไม่ค้างระหว่างรอ)

#include <Arduino.h>

#include "app_config.h"
#include "board_config.h"

namespace {

struct LedStep {
    const char *name;
    uint8_t pin;  // GPIO ที่ขับสีนี้ (แสดงใน Serial เพื่อเทียบกับสีที่เห็น)
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

size_t currentStep = 0;
uint32_t lastStepAt = 0;
uint32_t helloCount = 0;

void printBanner()
{
    Serial.println();
    Serial.println("==========================================");
    Serial.printf("%s from ESP32-2432S028 (CYD)!\n", app::HELLO_TEXT);
    Serial.println("Level 1 / Lab 1 : Template Demo");
    Serial.printf("Chip : %s rev %d, CPU %lu MHz\n", ESP.getChipModel(), ESP.getChipRevision(),
                  static_cast<unsigned long>(ESP.getCpuFreqMHz()));
    Serial.printf("Flash: %lu KB, free heap: %lu bytes\n", static_cast<unsigned long>(ESP.getFlashChipSize() / 1024),
                  static_cast<unsigned long>(ESP.getFreeHeap()));
    Serial.println("==========================================");
}

void showStep(const LedStep &step)
{
    cyd::ledSet(step.red, step.green, step.blue);

    helloCount++;
    Serial.printf("%s #%lu  LED = %s (GPIO%u)\n", app::HELLO_TEXT, static_cast<unsigned long>(helloCount), step.name,
                  step.pin);
}

}  // namespace

void setup()
{
    Serial.begin(app::SERIAL_BAUD);
    delay(200);  // รอให้ Serial พร้อมก่อนพิมพ์ข้อความแรก

    printBanner();

    cyd::ledInit();
    showStep(steps[currentStep]);
    lastStepAt = millis();
}

void loop()
{
    const uint32_t now = millis();
    if (now - lastStepAt >= app::LED_STEP_MS) {
        lastStepAt = now;
        currentStep = (currentStep + 1) % STEP_COUNT;
        showStep(steps[currentStep]);
    }
}
