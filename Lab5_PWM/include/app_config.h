#pragma once

// =====================================================================
//  app_config.h -- ค่าที่ "ปรับได้" ของ Lab 5 (PWM)
//
//  board_config.h = ข้อเท็จจริงของฮาร์ดแวร์ (ไม่แก้)
//  app_config.h   = ค่าที่เราอยากลองปรับ (แก้ได้เลย แล้ว upload ใหม่)
// =====================================================================

#include <Arduino.h>

namespace app {

constexpr uint32_t SERIAL_BAUD = 115200;  // ต้องตรงกับ monitor_speed ใน platformio.ini

// ---- PWM (ตัวสร้างสัญญาณ LEDC ของ ESP32) ----
constexpr uint8_t PWM_RESOLUTION_BITS = 8;  // ความละเอียดของ duty: 8 บิต = 0-255 (256 ระดับ)
constexpr uint32_t PWM_MAX_VALUE = (1UL << PWM_RESOLUTION_BITS) - 1;
constexpr uint8_t BACKLIGHT_CHANNEL = 0;    // ช่อง LEDC ที่คุมไฟหลังจอ (GPIO21)
constexpr uint8_t LED_CHANNEL = 1;          // ช่อง LEDC ที่คุม LED สีเขียว (GPIO16) ใช้ความถี่เดียวกับช่อง 0

// ความถี่ที่กดปุ่ม f เพื่อสลับ (หน่วย Hz) ความถี่ต่ำๆ จะเห็นไฟกะพริบ
constexpr uint32_t PWM_FREQUENCIES[] = {100, 500, 1000, 5000, 20000};
constexpr uint8_t PWM_FREQUENCY_COUNT = sizeof(PWM_FREQUENCIES) / sizeof(PWM_FREQUENCIES[0]);
constexpr uint8_t DEFAULT_FREQUENCY_INDEX = 3;  // 5000 Hz

// ---- Duty cycle (เป็น %) ----
constexpr int MIN_DUTY_PERCENT = 5;       // ต่ำสุดที่ให้ตั้งได้ (0% = จอดับสนิท ดูไม่รู้ว่าโปรแกรมยังทำงานอยู่)
constexpr int DEFAULT_DUTY_PERCENT = 50;  // ค่าเริ่มต้นหลังเข้าหน้าหลัก
constexpr int DUTY_STEP_PERCENT = 5;      // กด + หรือ - แล้วเปลี่ยนทีละกี่ %

// ---- โหมด Breathe (ไล่แสงขึ้น-ลงนุ่มๆ) ----
constexpr uint32_t BREATHE_PERIOD_MS = 4000;  // 1 รอบ (มืด → สว่าง → มืด) ใช้เวลากี่ ms

// ---- จับเวลาและปุ่ม ----
constexpr uint32_t PWM_UPDATE_MS = 20;       // คำนวณ duty ของโหมดอัตโนมัติทุกกี่ ms
constexpr uint32_t DISPLAY_REFRESH_MS = 50;  // วาดจอใหม่ได้ไม่เกินทุกกี่ ms (วาดเมื่อมีค่าเปลี่ยนเท่านั้น)
constexpr uint32_t HEARTBEAT_MS = 500;       // จุดกะพริบมุมล่างขวา
constexpr uint32_t SERIAL_PRINT_MS = 1000;   // พิมพ์สรุปออก Serial ทุกกี่ ms
constexpr uint32_t BUTTON_DEBOUNCE_MS = 40;  // กันปุ่มเด้ง

// ---- LDR สำหรับโหมด Auto (ย่อมาจาก Lab 2/4) ----
constexpr uint8_t OVERSAMPLE_COUNT = 8;
constexpr uint32_t SAMPLE_MS = 50;
constexpr float SMOOTH_ALPHA = 0.25f;
constexpr uint32_t CALIBRATION_MS = 6000;  // ช่วง calibrate ตอนเปิดเครื่อง (ครอบมือแล้วเปิดมือ)
constexpr int MIN_CALIBRATION_RANGE = 8;
constexpr int DEFAULT_RAW_BRIGHT = 0;
constexpr int DEFAULT_RAW_DARK = 120;

}  // namespace app
