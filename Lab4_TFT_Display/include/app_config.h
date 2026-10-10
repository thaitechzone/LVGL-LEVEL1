#pragma once

// =====================================================================
//  app_config.h -- ค่าที่ "ปรับได้" ของ Lab 4 (TFT Display)
//
//  board_config.h = ข้อเท็จจริงของฮาร์ดแวร์ (ไม่แก้)
//  app_config.h   = ค่าที่เราอยากลองปรับ (แก้ได้เลย แล้ว upload ใหม่)
//  secrets.h      = ชื่อ WiFi + รหัสผ่านของคุณ (สร้างเอง ไม่ขึ้น GitHub)
// =====================================================================

#include <Arduino.h>

namespace app {

constexpr uint32_t SERIAL_BAUD = 115200;  // ต้องตรงกับ monitor_speed ใน platformio.ini

// ---- การอ่านค่าแสง LDR (เหมือน Lab 2) ----
constexpr uint8_t OVERSAMPLE_COUNT = 8;     // อ่านซ้ำกี่ครั้งต่อ 1 ตัวอย่าง แล้วเฉลี่ย
constexpr uint32_t SAMPLE_MS = 50;          // อ่านค่าทุกกี่ ms
constexpr float SMOOTH_ALPHA = 0.25f;       // ความไวของตัวกรอง 0.0-1.0
constexpr uint32_t CALIBRATION_MS = 6000;   // เวลา calibrate ตอนเปิดเครื่อง (ครอบมือแล้วเปิดมือ)
constexpr int MIN_CALIBRATION_RANGE = 8;    // ช่วงมืด-สว่างขั้นต่ำที่ถือว่า calibrate สำเร็จ
constexpr int DEFAULT_RAW_BRIGHT = 0;       // ค่าสำรอง (raw ต่ำ = สว่าง)
constexpr int DEFAULT_RAW_DARK = 120;       // ค่าสำรอง (raw สูง = มืด)

// ---- การเชื่อมต่อ WiFi (แบบย่อจาก Lab 3) ----
constexpr uint32_t CONNECT_TIMEOUT_MS = 15000;  // เชื่อมไม่สำเร็จภายในกี่ ms ถือว่าล้มเหลว
constexpr uint32_t RETRY_DELAY_MS = 5000;       // รอกี่ ms ก่อนลองใหม่

// ---- การวาดจอ ----
constexpr uint32_t LIGHT_REFRESH_MS = 100;   // อัปเดตส่วนค่าแสงบนจอทุกกี่ ms
constexpr uint32_t STATUS_REFRESH_MS = 1000; // อัปเดตส่วน WiFi / uptime ทุกกี่ ms
constexpr uint32_t HEARTBEAT_MS = 500;       // จุดกะพริบมุมล่างขวา (บอกว่าโปรแกรมยังทำงานอยู่)
constexpr uint32_t SERIAL_PRINT_MS = 1000;   // พิมพ์สรุปออก Serial ทุกกี่ ms
constexpr uint8_t LIGHT_DARK_BELOW = 35;     // ต่ำกว่านี้ (%) ถือว่า "DARK"
constexpr uint8_t LIGHT_BRIGHT_ABOVE = 65;   // สูงกว่านี้ (%) ถือว่า "BRIGHT"

}  // namespace app
