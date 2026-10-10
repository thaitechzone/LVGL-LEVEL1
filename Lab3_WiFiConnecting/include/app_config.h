#pragma once

// =====================================================================
//  app_config.h -- ค่าที่ "ปรับได้" ของ Lab 3 (WiFi Connecting)
//
//  board_config.h = ข้อเท็จจริงของฮาร์ดแวร์ (ไม่แก้)
//  app_config.h   = ค่าที่เราอยากลองปรับ (แก้ได้เลย แล้ว upload ใหม่)
//  secrets.h      = ชื่อ WiFi + รหัสผ่านของคุณ (สร้างเอง ไม่ขึ้น GitHub)
// =====================================================================

#include <Arduino.h>

namespace app {

constexpr uint32_t SERIAL_BAUD = 115200;  // ต้องตรงกับ monitor_speed ใน platformio.ini

// ---- การเชื่อมต่อ WiFi ----
constexpr uint32_t CONNECT_TIMEOUT_MS = 15000;  // เชื่อมไม่สำเร็จภายในกี่ ms ถือว่าล้มเหลว
constexpr uint32_t RETRY_DELAY_MS = 5000;       // รอกี่ ms ก่อนลองใหม่หลังล้มเหลว/หลุด
constexpr uint32_t STATUS_PRINT_MS = 5000;      // ตอนเชื่อมแล้ว พิมพ์สัญญาณ (RSSI) ทุกกี่ ms
constexpr uint32_t CONNECTING_PRINT_MS = 1000;  // ตอนกำลังเชื่อม พิมพ์ความคืบหน้าทุกกี่ ms
constexpr int MAX_SCAN_RESULTS_PRINTED = 15;    // แสดงรายชื่อ WiFi ที่สแกนเจอสูงสุดกี่รายการ
constexpr uint32_t SCAN_TIMEOUT_MS = 15000;     // รอผลสแกนนานสุดกี่ ms (การสแกนปกติใช้ราว 3-7 วินาที)

// ---- ไฟ LED บอกสถานะ ----
constexpr uint32_t BLINK_FAST_MS = 150;         // จังหวะกะพริบเร็ว (ms ต่อครึ่งรอบ)
constexpr uint32_t BLINK_SLOW_MS = 500;         // จังหวะกะพริบช้า
constexpr uint32_t SELF_TEST_STEP_MS = 300;     // ทดสอบ LED ตอนเปิดเครื่อง R -> G -> B สีละกี่ ms

}  // namespace app
