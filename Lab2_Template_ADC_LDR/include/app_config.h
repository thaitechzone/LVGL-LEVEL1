#pragma once

// =====================================================================
//  app_config.h -- ค่าที่ "ปรับได้" ของ Lab 2 (ADC + LDR)
//
//  board_config.h = ข้อเท็จจริงของฮาร์ดแวร์ (ไม่แก้)
//  app_config.h   = ค่าที่เราอยากลองปรับ (แก้ได้เลย แล้ว upload ใหม่)
// =====================================================================

#include <Arduino.h>

namespace app {

constexpr uint32_t SERIAL_BAUD = 115200;  // ต้องตรงกับ monitor_speed ใน platformio.ini

// ---- การอ่านค่า LDR ----
constexpr uint8_t OVERSAMPLE_COUNT = 8;    // อ่านดิบกี่ครั้งต่อ 1 ตัวอย่าง แล้วเฉลี่ย (ลด noise)
constexpr uint32_t SAMPLE_MS = 50;         // อ่านค่าทุกกี่ ms
constexpr float SMOOTH_ALPHA = 0.25f;      // ความไวของตัวกรอง 0.0-1.0 (มาก = ไวแต่สั่น, น้อย = นิ่งแต่ช้า)
constexpr uint32_t PRINT_MS = 250;         // พิมพ์ Raw Data ออก Serial ทุกกี่ ms

// ---- Calibration ตอนเปิดเครื่อง ----
// ช่วงนี้ให้ครอบเซนเซอร์ด้วยมือสักครู่ แล้วเปิดมือ เพื่อให้โปรแกรมรู้ช่วงค่ามืด-สว่างของห้องคุณ
constexpr uint32_t CALIBRATION_MS = 6000;
constexpr int MIN_CALIBRATION_RANGE = 8;   // ถ้ามืด-สว่างต่างกันน้อยกว่านี้ ถือว่า calibrate ไม่สำเร็จ
// ค่าสำรองเมื่อ calibrate ไม่สำเร็จ (raw ต่ำ = สว่าง, raw สูง = มืด) เป็นค่าประมาณ ปรับตามบอร์ดคุณได้
constexpr int DEFAULT_RAW_BRIGHT = 0;
constexpr int DEFAULT_RAW_DARK = 120;

// ---- ความเร็วในการเปลี่ยนสี R -> G -> B ----
// มืด (บังแสง) = เร็ว,  สว่าง = ช้า  (หน่วย ms ต่อ 1 สี  ยิ่งน้อยยิ่งเร็ว)
constexpr uint32_t FAST_STEP_MS = 100;
constexpr uint32_t SLOW_STEP_MS = 1200;

}  // namespace app
