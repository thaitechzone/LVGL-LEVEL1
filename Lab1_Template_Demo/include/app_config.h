#pragma once

// =====================================================================
//  app_config.h -- ค่าที่ "ปรับได้" ของแต่ละ Lab
//
//  board_config.h = ข้อเท็จจริงของฮาร์ดแวร์ (ไม่แก้)
//  app_config.h   = ค่าที่เราอยากลองปรับ เช่น ความเร็ว ข้อความ (แก้ได้เลย)
// =====================================================================

#include <Arduino.h>

namespace app {

constexpr uint32_t SERIAL_BAUD = 115200;  // ต้องตรงกับ monitor_speed ใน platformio.ini

// เวลาที่ LED แต่ละสีค้างอยู่ (มิลลิวินาที) ลองเปลี่ยนเป็น 200 หรือ 1000 แล้ว upload ใหม่
constexpr uint32_t LED_STEP_MS = 2000;

constexpr char HELLO_TEXT[] = "Hello World";

}  // namespace app
