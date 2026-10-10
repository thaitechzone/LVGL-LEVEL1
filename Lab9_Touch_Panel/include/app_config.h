#pragma once

// =====================================================================
//  app_config.h -- ค่าที่ "ปรับได้" ของ Lab 9 (Touch Panel)
//
//  board_config.h = ข้อเท็จจริงของฮาร์ดแวร์ (ไม่แก้)
//  app_config.h   = ค่าที่เราอยากลองปรับ (แก้ได้เลย แล้ว upload ใหม่)
// =====================================================================

#include <Arduino.h>

namespace app {

constexpr uint32_t SERIAL_BAUD = 115200;  // ต้องตรงกับ monitor_speed ใน platformio.ini

// ---- เซนเซอร์ DHT11 ----
constexpr uint32_t DHT_READ_INTERVAL_MS = 2500;  // อ่านทุกกี่ ms (DHT11 อ่านถี่กว่า ~1 วินาทีไม่ได้)
constexpr uint8_t DHT_FAIL_LIMIT = 3;            // อ่านพลาดติดกันกี่ครั้งถึงถือว่า "เซนเซอร์มีปัญหา"

// ---- พัดลมจำลอง (LED น้ำเงินคุมด้วย PWM: duty มาก = พัดลมหมุนเร็ว) ----
constexpr float FAN_ON_TEMP_C = 30.0f;    // อุณหภูมิ "ตั้งแต่นี้ขึ้นไป" พัดลมเริ่มหมุน
constexpr float FAN_OFF_TEMP_C = 28.0f;   // อุณหภูมิ "ต่ำลงมาถึงนี้" พัดลมหยุด (ต้องต่ำกว่า ON = hysteresis)
constexpr float FAN_FULL_TEMP_C = 36.0f;  // อุณหภูมิที่พัดลมหมุนเต็มที่ (duty 100%)
constexpr int FAN_MIN_DUTY_PERCENT = 30;  // duty ต่ำสุดตอนพัดลมหมุน (พัดลมจริงมักหมุนไม่ออกถ้า duty ต่ำเกินไป)
constexpr int MANUAL_STEP_PERCENT = 10;   // โหมด MANUAL กด + หรือ - แล้วเปลี่ยนทีละกี่ %
constexpr uint32_t FAN_UPDATE_MS = 200;   // ตรวจเงื่อนไขพัดลมทุกกี่ ms

// ---- PWM (LEDC) ----
constexpr uint8_t PWM_RESOLUTION_BITS = 8;  // 8 บิต = ค่า 0-255
constexpr uint32_t PWM_MAX_VALUE = (1UL << PWM_RESOLUTION_BITS) - 1;
constexpr uint32_t PWM_FREQUENCY_HZ = 5000;
constexpr uint8_t FAN_CHANNEL = 0;          // ช่อง LEDC ที่คุม LED น้ำเงิน (GPIO17) ส่วน LED แดง (GPIO4) ไม่ใช้ PWM

// ---- ค่าจำลอง (SIM) ไว้ใช้เมื่อไม่ได้ต่อเซนเซอร์ หรืออยากทดสอบพัดลมโดยไม่ต้องรอความร้อนจริง ----
// กด s ใน Serial เพื่อสลับ: REAL (เซนเซอร์จริง) -> SIM random (สุ่ม) -> SIM sweep (ไล่ขึ้น-ลงสม่ำเสมอ) -> REAL ...
constexpr bool SIM_WHEN_NO_SENSOR = true;   // true = ถ้าเซนเซอร์มีปัญหา (อ่านพลาดติดกัน) ให้ใช้ค่า SIM random ให้เองและป้ายขึ้น SIM
constexpr uint32_t SIM_UPDATE_MS = 1000;    // สุ่มค่าใหม่ทุกกี่ ms
constexpr float SIM_TEMP_MIN_C = 24.0f;     // ขอบเขตอุณหภูมิจำลอง
constexpr float SIM_TEMP_MAX_C = 38.0f;
constexpr float SIM_TEMP_START_C = 28.0f;   // อุณหภูมิเริ่มต้นของ SIM random
constexpr float SIM_TEMP_STEP_C = 1.5f;     // SIM random: แต่ละครั้งอุณหภูมิเปลี่ยนสุ่มไม่เกิน +-นี้
constexpr float SIM_HUM_MIN = 40.0f;        // ขอบเขตความชื้นจำลอง (%)
constexpr float SIM_HUM_MAX = 90.0f;
constexpr float SIM_HUM_START = 65.0f;
constexpr float SIM_HUM_STEP = 3.0f;        // SIM random: แต่ละครั้งความชื้นเปลี่ยนสุ่มไม่เกิน +-นี้
constexpr uint32_t SIM_PERIOD_MS = 60000;   // SIM sweep: อุณหภูมิขึ้นแล้วลง 1 รอบใช้เวลากี่ ms (ความชื้นไล่สวนทางกัน)

// ---- การเชื่อมต่อ WiFi (แบบย่อจาก Lab 3: ไม่สแกน ต่อตรงๆ แล้วลองใหม่เมื่อล้มเหลว) ----
constexpr uint32_t CONNECT_TIMEOUT_MS = 15000;  // เชื่อมไม่สำเร็จภายในกี่ ms ถือว่าล้มเหลว
constexpr uint32_t RETRY_DELAY_MS = 5000;       // รอกี่ ms ก่อนลองใหม่

// ---- เวลาจริงจากอินเทอร์เน็ต (NTP) ----
inline constexpr char NTP_SERVER_1[] = "pool.ntp.org";  // เซิร์ฟเวอร์หลัก
inline constexpr char NTP_SERVER_2[] = "time.nist.gov"; // เซิร์ฟเวอร์สำรอง
constexpr long GMT_OFFSET_SEC = 7 * 3600;   // เขตเวลา UTC+7 (ไทย) หน่วยวินาที
constexpr int DAYLIGHT_OFFSET_SEC = 0;      // ไทยไม่มีเวลาออมแสง
constexpr uint32_t NTP_TIMEOUT_MS = 20000;  // เริ่มขอเวลาแล้วไม่ได้รับภายในกี่ ms ถือว่าล้มเหลว
constexpr uint32_t NTP_RETRY_MS = 30000;    // ล้มเหลวแล้วรอกี่ ms ก่อนขอใหม่
constexpr uint32_t NTP_RESYNC_MS = 3600000; // ขอเวลาใหม่เป็นระยะทุกกี่ ms (3600000 = 1 ชั่วโมง)
constexpr uint32_t NTP_STALE_FACTOR = 2;    // เวลาที่ sync ล่าสุดเก่ากว่า NTP_RESYNC_MS x ค่านี้ = ป้าย STALE
constexpr uint32_t SANE_EPOCH = 1700000000UL;  // เวลาที่เก่ากว่านี้ (ก่อน ~พ.ย. 2023) ถือว่ายังไม่ถูกตั้ง (ก่อน sync จะเป็นปี 1970)
constexpr uint32_t TIME_PRINT_MS = 5000;    // พิมพ์เวลาออก Serial ทุกกี่ ms

// ---- จอสัมผัส XPT2046 (อ่านค่าผ่าน SPI) ----
constexpr uint32_t TOUCH_POLL_MS = 10;       // อ่านจอสัมผัสทุกกี่ ms
constexpr int TOUCH_Z_MIN = 400;             // แรงกดขั้นต่ำที่ถือว่า "แตะ" (ค่าเล็กกว่านี้ = ไม่มีการแตะ)
constexpr uint8_t TOUCH_PRESS_SAMPLES = 2;   // ต้องเห็นการแตะติดกันกี่ครั้งถึงนับว่า "กดลง" (กันสัญญาณรบกวน)
constexpr uint8_t TOUCH_RELEASE_SAMPLES = 3; // ต้องไม่เห็นการแตะติดกันกี่ครั้งถึงนับว่า "ปล่อย"
constexpr float TOUCH_SMOOTH_ALPHA = 0.5f;   // ความไวของตัวกรองพิกัดขณะลากนิ้ว (0.0-1.0 ยิ่งน้อยยิ่งนิ่ง)

// ---- การ calibrate (แตะเป้า 4 มุม) ----
constexpr int CAL_MARGIN = 20;               // เป้าห่างขอบจอกี่พิกเซล
constexpr uint8_t CAL_SAMPLES = 10;          // เก็บตัวอย่างกี่ครั้งต่อ 1 เป้า (แล้วเฉลี่ย)
constexpr int CAL_MIN_SPAN = 300;            // ช่วงค่าดิบขั้นต่ำระหว่างขอบซ้าย-ขวา/บน-ล่าง (น้อยกว่านี้ = แตะผิดที่)
constexpr int CAL_MAX_ERROR_PX = 25;         // ตำแหน่งที่คำนวณย้อนกลับของเป้าต้องคลาดเคลื่อนไม่เกินกี่พิกเซล

// ---- ปุ่มและแถบเลื่อน ----
constexpr uint32_t LONG_PRESS_MS = 1000;     // ปุ่มสำคัญ (Recalibrate) ต้องกดค้างกี่ ms
constexpr uint32_t REPEAT_DELAY_MS = 500;    // กดปุ่ม + หรือ - ค้างนานเท่านี้แล้วเริ่มเพิ่มต่อเนื่อง
constexpr uint32_t REPEAT_INTERVAL_MS = 150; // เพิ่ม/ลดซ้ำทุกกี่ ms ขณะกดค้าง
constexpr float THRESHOLD_MIN_C = 15.0f;     // ช่วงที่ปรับเกณฑ์อุณหภูมิพัดลมได้
constexpr float THRESHOLD_MAX_C = 45.0f;

// ---- ความสว่างจอ (PWM ที่ GPIO21 เหมือน Lab 5) ----
constexpr uint8_t BACKLIGHT_CHANNEL = 2;     // ช่อง LEDC ของไฟหลังจอ (ช่อง 0 ใช้กับพัดลม)
constexpr int BACKLIGHT_MIN_PERCENT = 10;    // ต่ำสุดที่ให้ตั้งได้ (ต่ำกว่านี้จอมืดจนแตะปุ่มไม่เห็น)
constexpr int BACKLIGHT_DEFAULT_PERCENT = 100;

// ---- จอและจับเวลา ----
constexpr uint32_t DISPLAY_REFRESH_MS = 100;  // วาดจอส่วนค่า/พัดลมใหม่ได้ไม่เกินทุกกี่ ms (วาดเมื่อค่าเปลี่ยนเท่านั้น)
constexpr uint32_t STATUS_REFRESH_MS = 1000;  // อัปเดตบรรทัดสถานะท้ายจอทุกกี่ ms
constexpr uint32_t HEARTBEAT_MS = 500;
constexpr uint32_t SERIAL_PRINT_MS = 1000;
constexpr uint32_t BUTTON_DEBOUNCE_MS = 40;

}  // namespace app
