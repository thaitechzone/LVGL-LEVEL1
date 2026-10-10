#pragma once

// =====================================================================
//  app_config.h -- ค่าที่ "ปรับได้" ของ Lab 11 (MQTT Farm)
//
//  board_config.h = ข้อเท็จจริงของฮาร์ดแวร์ (ไม่แก้)
//  app_config.h   = ค่าที่เราอยากลองปรับ (แก้ได้เลย แล้ว upload ใหม่)
//
//  หมายเหตุ: เกณฑ์ที่ปรับจากหน้าจอได้ (หน้า SETUP) คือ "ค่าเริ่มต้น" ที่อยู่ในไฟล์นี้
//  ค่าที่ผู้ใช้ปรับจะถูกเก็บถาวร (Preferences) และใช้แทนค่าเริ่มต้นหลังเปิดเครื่อง
// =====================================================================

#include <Arduino.h>

namespace app {

constexpr uint32_t SERIAL_BAUD = 115200;  // ต้องตรงกับ monitor_speed ใน platformio.ini

// ---- เซนเซอร์ DHT11 ----
constexpr uint32_t DHT_READ_INTERVAL_MS = 2500;  // อ่านทุกกี่ ms (DHT11 อ่านถี่กว่า ~1 วินาทีไม่ได้)
constexpr uint8_t DHT_FAIL_LIMIT = 3;            // อ่านพลาดติดกันกี่ครั้งถึงถือว่า "เซนเซอร์มีปัญหา"

// ---- ตัวควบคุม 3 อุปกรณ์ (LED บนบอร์ดแทนอุปกรณ์จริง ทุกสี active-LOW) ----
//   LED น้ำเงิน GPIO17 = พัดลม (PWM)   LED เขียว GPIO16 = ไฟปลูก (เปิด/ปิด)   LED แดง GPIO4 = ปั๊มน้ำ (เปิด/ปิด)
constexpr uint32_t CONTROL_UPDATE_MS = 200;  // ตรวจเงื่อนไขของทั้ง 3 อุปกรณ์ทุกกี่ ms

// ---- พัดลม (ตามอุณหภูมิ เหมือน Lab 6) ----
constexpr int FAN_ON_TEMP_C = 30;         // ค่าเริ่มต้น: อุณหภูมิ "ตั้งแต่นี้ขึ้นไป" พัดลมเริ่มหมุน
constexpr int FAN_OFF_TEMP_C = 28;        // ค่าเริ่มต้น: อุณหภูมิ "ต่ำลงมาถึงนี้" พัดลมหยุด (ต้องต่ำกว่า ON = hysteresis)
constexpr float FAN_FULL_TEMP_C = 36.0f;  // อุณหภูมิที่พัดลมหมุนเต็มที่ (duty 100%)
constexpr int FAN_MIN_DUTY_PERCENT = 30;  // duty ต่ำสุดตอนพัดลมหมุน
constexpr int MANUAL_FAN_PERCENT = 100;   // โหมด MANUAL กด ON แล้วพัดลมหมุนกี่ %
constexpr int SENSOR_FAIL_FAN_PERCENT = 50;  // fail-safe: เซนเซอร์อุณหภูมิเสีย (และไม่ได้ใช้ SIM) ให้พัดลมหมุนกี่ % (ดีกว่าปิดเฉยๆ)
constexpr float THRESHOLD_MIN_C = 15.0f;  // ช่วงที่ปรับเกณฑ์อุณหภูมิพัดลมได้
constexpr float THRESHOLD_MAX_C = 45.0f;

// ---- ไฟปลูก (ตามแสงจริงจาก LDR + ช่วงเวลา) ----
constexpr int LIGHT_THRESHOLD_PERCENT = 40;  // ค่าเริ่มต้น: แสงต่ำกว่านี้ (%) ถึงเปิดไฟ
constexpr int LIGHT_HYSTERESIS_PERCENT = 5;  // แสงต้องสูงกว่าเกณฑ์อีกเท่านี้ ไฟถึงจะดับ (กันไฟกะพริบ)
constexpr int LIGHT_FROM_HOUR = 6;           // ค่าเริ่มต้น: เปิดไฟได้ตั้งแต่กี่โมง (0-23)
constexpr int LIGHT_TO_HOUR = 18;            // ค่าเริ่มต้น: ถึงกี่โมง (1-24)

// ---- ปั๊มน้ำ (ตามตารางเวลา + ความชื้นดิน) ----
constexpr int SOIL_DRY_PERCENT = 35;         // ค่าเริ่มต้น: ดินแห้งกว่านี้ (%) ปั๊มเริ่มทำงาน
constexpr int SOIL_HYSTERESIS_PERCENT = 20;  // รดจนความชื้นสูงถึง (แห้ง + ค่านี้) แล้วหยุด
constexpr int WATER_HOUR_1 = 7;              // ค่าเริ่มต้น: รดน้ำตามเวลารอบที่ 1 (กี่โมง ตรงนาทีที่ 0)
constexpr int WATER_HOUR_2 = 17;             // ค่าเริ่มต้น: รอบที่ 2
constexpr int PUMP_RUN_SEC = 10;             // ค่าเริ่มต้น: รดตามตารางเวลานานกี่วินาที
constexpr int PUMP_RUN_MIN_SEC = 5;          // ช่วงที่ปรับเวลารดได้
constexpr int PUMP_MAX_RUN_SEC = 30;         // ความปลอดภัย: ปั๊มทำงานต่อเนื่องได้ไม่เกินนี้ ไม่ว่าสาเหตุใด (watchdog)
constexpr int PUMP_COOLDOWN_SEC = 20;        // หลังปั๊มหยุดทุกครั้ง พักปั๊มอย่างน้อยกี่วินาที (ห้ามสั่งเริ่มใหม่)
constexpr int WATER_SLOT_GRACE_MIN = 5;      // ถึงเวลารดแล้ว ถ้าเลยมาเกินกี่นาทียังไม่ได้เริ่ม ถือว่าพลาดรอบนั้น

// ---- ดินชื้น (จำลอง: ไม่มีเซนเซอร์จริง ดินแห้งเองตามเวลา และเปียกขึ้นเมื่อปั๊มทำงาน) ----
constexpr uint32_t SOIL_UPDATE_MS = 1000;       // คำนวณความชื้นดินทุกกี่ ms
constexpr float SOIL_START_PERCENT = 60.0f;     // ความชื้นดินตอนเปิดเครื่อง
constexpr float SOIL_DRY_RATE = 0.2f;           // ดินแห้งลงกี่ % ต่อวินาที (ถ้าเอาไปใช้จริงต้องช้ากว่านี้มาก แต่ที่นี่ต้องการให้เห็นผลเร็ว)
constexpr float SOIL_WET_RATE = 1.5f;           // ดินชื้นขึ้นกี่ % ต่อวินาทีขณะปั๊มทำงาน
constexpr float SOIL_TEST_DRY_PERCENT = 20.0f;  // แตะการ์ด SOIL = ทำให้ดินแห้งเท่านี้ (ไว้ทดสอบ)

// ---- เซนเซอร์แสง LDR (จาก Lab 2: GPIO34, ค่า raw แปรผกผันกับแสง) ----
constexpr uint32_t LDR_READ_MS = 100;       // อ่าน LDR ทุกกี่ ms
constexpr uint8_t LDR_SAMPLES = 8;          // อ่านกี่ครั้งต่อรอบแล้วเฉลี่ย (oversampling)
constexpr float LDR_SMOOTH_ALPHA = 0.25f;   // ตัวกรอง EMA (0.0-1.0 ยิ่งน้อยยิ่งนิ่ง)
constexpr int LDR_RAW_BRIGHT = 0;           // ค่า raw ที่ถือว่า "สว่างเต็มที่" (ใช้ ADC 0 dB เหมือน Lab 2)
constexpr int LDR_RAW_DARK = 300;           // ค่า raw ที่ถือว่า "มืดสุด" (ค่าเริ่มต้นประมาณ ปรับให้ตรงห้องของคุณด้วยปุ่ม SET DARK / SET BRIGHT ในหน้า SYSTEM)
constexpr int LDR_MIN_SPAN = 20;            // ช่วง raw ระหว่าง "มืด" กับ "สว่าง" ต้องห่างกันอย่างน้อยเท่านี้ ไม่งั้นไม่รับค่า

// ---- MQTT (ส่งข้อมูล/รับคำสั่งผ่าน broker) ----
// ค่าเริ่มต้นเป็น broker สาธารณะของ HiveMQ ไม่มี user/password และไม่เข้ารหัส: ใครก็อ่านและส่งข้อความเข้า topic ได้
// ใช้เพื่อเรียนรู้เท่านั้น ของจริงต้องใช้ broker ที่มี user/password และ TLS (พอร์ต 8883)
inline constexpr char MQTT_HOST[] = "broker.hivemq.com";
constexpr uint16_t MQTT_PORT = 1883;
inline constexpr char MQTT_TOPIC_PREFIX[] = "cydfarm";  // topic หลัก = <prefix>/<รหัส 6 หลักจาก MAC ของบอร์ด>
constexpr uint32_t MQTT_TELEMETRY_MS = 10000;      // ส่งค่าเซนเซอร์/สถานะทุกกี่ ms
constexpr uint32_t MQTT_RECONNECT_MIN_MS = 5000;   // เชื่อมต่อไม่สำเร็จ รอกี่ ms ก่อนลองใหม่ (เพิ่มเป็นสองเท่าทุกครั้ง)
constexpr uint32_t MQTT_RECONNECT_MAX_MS = 60000;  // รอนานสุดกี่ ms
constexpr uint16_t MQTT_SOCKET_TIMEOUT_S = 2;      // รอ broker ตอบนานสุดกี่วินาที (สั้น เพื่อไม่ให้โปรแกรมค้างนาน)
constexpr uint16_t MQTT_KEEPALIVE_S = 30;          // ส่ง ping ให้ broker ทุกกี่วินาที
constexpr uint32_t REMOTE_ENABLE_MS = 600000;      // เปิดรับคำสั่งทาง MQTT ได้กี่ ms แล้วปิดเอง (600000 = 10 นาที)

// ---- PWM (LEDC) ----
constexpr uint8_t PWM_RESOLUTION_BITS = 8;  // 8 บิต = ค่า 0-255
constexpr uint32_t PWM_MAX_VALUE = (1UL << PWM_RESOLUTION_BITS) - 1;
constexpr uint32_t PWM_FREQUENCY_HZ = 5000;
constexpr uint8_t FAN_CHANNEL = 0;          // ช่อง LEDC ที่คุม LED น้ำเงิน (GPIO17)

// ---- ค่าจำลอง (SIM) ไว้ใช้เมื่อไม่ได้ต่อเซนเซอร์ หรืออยากทดสอบโดยไม่ต้องรอความร้อนจริง ----
// แตะการ์ด TEMP / HUM เพื่อสลับ: REAL (เซนเซอร์จริง) -> SIM random (สุ่ม) -> SIM sweep (ไล่ขึ้น-ลงสม่ำเสมอ) -> REAL ...
// ข้อควรระวัง: ตัวควบคุมจริงไม่ควร "แอบ" ใช้ค่าปลอมเมื่อเซนเซอร์เสีย Lab นี้จึงปิดค่านี้ไว้
// (เซนเซอร์เสีย = ขึ้น ERR และพัดลมเข้าโหมด fail-safe) ถ้าอยากให้เป็นแบบ Lab 6 ให้เปลี่ยนเป็น true
constexpr bool SIM_WHEN_NO_SENSOR = false;
constexpr uint32_t SIM_UPDATE_MS = 1000;    // สุ่มค่าใหม่ทุกกี่ ms
constexpr float SIM_TEMP_MIN_C = 24.0f;     // ขอบเขตอุณหภูมิจำลอง
constexpr float SIM_TEMP_MAX_C = 38.0f;
constexpr float SIM_TEMP_START_C = 28.0f;   // อุณหภูมิเริ่มต้นของ SIM random
constexpr float SIM_TEMP_STEP_C = 1.5f;     // SIM random: แต่ละครั้งอุณหภูมิเปลี่ยนสุ่มไม่เกิน +-นี้
constexpr float SIM_HUM_MIN = 40.0f;        // ขอบเขตความชื้นจำลอง (%)
constexpr float SIM_HUM_MAX = 90.0f;
constexpr float SIM_HUM_START = 65.0f;
constexpr float SIM_HUM_STEP = 3.0f;        // SIM random: แต่ละครั้งความชื้นเปลี่ยนสุ่มไม่เกิน +-นี้
constexpr uint32_t SIM_PERIOD_MS = 60000;   // SIM sweep (และแสงจำลอง): ขึ้นแล้วลง 1 รอบใช้เวลากี่ ms

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

// ---- จอสัมผัส XPT2046 (อ่านค่าผ่าน SPI) เหมือน Lab 9 ----
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

// ---- ความสว่างจอ (PWM ที่ GPIO21 เหมือน Lab 5) ----
constexpr uint8_t BACKLIGHT_CHANNEL = 2;     // ช่อง LEDC ของไฟหลังจอ (ช่อง 0 ใช้กับพัดลม)
constexpr int BACKLIGHT_MIN_PERCENT = 10;    // ต่ำสุดที่ให้ตั้งได้ (ต่ำกว่านี้จอมืดจนแตะปุ่มไม่เห็น)
constexpr int BACKLIGHT_DEFAULT_PERCENT = 100;

// ---- จอและจับเวลา ----
constexpr uint32_t DISPLAY_REFRESH_MS = 100;  // วาดจอส่วนค่า/อุปกรณ์ใหม่ได้ไม่เกินทุกกี่ ms (วาดเมื่อค่าเปลี่ยนเท่านั้น)
constexpr uint32_t STATUS_REFRESH_MS = 1000;  // อัปเดตบรรทัดสถานะท้ายจอทุกกี่ ms
constexpr uint32_t HEARTBEAT_MS = 500;
constexpr uint32_t SERIAL_PRINT_MS = 1000;
constexpr uint32_t BUTTON_DEBOUNCE_MS = 40;

}  // namespace app
