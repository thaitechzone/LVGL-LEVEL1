# Level 1 — เริ่มต้นกับ ESP32 CYD

โฟลเดอร์นี้เก็บโปรเจกต์ของ Level 1 โดยแยกเป็น Lab ละโฟลเดอร์ แต่ละ Lab เป็น **โปรเจกต์ PlatformIO อิสระ**
เปิดด้วย VS Code (File → Open Folder) ที่โฟลเดอร์ของ Lab นั้นได้เลย ไม่ต้องพึ่งไฟล์นอกโฟลเดอร์

## รายการ Lab

| Lab | โฟลเดอร์ | เนื้อหา |
|---|---|---|
| 1 | [Lab1_Template_Demo](Lab1_Template_Demo/README.md) | Template โปรเจกต์แรก + Hello World ทาง Serial + RGB LED |
| 2 | [Lab2_Template_ADC_LDR](Lab2_Template_ADC_LDR/README.md) | อ่านค่าแสงจาก LDR (ADC) แสดง Raw Data ใน Serial และใช้ควบคุมความเร็ว RGB LED (มืด = เร็ว, สว่าง = ช้า) |
| 3 | [Lab3_WiFiConnecting](Lab3_WiFiConnecting/README.md) | เชื่อมต่อ WiFi ด้วย SSID/รหัสผ่านที่กำหนดเอง (secrets.h) แสดงผลทาง Serial และใช้ไฟ R-G-B บอกสถานะ (สแกน/เชื่อมต่อ/สำเร็จ/ล้มเหลว/หลุด) |
| 4 | [Lab4_TFT_Display](Lab4_TFT_Display/README.md) | ใช้จอ TFT 320x240 แสดงค่าแสง (ตัวเลข+แถบกราฟ), สถานะ WiFi/IP/สัญญาณ และ uptime โดยวาดเฉพาะส่วนที่เปลี่ยน ไม่กะพริบ |
| 5 | [Lab5_PWM](Lab5_PWM/README.md) | PWM ปรับความสว่างไฟหลังจอและ LED ด้วย duty cycle แสดงค่า/รูปคลื่นบนจอและ Serial มีโหมด Manual / Breathe / Auto (ตามแสง LDR) |
| 6 | [Lab6_DHT11_SmartFan](Lab6_DHT11_SmartFan/README.md) | ต่อเซนเซอร์ DHT11 (GPIO27) อ่านอุณหภูมิ/ความชื้น แสดงบนจอและ Serial แล้วสั่งพัดลมจำลอง (LED น้ำเงิน PWM) ด้วยเกณฑ์ ON/OFF แบบมี hysteresis และรับมือกรณีเซนเซอร์เสีย |
| 7 | [Lab7_NTP_Clock](Lab7_NTP_Clock/README.md) | ต่อ WiFi แล้วขอเวลาจริงจาก NTP (UTC+7) แสดงนาฬิกาบนจอ พร้อมสถานะ SYNCED/SYNCING/NO SYNC และยังแสดงค่า DHT11 กับพัดลมจาก Lab 6 เตรียมเวลาจริงไว้ติด log ใน Lab ถัดไป (SD card) |
| 8 | [Lab8_SD_Logger](Lab8_SD_Logger/README.md) | บันทึกอุณหภูมิ/ความชื้น (REAL/SIM) และสถานะพัดลมลงไฟล์ CSV บน microSD card ทุก 10 วินาที พร้อมเวลาจริงจาก NTP ไฟล์ใหม่ทุกครั้งที่เปิดเครื่อง คำสั่งถอดการ์ดอย่างปลอดภัย และรับมือกรณีไม่มีการ์ด/ไม่มี WiFi |
| 9 | [Lab9_Touch_Panel](Lab9_Touch_Panel/README.md) | จอสัมผัส XPT2046: calibrate แตะ 4 มุม (เก็บค่าถาวร) แล้วควบคุมพัดลม ค่า REAL/SIM เวลา NTP และความสว่างจอ (PWM) ผ่าน UI 3 แท็บ มีปุ่ม แถบเลื่อน กดค้าง และหน้า Touch Test (ยังไม่รวม SD card เพราะใช้ SPI ชุดเดียวกัน) |
| 10 | [Lab10_Smart_Farm](Lab10_Smart_Farm/README.md) | ตัวควบคุมโรงเรือนจำลอง: รวมเซนเซอร์ DHT11 / LDR / ดินชื้น (จำลอง) สั่งพัดลม (PWM) ไฟปลูก และปั๊มน้ำ (LED แทนอุปกรณ์) ด้วย hysteresis ตารางเวลา NTP โหมด AUTO/MANUAL ปุ่มหยุดฉุกเฉิน และกลไกความปลอดภัย (watchdog, cooldown, fail-safe) ปรับเกณฑ์จากจอสัมผัสและเก็บถาวร |
| 11 | [Lab11_MQTT_Farm](Lab11_MQTT_Farm/README.md) | ต่อยอด Lab 10 ด้วย MQTT (broker สาธารณะ HiveMQ ไม่มีรหัสผ่าน): ส่ง telemetry/status (retained + Last Will)/event ขึ้น broker และรับคำสั่ง fan/light/pump/estop กลับ โดยเปิดรับคำสั่งได้จากปุ่ม REMOTE บนจอเท่านั้น (หมดอายุเอง 10 นาที) พร้อมวัดเวลาที่การเชื่อมต่อทำให้โปรแกรมหยุดรอ และ exponential backoff |

ใบงานฉบับละเอียดของ Lab 1: [WORKSHEET.md](Lab1_Template_Demo/WORKSHEET.md) (เฉลยผู้สอน: [WORKSHEET_ANSWERS.md](Lab1_Template_Demo/WORKSHEET_ANSWERS.md))

ใบงานฉบับละเอียดของ Lab 2: [WORKSHEET.md](Lab2_Template_ADC_LDR/WORKSHEET.md) (เฉลยผู้สอน: [WORKSHEET_ANSWERS.md](Lab2_Template_ADC_LDR/WORKSHEET_ANSWERS.md))

ใบงานฉบับละเอียดของ Lab 3: [WORKSHEET.md](Lab3_WiFiConnecting/WORKSHEET.md) (เฉลยผู้สอน: [WORKSHEET_ANSWERS.md](Lab3_WiFiConnecting/WORKSHEET_ANSWERS.md))

ใบงานฉบับละเอียดของ Lab 4: [WORKSHEET.md](Lab4_TFT_Display/WORKSHEET.md) (เฉลยผู้สอน: [WORKSHEET_ANSWERS.md](Lab4_TFT_Display/WORKSHEET_ANSWERS.md))

ใบงานฉบับละเอียดของ Lab 5: [WORKSHEET.md](Lab5_PWM/WORKSHEET.md) (เฉลยผู้สอน: [WORKSHEET_ANSWERS.md](Lab5_PWM/WORKSHEET_ANSWERS.md))

ใบงานฉบับละเอียดของ Lab 6: [WORKSHEET.md](Lab6_DHT11_SmartFan/WORKSHEET.md) (เฉลยผู้สอน: [WORKSHEET_ANSWERS.md](Lab6_DHT11_SmartFan/WORKSHEET_ANSWERS.md))

ใบงานฉบับละเอียดของ Lab 7: [WORKSHEET.md](Lab7_NTP_Clock/WORKSHEET.md) (เฉลยผู้สอน: [WORKSHEET_ANSWERS.md](Lab7_NTP_Clock/WORKSHEET_ANSWERS.md))

ใบงานฉบับละเอียดของ Lab 8: [WORKSHEET.md](Lab8_SD_Logger/WORKSHEET.md) (เฉลยผู้สอน: [WORKSHEET_ANSWERS.md](Lab8_SD_Logger/WORKSHEET_ANSWERS.md))

ใบงานฉบับละเอียดของ Lab 9: [WORKSHEET.md](Lab9_Touch_Panel/WORKSHEET.md) (เฉลยผู้สอน: [WORKSHEET_ANSWERS.md](Lab9_Touch_Panel/WORKSHEET_ANSWERS.md))

ใบงานฉบับละเอียดของ Lab 10: [WORKSHEET.md](Lab10_Smart_Farm/WORKSHEET.md) (เฉลยผู้สอน: [WORKSHEET_ANSWERS.md](Lab10_Smart_Farm/WORKSHEET_ANSWERS.md))

ใบงานฉบับละเอียดของ Lab 11: [WORKSHEET.md](Lab11_MQTT_Farm/WORKSHEET.md) (เฉลยผู้สอน: [WORKSHEET_ANSWERS.md](Lab11_MQTT_Farm/WORKSHEET_ANSWERS.md))

Lab ต่อไปให้คัดลอก `Lab1_Template_Demo` เป็นต้นแบบ (วิธีทำอยู่ใน README ของ Lab 1 หัวข้อ "ใช้เป็น Template เริ่ม Lab ใหม่")
แล้วตั้งชื่อตามรูปแบบ `LabN_ชื่อ_Lab` เช่น `Lab2_My_First_Screen`

## เอกสารที่เกี่ยวข้อง

- ใบงานภาษาไทยทีละขั้นของ Level 1 (จอ TFT, touch, LVGL, capstone): [`docs/level1/`](../docs/level1/README.md)
- ข้อมูล pin และข้อควรระวังของบอร์ด: `ESP32-2432S028_CYD_Knowledge_Base.md` ที่โฟลเดอร์หลัก

> หมายเหตุ: เลข Lab ในโฟลเดอร์นี้นับแยกจากโฟลเดอร์ `labs/` ที่เก็บ Lab ชุดเดิมของคอร์ส
> (เช่น `labs/Lab1_Touch_Calibration` กับ `Level1/Lab1_Template_Demo` เป็นคนละ Lab)
