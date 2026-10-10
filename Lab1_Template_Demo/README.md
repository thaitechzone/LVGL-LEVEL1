# Level 1 / Lab 1 — Template Demo (โปรเจกต์แรกของบอร์ด ESP32 CYD)

Template โปรเจกต์ PlatformIO สำหรับบอร์ด **ESP32-2432S028 (CYD)** ที่รวมไฟล์ตั้งค่าสำคัญของทั้งคอร์สไว้ให้แล้ว
และมีโปรแกรมตัวอย่างง่ายๆ ที่พิสูจน์ว่าเครื่องมือและบอร์ดทำงานครบ:

1. พิมพ์ **Hello World** ออกทาง Serial Monitor
2. ให้ **RGB LED** บนบอร์ดแสดงสี แดง → เขียว → น้ำเงิน วนไปเรื่อยๆ พร้อมพิมพ์ชื่อสีและเลข GPIO ใน Serial ให้ตรงกับสีที่เห็น

> **ใบงานฉบับละเอียด (อธิบายทฤษฎี อ่านโค้ดทีละบรรทัด แบบฝึกหัด และเฉลย) อยู่ที่ [WORKSHEET.md](WORKSHEET.md)**
> เฉลยสำหรับผู้สอน: [WORKSHEET_ANSWERS.md](WORKSHEET_ANSWERS.md)

## เป้าหมาย

- เปิดโปรเจกต์ด้วย VS Code + PlatformIO, build, upload และเปิด Serial Monitor ได้
- เข้าใจว่าไฟล์ตั้งค่าแต่ละไฟล์ทำหน้าที่อะไร และต้องแก้ไฟล์ไหน
- ใช้โฟลเดอร์นี้เป็นต้นแบบเริ่ม Lab ถัดไปได้ (คัดลอก → เปลี่ยนชื่อ → แก้ `src/main.cpp`)

## โครงสร้างโปรเจกต์

```
Lab1_Template_Demo/
├─ platformio.ini        ค่าตั้งของ PlatformIO: บอร์ด, baud, flag ของจอ TFT/LVGL, ไลบรารี (comment ไว้)
├─ include/
│  ├─ board_config.h     ทุก pin + ข้อมูลจอ/touch + ฟังก์ชันช่วย LED  (ฮาร์ดแวร์ ห้ามแก้ถ้าไม่จำเป็น)
│  ├─ app_config.h       ค่าที่ปรับได้ของ Lab นี้ เช่น ความเร็ว LED, ข้อความ  (แก้ได้เลย)
│  ├─ lv_conf.h          ค่าตั้งของ LVGL (Lab นี้ยังไม่ใช้ แต่เตรียมไว้ให้ Lab ที่มีจอ)
│  └─ secrets.h.example  แม่แบบรหัส WiFi (Level 2) คัดลอกเป็น secrets.h ซึ่งถูก gitignore
├─ src/
│  └─ main.cpp           โปรแกรม Hello World + RGB LED
├─ .vscode/extensions.json  แนะนำ extension PlatformIO ให้ VS Code
└─ .gitignore            ไม่เก็บ .pio, ไฟล์ build และ secrets.h
```

| ไฟล์ | แก้เมื่อไร |
|---|---|
| `board_config.h` | แทบไม่แก้ (ยกเว้นค่า calibration ของ touch ที่ Lab touch, หรือ `LED_ACTIVE_LOW` ถ้า LED กลับด้าน) |
| `app_config.h` | แก้บ่อย ใช้ลองเปลี่ยนค่าต่างๆ ของแต่ละ Lab |
| `platformio.ini` | เปิด `lib_deps` เมื่อเริ่มใช้จอ/touch/LVGL |
| `lv_conf.h` | ปรับเมื่อต้องเปิด widget/ฟอนต์เพิ่ม |

## วิธีเปิดโปรเจกต์

1. เปิด VS Code → File → **Open Folder** → เลือกโฟลเดอร์ `Level1/Lab1_Template_Demo`
   (ต้องเปิดที่โฟลเดอร์ที่มี `platformio.ini` อยู่ข้างใน ไม่ใช่โฟลเดอร์ระดับบน)
2. รอ PlatformIO โหลดโปรเจกต์ (ครั้งแรกอาจใช้เวลาหลายนาที)
3. เสียบบอร์ดด้วยสาย USB ที่ส่งข้อมูลได้

## Build / Upload / Serial Monitor

```powershell
pio run -t upload
pio device monitor -b 115200
```

- ถ้าเครื่องมีหลายพอร์ต ให้ระบุ `--upload-port COMx` เช่น `pio run -t upload --upload-port COM3`
- ถ้าไม่รู้จักคำสั่ง `pio` ให้ใช้ `python -m platformio ...` แทน
- **ปิด Serial Monitor (`Ctrl+C`) ก่อน upload ทุกครั้ง** ไม่เช่นนั้นพอร์ตจะถูกจอง

## ผลที่ควรเห็น

Serial Monitor (ตัวอย่างจริงจากบอร์ดต้นแบบ ชิป/หน่วยความจำของคุณอาจต่างเล็กน้อย)

```
==========================================
Hello World from ESP32-2432S028 (CYD)!
Level 1 / Lab 1 : Template Demo
Chip : ESP32-D0WD-V3 rev 3, CPU 240 MHz
Flash: 4096 KB, free heap: 350728 bytes
==========================================
Hello World #1  LED = RED (GPIO4)
Hello World #2  LED = GREEN (GPIO16)
Hello World #3  LED = BLUE (GPIO17)
Hello World #4  LED = RED (GPIO4)
...
```

RGB LED เปลี่ยนสีทุก `LED_STEP_MS` (ค่าในไฟล์ตอนนี้ = 2000 ms หรือ 2 วินาที) ตามลำดับนี้ แล้ววนกลับ **สีที่เห็นต้องตรงกับชื่อใน Serial เสมอ**

| ลำดับ | ข้อความใน Serial | สีที่ต้องเห็นบนบอร์ด |
|---|---|---|
| 1 | `LED = RED` | แดง |
| 2 | `LED = GREEN` | เขียว |
| 3 | `LED = BLUE` | น้ำเงิน |

ถ้าสีที่เห็นไม่ตรงกับชื่อ ดูหัวข้อ "ปัญหาที่พบบ่อย" ด้านล่าง

จอ TFT ยังไม่ถูกเริ่มใช้งานใน Lab นี้ จึงอาจมืดหรือแสดงภาพว่างๆ ซึ่งเป็นเรื่องปกติ (จะใช้จอใน Lab ถัดไป)

## แบบฝึกหัด

1. **เปลี่ยนความเร็ว:** แก้ `LED_STEP_MS` ใน `app_config.h` เป็น `200` แล้ว upload ใหม่
2. **เปลี่ยนข้อความ:** แก้ `HELLO_TEXT` เป็นชื่อของตัวเอง ดูผลใน Serial Monitor
3. **แก้ลำดับสี:** แก้อาร์เรย์ `steps[]` ใน `main.cpp` ให้เป็น น้ำเงิน → เขียว → แดง (สลับลำดับ) แล้วสังเกตว่าการวนซ้ำใช้ `% STEP_COUNT` ปรับตามจำนวนสมาชิกให้เองโดยไม่ต้องแก้ที่อื่น
   ท้าทายต่อ: เพิ่มขั้นสีเหลืองโดยใช้ `{"YELLOW", cyd::LED_PIN_R, true, true, false}`
4. **อ่านโค้ด:** ใน `main.cpp` โปรแกรมไม่ใช้ `delay()` ในการรอแต่ละสี แต่ใช้อะไร และดีกว่าอย่างไร
5. **ท้าทาย:** ทำให้ Serial พิมพ์ `uptime` (วินาทีตั้งแต่เปิดเครื่อง) ต่อท้ายทุกบรรทัด ด้วย `millis() / 1000`

## ใช้เป็น Template เริ่ม Lab ใหม่

1. คัดลอกโฟลเดอร์ `Lab1_Template_Demo` ไปวางข้างๆ แล้วตั้งชื่อใหม่ เช่น `Lab2_My_First_Screen`
2. เปิดโฟลเดอร์ใหม่ด้วย VS Code (Open Folder)
3. แก้ `src/main.cpp` ให้เป็นโปรแกรมของ Lab ใหม่ และปรับ `app_config.h`
4. ถ้าต้องใช้จอ/touch/LVGL ให้เอา `;` หน้า `lib_deps` ใน `platformio.ini` ออก
5. ถ้าต้องใช้ WiFi ให้คัดลอก `include/secrets.h.example` เป็น `include/secrets.h` แล้วกรอกค่า (ไฟล์นี้ไม่ถูกอัปขึ้น git)

> ห้ามแก้ pin ใน `board_config.h` ตามอำเภอใจ และ **ห้ามใช้ GPIO21 ต่อเซนเซอร์** (เป็นขา backlight ของจอ)
> ใช้ GPIO27 แทน (ดูคอมเมนต์ท้ายหมวด expansion ใน `board_config.h`)

## ปัญหาที่พบบ่อย

| อาการ | สาเหตุ / วิธีแก้ |
|---|---|
| ไม่เห็น COM port | เปลี่ยนสาย USB (ต้องส่งข้อมูลได้), ติดตั้งไดรเวอร์ CH340 |
| upload ไม่ผ่าน: port busy | ปิด Serial Monitor, เช็คเลข COM (เปลี่ยนได้เมื่อถอดเสียบสาย) |
| Serial Monitor มีแต่ตัวอักษรเพี้ยน | ตั้ง baud 115200 |
| สีผสมหรือตรงข้ามกับที่ควรเป็น (เช่น ตอน RED เห็นเป็นฟ้า) | บอร์ดเป็นแบบ active-HIGH ให้ตั้ง `LED_ACTIVE_LOW = false` ใน `board_config.h` |
| ติดสีเดียวแต่ผิดชื่อ (เช่น ตอน GREEN เห็นเป็นแดง) | เลข GPIO ของสีนั้นผิด ดูเลข GPIO ใน Serial แล้วสลับ `LED_PIN_R/G/B` ใน `board_config.h` ให้ตรงสีที่เห็นจริง |
| LED ไม่ติดเลย | ตรวจว่าไม่ได้แก้ `LED_PIN_R/G/B` และมองให้ถูกตำแหน่ง (LED RGB ดวงเล็กบนบอร์ด) |
| ไม่เห็นข้อความตอนเปิดเครื่อง | เปิด monitor ช้ากว่าบอร์ดบูต กดปุ่ม EN/RESET บนบอร์ด |

## เช็คลิสต์ผ่าน Lab

- [ ] build และ upload สำเร็จ
- [ ] เห็น `Hello World` ใน Serial Monitor
- [ ] RGB LED แสดงแดง → เขียว → น้ำเงิน และสีตรงกับชื่อใน Serial ทุกขั้น
- [ ] เปลี่ยนความเร็วหรือข้อความด้วย `app_config.h` ได้
- [ ] อธิบายได้ว่า `board_config.h` กับ `app_config.h` ต่างกันอย่างไร
