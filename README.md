# BLEGS — Bio-inspired LEGged System

ต้นแบบหุ่นยนต์สี่ขา ขาละสององศาอิสระด้วยกลไก **Separated Five-Bar Linkage** ขับด้วยมอเตอร์ BLDC
(FOC/SVPWM) + เอนโคดเดอร์สัมบูรณ์ AS5047P อัตราทด 8:1 พร้อมแบบจำลองชดเชยความคลาดเคลื่อนเชิงกล
แบบ data-driven — รีโปนี้คือศูนย์รวมทั้งโปรเจกต์: เฟิร์มแวร์ โค้ดควบคุมฝั่งโฮสต์ และต้นฉบับบทความวิจัย

> เดิมชื่อ `BLEGS_Actuator-Unit` (GitHub redirect ชื่อเก่ามาที่นี่อัตโนมัติ)
> รีโปเปเปอร์เดิม `BLEGS_Paper-TCI1` ถูกรวมเข้ามาที่ `paper/` พร้อมประวัติ และ archive แล้ว

## โครงสร้าง

| ส่วน | คือ | เริ่มอ่านที่ |
|---|---|---|
| [`firmware/`](firmware/) | เฟิร์มแวร์ STM32G431CB ของ Actuator Unit (1 บอร์ด = 1 มอเตอร์, ทั้งหุ่นใช้ 8 ชุด) | [`firmware/include/system.h`](firmware/include/system.h) (pin ground truth) |
| [`python/`](python/) | โค้ดควบคุมฝั่ง PC: gait, IK ขา five-bar, vision ground truth, สคริปต์ทดลองของเปเปอร์ | [`python/README.md`](python/README.md) |
| [`paper/`](paper/) | ต้นฉบับบทความ TCI Tier 1 (The Journal of KMUTNB) + ข้อกำหนดวารสาร + pipeline แปลง .docx | [`paper/README.md`](paper/README.md) |
| [`docs/`](docs/) | เอกสารเทคนิค/คู่มือ/roadmap (ไทย-อังกฤษ) และทฤษฎี LaTeX | [`docs/README.md`](docs/README.md) |
| [`logs/`](logs/) | ข้อมูลเดินจริงที่ใช้ในงานวิเคราะห์ (commit ไว้โดยตั้งใจ) | — |

## Quick start

**เฟิร์มแวร์** — PlatformIO (ini อยู่ที่ราก จึงสั่งจากรากได้เลย; ถ้า `pio` ไม่อยู่บน PATH
ใช้เทอร์มินัลของ PlatformIO extension หรือ `%USERPROFILE%\.platformio\penv\Scripts\pio.exe`):

```powershell
pio run              # build
pio run -t upload    # flash ผ่าน ST-Link
pio device monitor   # serial monitor @ 921600
```

**ฝั่ง Python** (Windows; ดู dependencies ใน `python/README.md`):

```powershell
python python/paper_testing/test_3/walk_test.py    # การทดสอบเดินปัจจุบัน: เมนู [7]/[8]/[9] และโหมด RC [R]
```

ไม่ต่อมอเตอร์ก็รันได้ — สคริปต์เข้าสู่ SIMULATION MODE อัตโนมัติเมื่อหามอเตอร์ไม่พบ

**เปเปอร์** — เขียนต้นฉบับใน `paper/manuscript/drafts/` แล้วแปลงเป็น .docx:

```powershell
powershell -File paper/build.ps1
```

## คำเตือนด้านความปลอดภัย

Emergency stop ในซอฟต์แวร์**ยังไม่หยุดมอเตอร์จริง** (ตัวจัดการฝั่งเฟิร์มแวร์ถูกปิดไว้ ดูรายละเอียดใน
[`CLAUDE.md`](CLAUDE.md) หัวข้อ Hardware & safety facts) — การหยุดฉุกเฉินที่เชื่อถือได้มีทางเดียวคือตัดไฟมอเตอร์

## สำหรับผู้พัฒนา (รวมถึง Claude Code)

ข้อเท็จจริงเชิงสถาปัตยกรรม โปรโตคอล ค่าคงที่ที่ diverge กันโดยตั้งใจ และกับดักต่างๆ ถูกรวบรวมไว้ที่
[`CLAUDE.md`](CLAUDE.md) — อ่านก่อนแก้โค้ด โดยเฉพาะก่อนแตะโปรโตคอลหรือสคริปต์ควบคุม
