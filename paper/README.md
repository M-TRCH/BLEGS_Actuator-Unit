# Paper — การพัฒนาต้นแบบหุ่นยนต์สี่ขารูปแบบสององศาอิสระภายใต้แรงบิดต่อน้ำหนักที่เหมาะสม

ต้นฉบับบทความวิจัยสำหรับ **The Journal of KMUTNB** (วารสารวิชาการพระจอมเกล้าพระนครเหนือ, TCI Tier 1)
ส่วนนี้เป็นชั้น "เปเปอร์" ของ monorepo BLEGS — โค้ดที่ผลิตข้อมูลทั้งหมดอยู่ใน [`../python/`](../python/)
และเฟิร์มแวร์อยู่ใน [`../firmware/`](../firmware/) จึงไม่มีสำเนาโค้ดในโฟลเดอร์นี้อีกต่อไป

## งานวิจัยโดยย่อ

หุ่นยนต์สี่ขา BLEGS (Bio-inspired LEGged System) ขาละสององศาอิสระด้วยกลไก Separated Five-Bar Linkage
ขับด้วยมอเตอร์ BLDC (FOC/SVPWM) + เอนโคดเดอร์สัมบูรณ์ AS5047P อัตราทด 8:1 — ใช้แนวทางเชิงข้อมูล
(วัดตำแหน่งจริงด้วย ArUco vision) สร้างแบบจำลองชดเชยความคลาดเคลื่อนเชิงกล เพื่อให้ได้ความแม่นยำสูง
โดยไม่ต้องเพิ่มเซนเซอร์หรือชิ้นส่วนกลไก

การทดลอง 3 ชุด: (1) แรงบิดต่อน้ำหนัก (2) ความแม่นยำปลายขา + แบบจำลองชดเชย ML
(3) การเดินทั้งตัว — เทียบเปิด/ปิดการชดเชย และหาขีดจำกัดบนพื้นผิวจริง

## โครงสร้าง

```
paper/
├── manuscript/
│   ├── drafts/            ต้นฉบับ (Markdown) 00_frontmatter → 06_references
│   └── submission/        ไฟล์ .docx ที่ส่งจริง + SUBMISSION_CHECKLIST.md
├── figures/  src/ export/  ต้นฉบับรูปที่แก้ได้ / .jpg สำหรับส่ง (วารสารบังคับ)
├── tables/   src/ export/  ตารางก็ต้องส่งเป็น .jpg เช่นกัน
├── data/     real/ vision/ ข้อมูลที่ใช้สร้างรูปและตาราง
├── references/            ไฟล์อ้างอิง / .bib / PDF ที่อ้างถึง
├── journal/               ข้อกำหนดวารสาร เทมเพลต และ reference.docx ของ pandoc
└── build.ps1              แปลง drafts → manuscript.docx
```

## วิธีทำงาน

1. เขียน/แก้ต้นฉบับใน `manuscript/drafts/` (ธรรมเนียมการเขียนดู [`CLAUDE.md`](CLAUDE.md))
2. รูปและตาราง: ไฟล์ต้นฉบับไว้ `*/src/` — ส่งออก .jpg ไว้ `*/export/`
3. แปลงเป็น .docx: `powershell -File paper/build.ps1` (ต้องมี pandoc และ `journal/templates/reference.docx` — ดู [`journal/README.md`](journal/README.md))
4. เก็บงานขั้นสุดท้ายใน Word ตาม [`manuscript/submission/SUBMISSION_CHECKLIST.md`](manuscript/submission/SUBMISSION_CHECKLIST.md) แล้วส่งผ่าน ThaiJO

## สถานะและแผน

ดู [`action-plan.md`](action-plan.md) — สรุป: บทที่ 2 (วิธีการ) เขียนครบแล้ว 26 สมการ,
บทอื่นยังไม่เริ่ม, การทดลอง 2.3.3 ออกแบบแล้วรอทดสอบจริง
