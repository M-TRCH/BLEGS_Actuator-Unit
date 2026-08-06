# journal/ — ข้อกำหนดของวารสาร

วารสารเป้าหมาย: **The Journal of KMUTNB** (วารสารวิชาการพระจอมเกล้าพระนครเหนือ, TCI Tier 1)
ส่งผ่านระบบ ThaiJO: https://ph01.tci-thaijo.org/index.php/kmutnb-journal/index

| ไฟล์ | คือ | ต้นฉบับ |
|---|---|---|
| `FORMAT_GUIDE.md` | สรุปข้อกำหนดรูปแบบ (แก้ให้ตรงเอกสารทางการแล้ว) | สรุปเอง |
| `instructions/manuscript-preparation-guide.pdf` | คำแนะนำเตรียมต้นฉบับฉบับเต็ม ไทย+อังกฤษ | `19 คำแนะในการเตรียมต้นฉบับ-หน้าสุดท้าย.pdf` |
| `instructions/manuscript-preparation-guide.docx` | ฉบับ .docx ของคำแนะนำ | `คำแนะนำเตรียมต้นฉบับบทความ(ใหม่).docx` |
| `instructions/thaijo-submission-guide.pdf` | ขั้นตอนสมัครและส่งบทความใน ThaiJO | `การสมัครสมาชิกและส่งบทความในระบบ Thaijo.pdf` |
| `instructions/citation-format.pdf` | รูปแบบการอ้างอิง | `รูปแบบการอ้างอิง.pdf` |
| `templates/kmutnb-article-template.docx` | เทมเพลตบทความของวารสาร (โครงเอกสารจริง) | `template_บทความ (กรุณาดาวน์โหลดไฟล์).docx` |
| `templates/examples-references-2026.docx/.pdf` | ตัวอย่างการอ้างอิง IEEE ทุกประเภท | `Examples References (New2026)` |
| `templates/example-manuscript.pdf` | ตัวอย่างบทความที่จัดรูปแบบแล้ว | `Example Template.pdf` |
| `templates/reference.docx` | **สไตล์สำหรับ pandoc** ใช้โดย `paper/build.ps1` | สร้างเอง (ดูด้านล่าง) |

ไฟล์ต้นฉบับชุดเต็มอยู่ที่ `C:\Users\mteer\OneDrive\Education\Documents\Thesis\Paper\Templates\`

## reference.docx

ไฟล์สไตล์ที่ pandoc ใช้จัดรูปแบบ .docx — **ห้ามใช้ `kmutnb-article-template.docx` แทน** เพราะ pandoc
ต้องการชุดชื่อสไตล์ของตัวเอง (Body Text, Heading 1–6, Title, Image Caption ฯลฯ)

สร้างใหม่เมื่อจำเป็น:

```powershell
pandoc --print-default-data-file reference.docx > paper/journal/templates/reference.docx
```

แล้วปรับใน Word (หรือสคริปต์) ให้ตรงข้อกำหนด:

1. หน้ากระดาษ: ขอบบน-ล่าง 3 ซม. ซ้าย-ขวา 2.5 ซม.
2. ฟอนต์ทุกสไตล์ → TH Sarabun New (ชื่อเรื่อง 18 หนา, หัวข้อ 14 หนา, เนื้อความ 14, คำบรรยาย 12)
3. **สำคัญกับภาษาไทย:** ต้องตั้งฟอนต์ช่อง Complex scripts (`w:cs`) และขนาด `w:szCs` ด้วย
   ไม่ใช่แค่ช่อง Latin — ไม่งั้นข้อความไทยจะหลุดไปใช้ฟอนต์ดีฟอลต์ทั้งเอกสารโดยไม่มีคำเตือน
4. ไม่ต้องตั้ง 2 คอลัมน์ใน reference.docx — front matter เป็นคอลัมน์เดียว จึงต้องแบ่ง section
   ใน Word ขั้นตอนเก็บงานท้ายสุดอยู่ดี
