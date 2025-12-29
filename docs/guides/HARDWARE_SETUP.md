# 🔧 Hardware Setup Guide

คู่มือการติดตั้งและตั้งค่า Hardware สำหรับ BLEGS Actuator Unit

---

## 📋 รายการอุปกรณ์ที่ต้องใช้

### อุปกรณ์หลัก
- [ ] **BLEGS Actuator Unit PCB** (STM32G431CBU6)
- [ ] **BLDC Motor** with encoder connection
- [ ] **AS5047P Magnetic Encoder** (14-bit absolute)
- [ ] **Gear Reducer** (8:1 ratio)
- [ ] **Power Supply** 24V DC, 3A+ recommended

### อุปกรณ์เสริม
- [ ] **USB-to-Serial Adapter** (FTDI, CP2102, หรือ CH340)
  - ต้องรองรับ 921,600 baud
  - แนะนำ FTDI FT232RL
- [ ] **START Button** (momentary push button)
- [ ] **NeoPixel LED** (WS2812B, 1 ดวง)
- [ ] **Capacitors** (power supply filtering)

### เครื่องมือ
- [ ] Multimeter
- [ ] Oscilloscope (optional, for debugging)
- [ ] Soldering iron
- [ ] Wire strippers
- [ ] Screwdrivers

---

## 🔌 Pinout และการเชื่อมต่อ

### STM32G431CBU6 Pinout (สำคัญ)

#### Motor Driver (3-Phase PWM)
```
PB0  → Phase A PWM
PB1  → Phase B PWM
PB13 → Phase C PWM
```

#### Encoder (AS5047P - SPI)
```
PA5  → SPI_SCK  (Encoder Clock)
PA6  → SPI_MISO (Encoder Data Out)
PA7  → SPI_MOSI (Encoder Data In)
PB12 → SPI_CS   (Encoder Chip Select)
```

#### Communication (System Serial)
```
PA9  → System Serial TX (SYS_TX1)
PA10 → System Serial RX (SYS_RX1)
```

**Alternative RS232:**
```
PC4 → RS232 TX
PB7 → RS232 RX
```

#### GPIO
```
PA1  → START Button (active low, internal pull-up)
PA0  → CALC Button (active low, internal pull-up)
PC13 → Status LED (built-in)
```

#### ADC (Current Sensing)
```
PA2 → Phase A Current (SEN_IA)
PA3 → Phase C Current (SEN_IC)
```

---

## ⚡ การเชื่อมต่อ Power Supply

### Main Power (Motor Driver)

```
Power Supply 24V (+) ────┬──→ BLEGS V_MOTOR
                         │
                    [100µF-470µF]
                         │
Power Supply GND (-)  ───┴──→ BLEGS GND
```

**สำคัญ:**
- ใช้ capacitor แรงดันสูง (35V+) ขนาด 100-470µF ใกล้ตัว MOSFET
- เพิ่ม ceramic capacitor 100nF สำหรับ high frequency filtering
- ใช้สาย power ขนาดเหมาะสม (AWG 18-20)

### Logic Power (MCU)

```
24V ───→ [DC-DC Buck Converter] ───→ 3.3V (MCU)
                                 └──→ 5V (Encoder, optional)
```

**หมายเหตุ:**
- MCU ใช้ 3.3V
- AS5047P รองรับทั้ง 3.3V และ 5V
- ตรวจสอบ datasheet ของ encoder ที่ใช้

---

## 🔩 การเชื่อมต่อ Motor และ Encoder

### BLDC Motor Connection

```
Motor Phase A ───→ BLEGS OUT_A
Motor Phase B ───→ BLEGS OUT_B
Motor Phase C ───→ BLEGS OUT_C
```

**การตรวจสอบ:**
1. วัดความต้านทานระหว่าง phase (ควรเท่ากัน)
2. ตรวจสอบไม่มี short circuit ระหว่าง phase
3. ตรวจสอบ isolation จาก motor housing

### AS5047P Encoder (SPI)

```
Encoder VDD  ───→ 3.3V (or 5V)
Encoder GND  ───→ GND
Encoder CLK  ───→ PA5 (SPI1_SCK)
Encoder MISO ───→ PA6 (SPI1_MISO)  
Encoder MOSI ───→ PA7 (SPI1_MOSI)
Encoder CS   ───→ PA4 (SPI1_CS)
```

**สำคัญ:**
- ใช้สายสั้นที่สุด (< 10 cm แนะนำ)
- เพิ่ม pull-up resistor 4.7kΩ ที่ CS (optional)
- ใช้ shielded cable ถ้าระยะไกล

### Encoder Magnet Alignment

```
        Encoder IC
            │
    ┌───────┴───────┐
    │   AS5047P     │
    │               │
    └───────┬───────┘
            │ Gap: 0.5-2.0mm
    ────────┴────────
      Diametric Magnet
      (มุมขั้ว)
```

**การปรับแต่ง:**
1. ระยะห่าง: **0.5-2.0 mm** (optimal ~1.0 mm)
2. ความเข้าแนว: magnet ต้องอยู่กึ่งกลาง sensor
3. ตรวจสอบด้วย `AS5047P_readDiagnostic()` ใน code

---

## 🔘 START Button และ LED

### START Button

```
3.3V ──────────┐
               │
          ┌────┴────┐
          │  Button │ (Normally Open)
          └────┬────┘
               │
               ├─────→ PA1 (with internal pull-up)
               │
              GND
```

**หมายเหตุ:**
- ใช้ internal pull-up ของ STM32
- Button active low (กดแล้วเป็น LOW)

### NeoPixel LED

```
3.3V or 5V ───→ LED VCC
GND        ───→ LED GND
PB0        ───→ LED DIN (Data Input)
```

**สำคัญ:**
- เพิ่ม capacitor 100µF ข้าม VCC-GND
- เพิ่ม resistor 220-470Ω ที่ data line (optional)
- LED ต้องเป็น WS2812B หรือ compatible

---

## 📡 Serial Communication Setup

### USB-to-Serial Adapter

```
Adapter TX  → PA10 (System Serial RX / SYS_RX1)
Adapter RX  → PA9  (System Serial TX / SYS_TX1)
Adapter GND ───→ GND
```

**สำคัญ:**
- **สลับ TX/RX** (adapter TX → MCU RX)
- ไม่ต้องต่อ VCC (MCU มี power แยก)
- ต้องต่อ GND ร่วมกัน

### Adapter Recommendations

| Adapter | Chip | Max Baud | ราคา | คำแนะนำ |
|---------|------|----------|------|---------|
| FTDI FT232RL | FT232 | 3M | $$$ | ✅ Best (stable @ 921600) |
| CP2102 | CP2102 | 1M | $$ | ✅ Good |
| CH340G | CH340 | 2M | $ | ⚠️ OK (may need driver update) |

---

## ⚙️ Hardware Specifications

### STM32G431CBU6

| Parameter | Value |
|-----------|-------|
| CPU | ARM Cortex-M4F @ 170MHz |
| Flash | 128 KB |
| RAM | 32 KB |
| ADC | 12-bit, 5 Msps |
| Timers | Advanced + General purpose |
| Package | UFQFPN48 |

### Motor Driver

| Parameter | Typical | Max |
|-----------|---------|-----|
| Supply Voltage | 24V | 30V |
| Phase Current | 3A | 5A peak |
| PWM Frequency | 10 kHz | 20 kHz |
| Dead Time | 1 µs | - |

### AS5047P Encoder

| Parameter | Value |
|-----------|-------|
| Resolution | 14-bit (16384 counts/rev) |
| Interface | SPI (up to 10 MHz) |
| Update Rate | 28 kHz |
| Position Error | ±0.022° (typ) |
| Supply Voltage | 3.0-3.6V or 4.5-5.5V |

---

## 🧪 การทดสอบ Hardware

### 1. Power Supply Test

```bash
# ตรวจสอบแรงดัน
Multimeter:
- V_MOTOR: 24V ± 1V
- 3.3V Logic: 3.3V ± 0.1V
- 5V (if used): 5.0V ± 0.2V
```

### 2. Encoder Communication Test

```cpp
// ใน Serial Monitor
[INIT] Checking AS5047P...
[OK] Encoder detected
[OK] Angle: 123.45°
```

ถ้าไม่เจอ encoder:
- ตรวจสอบ SPI connections
- ตรวจสอบ VCC และ GND
- ลดความเร็ว SPI clock

### 3. Motor Phase Resistance

```
วัดความต้านทาน DC ระหว่าง phases:
Phase A-B: 1-10 Ω (typical)
Phase B-C: 1-10 Ω (should be equal)
Phase C-A: 1-10 Ω (should be equal)
```

### 4. LED Test

```cpp
// LED ควรแสดงสี:
INIT       → Yellow (เหลือง)
READY      → Green (เขียว)
RUNNING    → Blue (น้ำเงิน)
```

---

## ⚠️ ข้อควรระวัง

### Safety Precautions

1. **ตัดไฟก่อนเชื่อมต่อ** อุปกรณ์ทุกครั้ง
2. **ตรวจสอบขั้วไฟ** ก่อนเปิดเครื่อง
3. **ใช้ fuse** หรือ current limiter
4. **ระวังมอเตอร์หมุนเร็ว** ในโหมดทดสอบ
5. **ใช้แว่นตา** ป้องกันเมื่อทดสอบ

### Common Mistakes

❌ **ต่อ TX-TX, RX-RX** (ต้องสลับกัน)  
❌ **ไม่ต่อ GND ร่วมกัน** ระหว่าง modules  
❌ **ใช้ power supply กำลังไม่พอ**  
❌ **Magnet ห่างเกินไป** จาก encoder  
❌ **สายสัญญาณยาวเกินไป**  

---

## 📊 Wiring Diagram

```
                    BLEGS Actuator Unit
                   ┌─────────────────┐
    24V ──────────→│ V_MOTOR         │
    GND ──────────→│ GND             │
                   │                 │
    BLDC ─────────→│ Phase A,B,C     │
                   │                 │
    Encoder ──────→│ SPI1 (PA4-7)    │
                   │                 │
    USB-Serial ───→│ UART2 (PA2-3)   │
                   │                 │
    LED ──────────→│ PB0             │
    Button ───────→│ PC13            │
                   └─────────────────┘
```

---

## 🔗 Related Documents

- [Quick Start Guide](../getting-started/QUICK_START.md) - การเริ่มต้นใช้งาน
- [Troubleshooting](./TROUBLESHOOTING.md) - แก้ไขปัญหา hardware
- [Pin Configuration](./PIN_CONFIGURATION.md) - รายละเอียด pinout ทั้งหมด
- [Motor Specifications](../technical/MOTOR_SPECS.md) - ข้อมูล motor

---

**Last Updated:** December 20, 2025  
**Version:** 1.0
