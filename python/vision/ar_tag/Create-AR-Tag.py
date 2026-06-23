import cv2
import numpy as np

# ==========================================
# 1. การตั้งค่าพารามิเตอร์ (Parameters)
# ==========================================
# เลือก Dictionary ที่จะใช้ (แนะนำ DICT_6X6_250 สำหรับงาน Tracking)
ARUCO_DICT = cv2.aruco.DICT_6X6_250

# ระบุ ID ของ Marker ที่ต้องการสร้าง (เราจะสร้าง ID: 0 ถึง 4 สำหรับจุด A, B, C, D, E)
MARKER_IDS = [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11] 

# ตั้งค่าความละเอียดพิกเซล (Pixels)
# เพื่อให้พิมพ์ออกมาแล้วคมชัด เราจะเซ็ตความละเอียดให้สูงๆ (เช่น 300 DPI)
# ขนาดจริง 46 มม. = 1.81 นิ้ว -> 1.81 * 300 DPI ≈ 543 Pixels
IMAGE_SIZE_PX = 600  # ขนาดของภาพรวมทั้งหมด (พิกเซล)

# ขนาดของ Marker สีดำตรงกลาง
# ต้องเว้นขอบขาวไว้ให้ OpenCV จับขอบ (Edge) ได้ง่าย
MARKER_SIZE_PX = 400 # ขนาดของส่วนสีดำตรงกลาง (เว้นขอบขาวไว้ข้างละ 100 พิกเซล)

# ==========================================
# 2. การสร้างภาพ (Generation)
# ==========================================
# ดึง Dictionary
dictionary = cv2.aruco.getPredefinedDictionary(ARUCO_DICT)

for marker_id in MARKER_IDS:
    # 2.1 สร้างภาพพื้นหลังสีขาวล้วนขนาด 600x600 พิกเซล
    # (np.ones จะได้ค่า 1, แล้วคูณ 255 เพื่อให้เป็นสีขาว)
    bg_image = np.ones((IMAGE_SIZE_PX, IMAGE_SIZE_PX), dtype=np.uint8) * 255
    
    # 2.2 สร้าง ArUco Marker เฉพาะส่วนสีดำ
    marker_image = cv2.aruco.generateImageMarker(dictionary, marker_id, MARKER_SIZE_PX)
    
    # 2.3 คำนวณจุดที่จะวาง Marker สีดำลงไปตรงกลางแผ่นสีขาว
    start_point = (IMAGE_SIZE_PX - MARKER_SIZE_PX) // 2
    end_point = start_point + MARKER_SIZE_PX
    
    # 2.4 นำ Marker สีดำ ไปแปะทับลงบนพื้นหลังสีขาว
    bg_image[start_point:end_point, start_point:end_point] = marker_image
    
    # ==========================================
# 3. บันทึกเป็นไฟล์ (Save to File)
# ==========================================
    # ตีกรอบเส้นสีเทาบางๆ รอบขอบนอกสุด (เผื่อใช้เป็นเส้นไกด์ตอนใช้กรรไกรตัด)
    cv2.rectangle(bg_image, (0, 0), (IMAGE_SIZE_PX-1, IMAGE_SIZE_PX-1), (200, 200, 200), 2)
    
    filename = f"ArUco_ID{marker_id}_46x46mm.png"
    cv2.imwrite(filename, bg_image)
    print(f"✅ สร้างไฟล์สำเร็จ: {filename}")

print("\n🎉 สร้าง Marker ทั้งหมดเสร็จสิ้นแล้ว!")
print("💡 วิธีพิมพ์: ตอนสั่งพิมพ์ ให้เลือก 'Actual Size' หรือพิมพ์ให้ภาพมีขนาด 4.6 x 4.6 เซนติเมตรเป๊ะๆ นะครับ")