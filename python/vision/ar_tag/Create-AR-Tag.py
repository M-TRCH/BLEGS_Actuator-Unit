import cv2
import numpy as np
import os

# --- ตั้งค่า Marker ---
# เลือก Dictionary (ตระกูลของ Tag)
# DICT_6X6_250 คือตระกูลที่มี 250 Tag ขนาด 6x6 blocks
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)

# เลือก ID ของ Marker (เลือกได้ 0-249)
marker_id = 7 

# ขนาดของภาพ Marker ที่จะสร้าง (pixels)
img_size = 600 
#------------------------

# สร้าง Marker
marker_image = np.zeros((img_size, img_size), dtype=np.uint8)
marker_image = cv2.aruco.generateImageMarker(aruco_dict, marker_id, img_size, marker_image, 1)

# สร้างชื่อไฟล์ตาม Dictionary, ขนาด, และ ID
dict_name = "6X6_250"  # ชื่อของ Dictionary ที่ใช้
filename = f"aruco_{dict_name}_size{img_size}_id{marker_id}.png"

# หา path ของไฟล์ Python ปัจจุบัน
script_dir = os.path.dirname(os.path.abspath(__file__))

# สร้างโฟลเดอร์ tag_images ในโฟลเดอร์เดียวกันกับไฟล์ Python
output_dir = os.path.join(script_dir, "tag_images")
os.makedirs(output_dir, exist_ok=True)

# สร้างเส้นทางไฟล์เต็ม
filepath = os.path.join(output_dir, filename)

# บันทึกเป็นไฟล์
cv2.imwrite(filepath, marker_image)

print(f"สร้าง Marker ID={marker_id} ขนาด {img_size}x{img_size} pixels")
print(f"บันทึกเป็นไฟล์ '{filepath}' เรียบร้อย")

# แสดงผล (กด q เพื่อปิด)
cv2.imshow("ArUco Marker", marker_image)
cv2.waitKey(0)
cv2.destroyAllWindows()