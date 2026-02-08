import cv2
import numpy as np
import glob
import os

# --- ตั้งค่า (ต้องตรงกับไฟล์แรก) ---
CHECKERBOARD_DIMS = (9, 6)
IMAGE_DIR = "calib_images"
EXTENSION = "*.png"

# !!! สำคัญมาก แก้ตรงนี้ !!!
# ขนาดความกว้างของ 1 ช่องสี่เหลี่ยม (หน่วยเป็นเมตร)
# เช่น 2.5 เซนติเมตร = 0.025
SQUARE_SIZE = 0.0386 
# -----------------------------

# ตั้งค่าเงื่อนไขการหยุดคำนวณ (เพื่อความแม่นยำระดับ sub-pixel)
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

# เตรียมชุดข้อมูลพิกัด 3D (0,0,0), (1,0,0), (2,0,0) ...
objp = np.zeros((CHECKERBOARD_DIMS[0] * CHECKERBOARD_DIMS[1], 3), np.float32)
objp[:, :2] = np.mgrid[0:CHECKERBOARD_DIMS[0], 0:CHECKERBOARD_DIMS[1]].T.reshape(-1, 2)
objp = objp * SQUARE_SIZE # คูณด้วยขนาดจริง

# ตัวแปรเก็บค่า
objpoints = [] # จุด 3D ในโลกจริง
imgpoints = [] # จุด 2D บนภาพ
img_shape = None

# ดึงชื่อไฟล์ภาพทั้งหมด
images = glob.glob(os.path.join(IMAGE_DIR, EXTENSION))

if len(images) < 10:
    print(f"คำเตือน: มีภาพแค่ {len(images)} ภาพ (แนะนำ 15-20 ภาพ)")

print(f"กำลังประมวลผล {len(images)} ภาพ...")

for fname in images:
    img = cv2.imread(fname)
    if img is None: continue

    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    
    # เก็บขนาดภาพ (ใช้รูปแรกเป็นหลัก)
    if img_shape is None:
        img_shape = gray.shape[::-1]

    # หาจุดตัด
    ret, corners = cv2.findChessboardCorners(gray, CHECKERBOARD_DIMS, None)

    if ret:
        objpoints.append(objp)
        
        # ปรับความละเอียดจุดตัดให้คมระดับ sub-pixel
        corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
        imgpoints.append(corners2)
        print(f"Pass: {fname}")
    else:
        print(f"Fail: {fname} (หาตารางไม่เจอ)")

print("\nกำลังเริ่มคำนวณ Calibrate... (อาจใช้เวลาสักครู่)")

# ฟังก์ชันหลักในการ Calibrate
ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(
    objpoints, imgpoints, img_shape, None, None
)

if ret:
    print("\n✅ Calibration สำเร็จ!")
    print(f"ความละเอียดภาพ: {img_shape}")
    print("\n--- Camera Matrix (K) ---")
    print(mtx)
    print("\n--- Distortion Coeffs (D) ---")
    print(dist)

    # คำนวณ Error
    mean_error = 0
    for i in range(len(objpoints)):
        imgpoints2, _ = cv2.projectPoints(objpoints[i], rvecs[i], tvecs[i], mtx, dist)
        error = cv2.norm(imgpoints[i], imgpoints2, cv2.NORM_L2) / len(imgpoints2)
        mean_error += error
    print(f"\nRe-projection Error: {mean_error/len(objpoints):.5f} (ยิ่งน้อยยิ่งดี ควร < 1.0)")

    # บันทึกไฟล์
    np.savez("camera_params_1080p60.npz", mtx=mtx, dist=dist)
    print("\nบันทึกไฟล์: camera_params_1080p60.npz เรียบร้อยแล้ว")
else:
    print("\n❌ Calibration ล้มเหลว")