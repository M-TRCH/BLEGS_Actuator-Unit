import cv2
import numpy as np
import os

# ==========================================
# 1. ตั้งค่าพารามิเตอร์การสอบเทียบ
# ==========================================
VIDEO_PATH  = r"C:\Users\mteer\OneDrive\Desktop\chess_board.MOV"
OUTPUT_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "calibration_olympus25mm.npz")

# ขนาดของตารางหมากรุก (จำนวนจุดตัดขอบด้านใน ไม่ใช่จำนวนช่อง)
# ตัวอย่าง: ตาราง 9x6 ช่อง จะมีจุดตัดด้านใน = (8, 5)
CHESSBOARD_SIZE = (9, 6) 

# ขนาดความกว้างของช่องสี่เหลี่ยม 1 ช่อง (หน่วยเป็นมิลลิเมตร)
SQUARE_SIZE = 0.03125  

# การดึงเฟรม (Sub-sampling): วิดีโอ 120fps ดึงทุกๆ 60 เฟรม (ดึง 2 รูปต่อวินาที)
FRAME_STEP = 15

# ==========================================
# 2. เตรียมตัวแปร
# ==========================================
# สร้างพิกัด 3D เสมือนของตารางหมากรุก (0,0,0), (20,0,0), (40,0,0) ...
objp = np.zeros((CHESSBOARD_SIZE[0] * CHESSBOARD_SIZE[1], 3), np.float32)
objp[:, :2] = np.mgrid[0:CHESSBOARD_SIZE[0], 0:CHESSBOARD_SIZE[1]].T.reshape(-1, 2)
objp = objp * SQUARE_SIZE

objpoints = [] # เก็บพิกัด 3D ในโลกจริง
imgpoints = [] # เก็บพิกัด 2D บนรูปภาพ

# ==========================================
# 3. วนลูปอ่านวิดีโอและหาจุดตัด
# ==========================================
cap = cv2.VideoCapture(VIDEO_PATH)
frame_count = 0
valid_frames = 0

print("Extracting frames and detecting chessboard...")

while True:
    ret, frame = cap.read()
    if not ret:
        break
        
    frame_count += 1
    
    # ข้ามเฟรมเพื่อลดความซ้ำซ้อน
    if frame_count % FRAME_STEP != 0:
        continue
        
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    
    # ค้นหาจุดตัดตารางหมากรุก
    ret, corners = cv2.findChessboardCorners(gray, CHESSBOARD_SIZE, None)
    
    if ret == True:
        # ขัดเกลาพิกัดให้แม่นยำระดับ Sub-pixel
        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
        corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
        
        objpoints.append(objp)
        imgpoints.append(corners2)
        valid_frames += 1
        
        print(f"[PASS] Frame {frame_count:6d}  |  valid {valid_frames}")

cap.release()

# ==========================================
# 4. คำนวณสมการเลนส์ (Camera Calibration)
# ==========================================
if valid_frames > 10:
    print(f"\nRunning calibration with {valid_frames} frames...")
    ret, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(
        objpoints, imgpoints, gray.shape[::-1], None, None
    )
    
    # คำนวณ Re-projection Error รายภาพ
    per_errors = []
    for i in range(len(objpoints)):
        imgpts2, _ = cv2.projectPoints(objpoints[i], rvecs[i], tvecs[i], camera_matrix, dist_coeffs)
        err = cv2.norm(imgpoints[i], imgpts2, cv2.NORM_L2) / len(imgpts2)
        per_errors.append(err)

    mean_err = np.mean(per_errors)
    min_err  = np.min(per_errors)
    max_err  = np.max(per_errors)

    # บันทึกค่าลงไฟล์ .npz
    np.savez(OUTPUT_FILE, camera_matrix=camera_matrix, dist_coeffs=dist_coeffs)

    print("\nCalibration successful!")
    print("Camera Matrix:\n", camera_matrix)
    print("\nDistortion Coefficients:\n", dist_coeffs)

    print("\n--- Re-projection Error per frame ---")
    print(f"  {'Frame':>5}  {'Error (px)':>12}  Status")
    print(f"  {'-'*36}")
    for i, e in enumerate(per_errors):
        status = "[good]" if e < 1.0 else "[high]"
        print(f"  {i+1:5d}  {e:12.5f}  {status}")
    print(f"\n  mean = {mean_err:.5f} px  |  min = {min_err:.5f} px  |  max = {max_err:.5f} px")
    print(f"  (target: mean < 1.0 px)")

    print(f"\nSaved: {OUTPUT_FILE}")
else:
    print(f"\nFailed: only {valid_frames} valid frame(s) found (need at least 10)")
    print("Tip: reduce FRAME_STEP or check lighting / motion blur in the video")