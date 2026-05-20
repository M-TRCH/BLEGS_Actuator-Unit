import os
import cv2
import cv2.aruco as aruco
import numpy as np
import matplotlib.pyplot as plt

# ==========================================
# 1. ตั้งค่าพารามิเตอร์
# ==========================================
VIDEO_UNCOMP = r"C:\Users\mteer\OneDrive\Desktop\nocomp_28_0_-190_5.MOV"   # ไฟล์วิดีโอแบบ ไม่ชดเชย (ปิด ML)
VIDEO_COMP   = r"C:\Users\mteer\OneDrive\Desktop\comp_28_0_-190_5.MOV"     # ไฟล์วิดีโอแบบ ชดเชย (เปิด ML)
CALIB_FILE   = os.path.join(os.path.dirname(os.path.abspath(__file__)), "calibration_olympus25mm.npz")

TARGET_ID = 4  # ID ของ ArTag ที่ปลายเท้า (เคลื่อนที่)

# พิกัดโลกจริง (มม.) ของ motor joint ที่ตรึงอยู่กับที่ (5-bar mechanism)
# id2, id3, id4 เคลื่อนที่ตามกลไก จึงไม่ใช้เป็น reference
REF_WORLD = {
    0: np.array([-42.5, 0.0]),   # motor joint ซ้าย
    1: np.array([ 42.5, 0.0]),   # motor joint ขวา
}
REF_DIST_MM = np.linalg.norm(REF_WORLD[1] - REF_WORLD[0])  # = 85.0 mm

FRAME_STEP  = 1      # ประมวลผลทุก N เฟรม (1 = ทุกเฟรม, 3 = ข้าม 2)

CENTER_X_MM = 0.0       # จุดศูนย์กลางวงโคจรอุดมคติ แกน X
CENTER_Y_MM = -190.0    # จุดศูนย์กลางวงโคจรอุดมคติ แกน Y
RADIUS_MM   = 28.0      # รัศมีวงโคจรอุดมคติ

# ==========================================
# 2. Helper: แปลงผล detectMarkers เป็น dict {id: center_px}
# ==========================================
def _marker_centers(corners, ids):
    result = {}
    if ids is None:
        return result
    for i, mid in enumerate(ids.flatten()):
        c = corners[i][0]
        result[int(mid)] = np.array([np.mean(c[:, 0]), np.mean(c[:, 1])], dtype=np.float32)
    return result

# ==========================================
# 3. Helper: สร้าง pixel→mm similarity transform จาก id0, id1
#    (scale + rotation + translation  — ตรึงด้วย 2 จุด motor joint)
#    คืน None ถ้า detect id0 หรือ id1 ไม่ได้
# ==========================================
def _build_transform(centers):
    if 0 not in centers or 1 not in centers:
        return None

    p0_px = centers[0].astype(np.float64)
    p1_px = centers[1].astype(np.float64)
    p0_mm = REF_WORLD[0].astype(np.float64)
    p1_mm = REF_WORLD[1].astype(np.float64)

    # แก้ระบบ (reflection): pixel Y ชี้ลง → world Y ชี้ขึ้น
    # matrix form: [[a, b, tx], [b, -a, ty]]
    dx_px = p1_px[0] - p0_px[0]
    dy_px = p1_px[1] - p0_px[1]
    dx_mm = p1_mm[0] - p0_mm[0]   # = +85.0
    dy_mm = p1_mm[1] - p0_mm[1]   # =   0.0

    denom = dx_px**2 + dy_px**2
    a  = (dx_mm * dx_px - dy_mm * dy_px) / denom
    b  = (dx_mm * dy_px + dy_mm * dx_px) / denom
    tx = p0_mm[0] - a * p0_px[0] - b * p0_px[1]
    ty = p0_mm[1] - b * p0_px[0] + a * p0_px[1]

    M = np.array([[a,  b, tx],
                  [b, -a, ty]], dtype=np.float64)

    def transform(pt):
        return M @ np.array([float(pt[0]), float(pt[1]), 1.0])

    return transform

# ==========================================
# 4. ฟังก์ชันสำหรับแทร็กวิดีโอ (คืนค่าพิกัด mm โดยตรง)
# ==========================================
def track_video(video_path, calib_file, label):
    print(f"Processing: {video_path}...")

    with np.load(calib_file) as data:
        mtx, dist = data['camera_matrix'], data['dist_coeffs']

    cap = cv2.VideoCapture(video_path)
    if not cap.isOpened():
        raise FileNotFoundError(f"Cannot open video: {video_path}")

    # คำนวณ undistort map ครั้งเดียว (w, h คงที่ตลอดวิดีโอ)
    w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    new_mtx, _ = cv2.getOptimalNewCameraMatrix(mtx, dist, (w, h), 0)

    detector = aruco.ArucoDetector(
        aruco.getPredefinedDictionary(aruco.DICT_6X6_250),
        aruco.DetectorParameters()
    )

    tracked_data = []
    skipped = 0
    frame_count = 0

    while cap.isOpened():
        ret, frame = cap.read()
        if not ret:
            break
        frame_count += 1

        if frame_count % FRAME_STEP != 0:
            continue

        img_undist = cv2.undistort(frame, mtx, dist, None, new_mtx)
        gray = cv2.cvtColor(img_undist, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = detector.detectMarkers(gray)

        centers   = _marker_centers(corners, ids)
        transform = _build_transform(centers)

        if transform is not None and TARGET_ID in centers:
            pt_mm = transform(centers[TARGET_ID])
            tracked_data.append(pt_mm)
        else:
            skipped += 1

        if frame_count % 30 == 0:
            print(f"  Frame {frame_count}  |  tracked {len(tracked_data)}  |  skipped {skipped}", end='\r', flush=True)

    cap.release()
    processed = frame_count // FRAME_STEP
    print(f"\n[OK] {label}: tracked {len(tracked_data)} / {processed} processed frames  ({skipped} skipped — id0/id1 not visible)")
    return np.array(tracked_data)

# ==========================================
# 5. รันประมวลผลทั้ง 2 คลิป
# ==========================================
data_uncomp = track_video(VIDEO_UNCOMP, CALIB_FILE, "Uncompensated")
data_comp   = track_video(VIDEO_COMP,   CALIB_FILE, "ML Compensated")

if len(data_uncomp) == 0 or len(data_comp) == 0:
    print("[ERROR] No data tracked. Check TARGET_ID and video files.")
    exit(1)

# พิกัดอยู่ในหน่วย mm แล้ว (จาก homography) — ไม่ต้องแปลงเพิ่มเติม
mm_uncomp_x = data_uncomp[:, 0]
mm_uncomp_y = data_uncomp[:, 1]
mm_comp_x   = data_comp[:, 0]
mm_comp_y   = data_comp[:, 1]

# สรุป RMS error เทียบกับวงโคจรอุดมคติ
err_uncomp = np.sqrt((mm_uncomp_x - CENTER_X_MM)**2 + (mm_uncomp_y - CENTER_Y_MM)**2) - RADIUS_MM
err_comp   = np.sqrt((mm_comp_x   - CENTER_X_MM)**2 + (mm_comp_y   - CENTER_Y_MM)**2) - RADIUS_MM
rms_u  = np.sqrt(np.mean(err_uncomp**2))
rms_c  = np.sqrt(np.mean(err_comp**2))
mean_u = np.mean(err_uncomp)
mean_c = np.mean(err_comp)
std_u  = np.std(err_uncomp)
std_c  = np.std(err_comp)
max_u  = np.max(np.abs(err_uncomp))
max_c  = np.max(np.abs(err_comp))

print(f"\n{'Metric':<18} {'Uncompensated':>16} {'ML Compensated':>16}")
print("-" * 52)
print(f"{'Mean error':<18} {mean_u:>14.3f} mm {mean_c:>14.3f} mm")
print(f"{'Std dev':<18} {std_u:>14.3f} mm {std_c:>14.3f} mm")
print(f"{'RMS error':<18} {rms_u:>14.3f} mm {rms_c:>14.3f} mm")
print(f"{'Max  error':<18} {max_u:>14.3f} mm {max_c:>14.3f} mm")
print(f"{'Improvement':<18} {'—':>16} {(1 - rms_c/rms_u)*100:>13.1f} %")

# ==========================================
# 6. วาดกราฟวงโคจรอุดมคติ
# ==========================================
theta   = np.linspace(0, 2 * np.pi, 100)
ideal_x = CENTER_X_MM + RADIUS_MM * np.cos(theta)
ideal_y = CENTER_Y_MM + RADIUS_MM * np.sin(theta)

# ==========================================
# 7. พล็อตกราฟเปรียบเทียบ (A/B Testing)
# ==========================================
plt.figure(figsize=(10, 10))
plt.plot(ideal_x, ideal_y, 'k--', linewidth=2, label='Ideal Target (R=25mm)')
plt.scatter(mm_uncomp_x, mm_uncomp_y, c='red',   s=10, alpha=0.5, label='Without ML (Uncompensated)')
plt.scatter(mm_comp_x,   mm_comp_y,   c='green', s=10, alpha=0.5, label='With ML (Compensated)')

# แสดงตำแหน่ง motor joints บนกราฟ
for mid, pos in REF_WORLD.items():
    plt.plot(*pos, 'bs', markersize=10)
    plt.annotate(f'id{mid} (motor)', pos, textcoords='offset points', xytext=(6, 4), fontsize=9, color='blue')

plt.title('Dynamic Trajectory Tracking: Baseline vs ML Compensation', fontsize=14)
plt.xlabel('X Position (mm)', fontsize=12)
plt.ylabel('Y Position (mm)', fontsize=12)
plt.axis('equal')
plt.grid(True, linestyle=':', alpha=0.7)
plt.legend(loc='lower right')

metrics_text = (
    f"{'Metric':<14} {'Uncomp':>9} {'Comp':>9}\n"
    f"{'─'*34}\n"
    f"{'Mean error':<14} {mean_u:>7.3f}mm {mean_c:>7.3f}mm\n"
    f"{'Std dev':<14} {std_u:>7.3f}mm {std_c:>7.3f}mm\n"
    f"{'RMS error':<14} {rms_u:>7.3f}mm {rms_c:>7.3f}mm\n"
    f"{'Max error':<14} {max_u:>7.3f}mm {max_c:>7.3f}mm\n"
    f"{'─'*34}\n"
    f"{'Improvement':<14} {(1 - rms_c/rms_u)*100:>17.1f}%"
)
plt.gca().text(
    0.02, 0.02, metrics_text,
    transform=plt.gca().transAxes,
    fontsize=9, verticalalignment='bottom',
    fontfamily='monospace',
    bbox=dict(boxstyle='round,pad=0.5', facecolor='white', alpha=0.85, edgecolor='gray')
)

out_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'trajectory_comparison.png')
plt.savefig(out_path, dpi=300)
plt.show()
print(f"Saved: {out_path}")