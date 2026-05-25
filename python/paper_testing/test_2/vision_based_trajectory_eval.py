import os
import cv2
import cv2.aruco as aruco
import numpy as np
import matplotlib.pyplot as plt

# ==========================================
# 1. ตั้งค่าพารามิเตอร์
# ==========================================
# (path, label, scatter_color)
VIDEOS = [
    (r"D:\THESIS\original.MOV",     "Original",            "dimgray"),
    (r"D:\THESIS\model_mlp.MOV",    "MLP",                 "royalblue"),
    (r"D:\THESIS\model_poly3.MOV",  "Poly-3",              "darkorange"),
    (r"D:\THESIS\model_poly4.MOV",  "Poly-4",              "purple"),
    (r"D:\THESIS\model_forest.MOV", "Random Forest",       "green"),
    (r"D:\THESIS\model_svr.MOV",    "SVR",                 "crimson"),
]
CALIB_FILE   = os.path.join(os.path.dirname(os.path.abspath(__file__)), "output", "params", "calibration_olympus25f1.2.npz")

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
CENTER_Y_MM = -170.0    # จุดศูนย์กลางวงโคจรอุดมคติ แกน Y
RADIUS_MM   = 30.0      # รัศมีวงโคจรอุดมคติ

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
# 5. รันประมวลผลทุกคลิป
# ==========================================
results = []  # list of (label, color, data_array)

for vid_path, label, color in VIDEOS:
    data = track_video(vid_path, CALIB_FILE, label)
    results.append((label, color, data))

# ตรวจสอบว่ามีข้อมูลครบ
for label, color, data in results:
    if len(data) == 0:
        print(f"[ERROR] No data tracked for '{label}'. Check TARGET_ID and video file.")
        exit(1)

# ==========================================
# 5a. คำนวณ metrics ของแต่ละคลิป
# ==========================================
def compute_metrics(data):
    x, y = data[:, 0], data[:, 1]
    err = np.sqrt((x - CENTER_X_MM)**2 + (y - CENTER_Y_MM)**2) - RADIUS_MM
    return {
        "mean": np.mean(err),
        "std":  np.std(err),
        "rms":  np.sqrt(np.mean(err**2)),
        "max":  np.max(np.abs(err)),
    }

metrics_list = [(label, color, data, compute_metrics(data)) for label, color, data in results]

# baseline RMS คือ original (index 0)
rms_baseline = metrics_list[0][3]["rms"]

# พิมพ์ตารางสรุป
col_w = 14
header = f"{'Metric':<16}" + "".join(f"{m[0]:>{col_w}}" for m in metrics_list)
print(f"\n{header}")
print("-" * (16 + col_w * len(metrics_list)))
for key, label_row in [("mean", "Mean error (mm)"), ("std", "Std dev (mm)"),
                        ("rms",  "RMS error (mm)"),  ("max", "Max error (mm)")]:
    row = f"{label_row:<16}" + "".join(f"{m[3][key]:>{col_w}.3f}" for m in metrics_list)
    print(row)
print("-" * (16 + col_w * len(metrics_list)))
impr_row = f"{'Improvement':<16}" + f"{'—':>{col_w}}" + "".join(
    f"{(1 - m[3]['rms'] / rms_baseline) * 100:>{col_w}.1f}" for m in metrics_list[1:]
) + " %"
print(impr_row)

# ==========================================
# 6. วาดกราฟวงโคจรอุดมคติ
# ==========================================
theta   = np.linspace(0, 2 * np.pi, 100)
ideal_x = CENTER_X_MM + RADIUS_MM * np.cos(theta)
ideal_y = CENTER_Y_MM + RADIUS_MM * np.sin(theta)

# ==========================================
# 7. พล็อตกราฟเปรียบเทียบ (Multi-model)
# ==========================================
fig, axes = plt.subplots(1, 2, figsize=(18, 9),
                         gridspec_kw={'width_ratios': [1.4, 1]})

# --- subplot ซ้าย: trajectory scatter ---
ax = axes[0]
ax.plot(ideal_x, ideal_y, 'k--', linewidth=2, label=f'Ideal Target (R={RADIUS_MM:.0f}mm)')

for label, color, data, _ in metrics_list:
    ax.scatter(data[:, 0], data[:, 1], c=color, s=8, alpha=0.45, label=label)

for mid, pos in REF_WORLD.items():
    ax.plot(*pos, 'bs', markersize=10)
    ax.annotate(f'id{mid} (motor)', pos, textcoords='offset points', xytext=(6, 4),
                fontsize=9, color='blue')

ax.set_title('Dynamic Trajectory Tracking: Baseline vs ML Models', fontsize=13)
ax.set_xlabel('X Position (mm)', fontsize=11)
ax.set_ylabel('Y Position (mm)', fontsize=11)
ax.axis('equal')
ax.grid(True, linestyle=':', alpha=0.7)
ax.legend(loc='lower right', fontsize=9)

# --- metrics table text box inside scatter ---
_abbr  = {"Original": "Orig", "MLP": "MLP", "Poly-3": "Poly3",
          "Poly-4": "Poly4", "Random Forest": "RF", "SVR": "SVR"}
_cols  = [_abbr.get(m[0], m[0]) for m in metrics_list]
_cw    = 8
_lw    = 13
_sep   = "-" * (_lw + _cw * len(metrics_list))
_lines = [f"{'':>{_lw}}" + "".join(f"{c:>{_cw}}" for c in _cols), _sep]
for _key, _lbl in [("mean", "Mean (mm)"), ("std",  "Std  (mm)"),
                   ("rms",  "RMS  (mm)"), ("max",  "Max  (mm)")]:
    _lines.append(f"{_lbl:>{_lw}}" + "".join(f"{m[3][_key]:>{_cw}.3f}" for m in metrics_list))
_lines.append(_sep)
_impr_vals = [f"{'—':>{_cw}}"] + [f"{(1 - m[3]['rms'] / rms_baseline) * 100:>+7.1f}%" for m in metrics_list[1:]]
_lines.append(f"{'Improv. (%)':>{_lw}}" + "".join(_impr_vals))
ax.text(0.02, 0.98, "\n".join(_lines),
        transform=ax.transAxes, fontsize=7.5,
        verticalalignment='top', fontfamily='monospace',
        bbox=dict(boxstyle='round,pad=0.4', facecolor='white', alpha=0.88, edgecolor='gray'))

# --- subplot ขวา: bar chart RMS error ---
ax2 = axes[1]
labels_bar  = [m[0] for m in metrics_list]
rms_values  = [m[3]["rms"] for m in metrics_list]
colors_bar  = [m[1] for m in metrics_list]

bars = ax2.bar(labels_bar, rms_values, color=colors_bar, edgecolor='black', linewidth=0.7)
ax2.set_title('RMS Trajectory Error per Model', fontsize=13)
ax2.set_ylabel('RMS Error (mm)', fontsize=11)
ax2.set_xlabel('Model', fontsize=11)
ax2.tick_params(axis='x', rotation=20)
ax2.grid(axis='y', linestyle=':', alpha=0.7)

# ป้ายตัวเลขบนแท่ง + % improvement
for bar, (label, color, data, m) in zip(bars, metrics_list):
    impr = (1 - m["rms"] / rms_baseline) * 100
    impr_str = f"—" if label == metrics_list[0][0] else f"{impr:+.1f}%"
    ax2.text(bar.get_x() + bar.get_width() / 2,
             bar.get_height() + 0.05,
             f"{m['rms']:.3f}\n({impr_str})",
             ha='center', va='bottom', fontsize=8)

plt.tight_layout()

out_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'output', 'plots', 'trajectory_comparison.png')
os.makedirs(os.path.dirname(out_path), exist_ok=True)
plt.savefig(out_path, dpi=300)
plt.show()
print(f"Saved: {out_path}")