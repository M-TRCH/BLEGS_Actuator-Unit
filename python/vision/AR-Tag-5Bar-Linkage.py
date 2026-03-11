"""
AR-Tag-5Bar-Linkage.py
======================
Process a video file to detect ArUco markers (IDs 1–4) and overlay a
separated 5-bar linkage. Computes the end-effector point E in pixel
space using calibrated link-ratios.

Marker ↔ Kinematic points:
    ID 1 = A  (motor joint A)
    ID 2 = B  (motor joint B)
    ID 3 = C  (end of link AC – 105 mm)
    ID 4 = D  (end of link BD – 105 mm)
    E = computed in pixel-space (circle-intersection C(145) ∩ D(145))
        radius calibrated from measured AC/BD pixel distances (ratio 145/105)

Controls:
    [Space]  Pause / Play
    [→] / d  Jump forward 5 s
    [←] / a  Jump back 5 s
    [q]      Quit
"""

import cv2
import numpy as np
import matplotlib
matplotlib.use("TkAgg")
import matplotlib.pyplot as plt
import time
import sys
import os
import threading
import collections
import csv
import tkinter as tk
from tkinter import filedialog

# ─────────────────────────────────────────────────────────────
# 1. ตั้งค่า ArUco detector
# ─────────────────────────────────────────────────────────────
aruco_dict   = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
aruco_params = cv2.aruco.DetectorParameters()

aruco_params.adaptiveThreshWinSizeMin    = 3
aruco_params.adaptiveThreshWinSizeMax    = 23
aruco_params.adaptiveThreshWinSizeStep   = 10
aruco_params.minMarkerPerimeterRate      = 0.03
aruco_params.maxMarkerPerimeterRate      = 4.0
aruco_params.polygonalApproxAccuracyRate = 0.05
aruco_params.cornerRefinementMethod      = cv2.aruco.CORNER_REFINE_NONE

detector = cv2.aruco.ArucoDetector(aruco_dict, aruco_params)

# ─────────────────────────────────────────────────────────────
# 2. ขนาด Marker และ Object Points
# ─────────────────────────────────────────────────────────────
MARKER_SIZE_METERS = 0.03125   # 3.125 cm (วัดส่วนสีดำของ tag จริง)

OBJ_POINTS = np.array([
    [-MARKER_SIZE_METERS / 2,  MARKER_SIZE_METERS / 2, 0],
    [ MARKER_SIZE_METERS / 2,  MARKER_SIZE_METERS / 2, 0],
    [ MARKER_SIZE_METERS / 2, -MARKER_SIZE_METERS / 2, 0],
    [-MARKER_SIZE_METERS / 2, -MARKER_SIZE_METERS / 2, 0],
], dtype=np.float32)

# ─────────────────────────────────────────────────────────────
# 3. พารามิเตอร์ Separated 5-Bar Linkage  (หน่วย mm)
#    อ้างอิง: relative_position_control.py  (Command [7])
# ─────────────────────────────────────────────────────────────
L_AC = 105.0
L_BD = 105.0
L_CE = 145.0   # = (145/105) × L_AC
L_DE = 145.0   # = (145/105) × L_BD

# ID → ชื่อจุด IK
ID_TO_POINT = {1: 'A', 2: 'B', 3: 'C', 4: 'D'}

# ─── Performance flags ──────────────────────────────────────
RUN_PNP         = False   # True = เปิด solvePnP + drawFrameAxes (ช้า)
DETECT_INTERVAL = 1       # detect ทุก N เฟรม (1 = ทุกเฟรม, 2 = ครึ่ง)

# ─────────────────────────────────────────────────────────────
# 4. สี / สไตล์ภาพ overlay
# ─────────────────────────────────────────────────────────────
# BGR colors
COLOR_A_B  = (180, 180, 180)    # เทา    – ฐาน A–B
COLOR_A_C  = (0,   255, 255)    # เหลือง – link A→C
COLOR_B_D  = (255, 255,   0)    # ฟ้า    – link B→D
COLOR_C_E  = (0,   165, 255)    # ส้ม    – link C→E
COLOR_D_E  = (255,   0, 255)    # ม่วง   – link D→E

POINT_COLOR = {
    'A': (0,   255, 255),
    'B': (255, 255,   0),
    'C': (0,   165, 255),
    'D': (255,   0, 255),
    'E': (0,   255,   0),   # เขียวสด – end-effector
}
LINK_THICK  = 2
DOT_RADIUS  = 5

# ─────────────────────────────────────────────────────────────
# 5. โหลด Camera Calibration
# ─────────────────────────────────────────────────────────────
script_dir = os.path.dirname(os.path.abspath(__file__))
calib_file = os.path.join(script_dir, "camera_calibration",
                          "camera_params_sigma35f14dgdn_1080p60.npz")
try:
    calib_data    = np.load(calib_file)
    camera_matrix = calib_data["mtx"]
    print(f"Loaded camera parameters from: {calib_file}")
except FileNotFoundError:
    print(f"⚠  Calibration file not found: {calib_file}")
    print("   Using default 1920×1080 – lower accuracy")
    camera_matrix = np.array([[1400., 0., 960.],
                               [0., 1400., 540.],
                               [0.,    0.,   1.]], dtype=np.float64)

# ปิดการแก้ distortion – ไม่มี camera parameters จากการทดลองนี้
dist_coeffs = np.zeros((5, 1), dtype=np.float64)

# ─────────────────────────────────────────────────────────────
# 6. คำนวณ E ในพื้นที่ pixel (2D)
#
#   อัลกอริทึม (mirror calculate_fk_no_ef):
#   1. วัด pixel-distance ของ AC และ BD (= 105 mm จริง) → ได้ scale
#   2. r_CE = d_AC_px × (145/105)   r_DE = d_BD_px × (145/105)
#   3. circle-intersection C(r_CE) ∩ D(r_DE) ใน pixel space
#      v_perp = left-perpendicular ของ unit(C→D)  → E1, E2
#   4. เลือก E ที่ dot(E − mid_AB, y_axis) มากกว่า
#      (y_axis = mid_AB → mid_CD = ทิศปลายเท้า)
# ─────────────────────────────────────────────────────────────
def find_end_effector_px(A_px, B_px, C_px, D_px):
    """คืน (u, v) ของ E ใน pixel  หรือ None"""
    A = np.array(A_px, dtype=np.float64)
    B = np.array(B_px, dtype=np.float64)
    C = np.array(C_px, dtype=np.float64)
    D = np.array(D_px, dtype=np.float64)

    d_AC = np.linalg.norm(C - A)   # pixel-length ของ link AC (= 105 mm)
    d_BD = np.linalg.norm(D - B)   # pixel-length ของ link BD (= 105 mm)
    if d_AC < 1e-6 or d_BD < 1e-6:
        return None

    # radius CE, DE ใน pixel  (อัตราส่วน 145/105 ของ link ที่วัดได้)
    r_CE = d_AC * (L_CE / L_AC)
    r_DE = d_BD * (L_DE / L_BD)

    V_CD = D - C
    d    = np.linalg.norm(V_CD)
    if d < 1e-6 or d > r_CE + r_DE or d < abs(r_CE - r_DE):
        return None

    a    = (r_CE**2 - r_DE**2 + d**2) / (2.0 * d)
    h    = np.sqrt(max(0.0, r_CE**2 - a**2))
    u_cd   = V_CD / d
    v_perp = np.array([-u_cd[1], u_cd[0]])   # left-perpendicular (CCW 90°)

    E1 = C + a * u_cd + h * v_perp
    E2 = C + a * u_cd - h * v_perp

    # เลือก E ฝั่งปลายเท้า (dot product ตาม y_axis มากกว่า)
    mid_AB = (A + B) * 0.5
    y_axis = (C + D) * 0.5 - mid_AB
    y_len  = np.linalg.norm(y_axis)
    if y_len < 1e-6:
        E = E1
    else:
        y_axis /= y_len
        E = E1 if np.dot(E1 - mid_AB, y_axis) >= np.dot(E2 - mid_AB, y_axis) else E2

    return (int(round(E[0])), int(round(E[1])))


def draw_linkage(img, pts_px):
    """
    วาด Separated 5-Bar Linkage ลงบนภาพ (in-place)

    pts_px: dict ชื่อจุด → (u, v)  เช่น {'A': (100, 200), 'E': (150, 300), ...}
    """
    h_img, w_img = img.shape[:2]

    def safe(key):
        p = pts_px.get(key)
        if p is None:
            return None
        if 0 <= p[0] < w_img and 0 <= p[1] < h_img:
            return p
        return None

    def line(k1, k2, color, thick=LINK_THICK):
        p1, p2 = safe(k1), safe(k2)
        if p1 and p2:
            cv2.line(img, p1, p2, color, thick, cv2.LINE_AA)

    def dot(key, color, r=DOT_RADIUS):
        p = safe(key)
        if p:
            cv2.circle(img, p, r, (0, 0, 0), -1)           # เงา
            cv2.circle(img, p, r - 1, color, -1)

    def label(key, color, text=None):
        p = safe(key)
        if p:
            lbl = text if text else key
            cv2.putText(img, lbl, (p[0] + 8, p[1] - 8),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 2, cv2.LINE_AA)

    # ─ วาดลิงก์ ─────────────────────────────────────────────
    line('A', 'B', COLOR_A_B, 1)          # ฐาน (บาง)
    line('A', 'C', COLOR_A_C)             # A→C
    line('B', 'D', COLOR_B_D)             # B→D
    line('C', 'E', COLOR_C_E, LINK_THICK + 1)
    line('D', 'E', COLOR_D_E, LINK_THICK + 1)

    # ─ วาดจุด + label ───────────────────────────────────────
    for name in ('A', 'B', 'C', 'D'):
        dot(name, POINT_COLOR[name])
        label(name, POINT_COLOR[name])

    # จุด E ใหญ่กว่าและมี ring
    epos = safe('E')
    if epos:
        cv2.circle(img, epos, DOT_RADIUS + 4, POINT_COLOR['E'], 2, cv2.LINE_AA)
        cv2.circle(img, epos, DOT_RADIUS + 1, POINT_COLOR['E'], -1)
        cv2.putText(img, 'E', (epos[0] + 8, epos[1] - 8),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, POINT_COLOR['E'], 2, cv2.LINE_AA)


# ─────────────────────────────────────────────────────────────
# 7. เลือกไฟล์วิดีโอ
# ─────────────────────────────────────────────────────────────
if len(sys.argv) > 1:
    video_path = sys.argv[1]
else:
    root = tk.Tk()
    root.withdraw()
    video_path = filedialog.askopenfilename(
        title="เลือกไฟล์วิดีโอ",
        filetypes=[
            ("Video files", "*.mp4 *.avi *.mov *.mkv *.wmv *.flv *.webm"),
            ("All files", "*.*"),
        ],
    )
    root.destroy()

if not video_path or not os.path.isfile(video_path):
    print("No file selected or file not found — exiting")
    sys.exit(1)

print(f"\nVideo file: {video_path}")

cap = cv2.VideoCapture(video_path)
if not cap.isOpened():
    print("ไม่สามารถเปิดไฟล์วิดีโอได้")
    sys.exit(1)

total_frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
video_fps    = cap.get(cv2.CAP_PROP_FPS) or 30.0
vid_w        = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
vid_h        = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
print(f"Resolution: {vid_w}×{vid_h}  |  FPS: {video_fps:.1f}  |  Frames: {total_frames}")

# ─── CSV log setup ───────────────────────────────────────────────────────────
_log_stamp  = time.strftime("%Y%m%d_%H%M%S")
_log_name   = os.path.splitext(os.path.basename(video_path))[0]
_log_path   = os.path.join(os.path.dirname(video_path),
                           f"{_log_name}_linkage_{_log_stamp}.csv")
_log_file   = open(_log_path, "w", newline="", encoding="utf-8")
_log_writer = csv.writer(_log_file)
_log_writer.writerow(["timestamp_s", "frame", "E_x_mm", "E_y_mm"])
print(f"Logging to: {_log_path}")

DISPLAY_SCALE = min(1280 / vid_w, 720 / vid_h, 1.0)
disp_w        = int(vid_w * DISPLAY_SCALE)
disp_h        = int(vid_h * DISPLAY_SCALE)
DETECTION_SCALE = 0.5
frame_period    = 1.0 / video_fps   # ← real-time sync (วินาทีต่อเฟรม)

# ─── Threaded video reader ──────────────────────────────────
# อ่านเฟรมล่วงหน้าใน thread แยก เพื่อไม่ให้ decode block main loop
_frame_q    = collections.deque(maxlen=8)
_reader_run = True

def _reader_thread(cap_obj):
    global _reader_run
    while _reader_run:
        pos_ms = cap_obj.get(cv2.CAP_PROP_POS_MSEC)   # video timestamp of next frame
        ret, frm = cap_obj.read()
        if not ret:
            _frame_q.append(None)   # sentinel = end of video
            break
        _frame_q.append((frm, pos_ms))

_reader_t = threading.Thread(target=_reader_thread, args=(cap,), daemon=True)
_reader_t.start()

# ─────────────────────────────────────────────────────────────
# 8. แปลง pixel → robot mm frame
#    A_robot = (-42.5, 0)   B_robot = (42.5, 0)  (mm)
# ─────────────────────────────────────────────────────────────
A_ROBOT = np.array([-42.5, 0.0])   # mm
B_ROBOT = np.array([ 42.5, 0.0])   # mm  spacing = 85 mm

def px_to_robot(P_px, A_px, B_px):
    """
    แปลง pixel-coordinate → robot mm frame
    ใช้ A_px → (-42.5, 0) และ B_px → (42.5, 0) เป็น reference
    robot Y: ขึ้น = บวก  (ขาแกว่งลงมา = ค่าลบ)
    """
    A  = np.array(A_px, dtype=np.float64)
    B  = np.array(B_px, dtype=np.float64)
    P  = np.array(P_px, dtype=np.float64)

    mid   = (A + B) * 0.5
    AB    = B - A
    d_AB  = np.linalg.norm(AB)
    if d_AB < 1e-6:
        return None

    x_hat = AB / d_AB
    # CW 90° = [x[1], -x[0]]  →  ชี้ขึ้นในภาพ = robot +Y
    y_hat = np.array([x_hat[1], -x_hat[0]])
    scale = d_AB / 85.0            # px per mm

    v = P - mid
    rx = np.dot(v, x_hat) / scale  # mm
    ry = np.dot(v, y_hat) / scale  # mm
    return np.array([rx, ry])


# ─────────────────────────────────────────────────────────────
# 9. ตั้งค่า Matplotlib – live 2D linkage plot (robot mm frame)
# ─────────────────────────────────────────────────────────────
plt.ion()
fig_rob, ax_rob = plt.subplots(figsize=(6, 8))
fig_rob.suptitle("5-Bar Linkage – Robot Frame (mm)", fontsize=12)
ax_rob.set_xlabel("X (mm)")
ax_rob.set_ylabel("Y (mm)")
ax_rob.set_xlim(-200, 200)
ax_rob.set_ylim(-280, 80)
ax_rob.set_aspect("equal")
ax_rob.grid(True, alpha=0.35)
ax_rob.axhline(0, color="gray", linewidth=0.8, linestyle="--")
ax_rob.axvline(0, color="gray", linewidth=0.8, linestyle="--")

# Fixed motor markers
ax_rob.plot(*A_ROBOT, marker="o", markersize=10,
            color=(0, 1, 1), label="A (-42.5, 0)")
ax_rob.plot(*B_ROBOT, marker="o", markersize=10,
            color=(1, 1, 0), label="B (42.5, 0)")
ax_rob.annotate("A", xy=A_ROBOT, xytext=(-8, 6), textcoords="offset points",
                color=(0, 1, 1), fontsize=10, fontweight="bold")
ax_rob.annotate("B", xy=B_ROBOT, xytext=(4, 6), textcoords="offset points",
                color=(1, 1, 0), fontsize=10, fontweight="bold")

# Live plot handles (สำหรับ update)
_MCOLOR = {"C": (1, 0.65, 0), "D": (1, 0, 1), "E": (0, 1, 0)}
_rob_pts  = {}   # name → scatter artist
_rob_lbls = {}   # name → text artist
_rob_segs = {}   # (k1,k2) → line2D

for name in ("C", "D", "E"):
    sc, = ax_rob.plot([], [], marker="o", markersize=7,
                      color=_MCOLOR[name], linestyle="None", label=name)
    lb = ax_rob.annotate("", xy=(0, 0), xytext=(5, 5),
                         textcoords="offset points",
                         color=_MCOLOR[name], fontsize=9)
    _rob_pts[name]  = sc
    _rob_lbls[name] = lb

for (k1, k2, c) in [("A", "B",  (0.7, 0.7, 0.7)),
                     ("A", "C",  (0, 1, 1)),
                     ("B", "D",  (1, 1, 0)),
                     ("C", "E",  (1, 0.65, 0)),
                     ("D", "E",  (1, 0, 1))]:
    seg, = ax_rob.plot([], [], color=c, linewidth=2)
    _rob_segs[(k1, k2)] = seg

ax_rob.legend(loc="upper right", fontsize=8)
fig_rob.tight_layout()
fig_rob.canvas.draw()
plt.pause(0.001)

_last_plot_t   = time.time()
_PLOT_INTERVAL = 0.15   # วินาที (ลด redraw → เร็วขึ้น)

# ─── Blitting: จำ background เพื่อ restore แทน full redraw ────
fig_rob.canvas.draw()
_blit_bg = fig_rob.canvas.copy_from_bbox(ax_rob.bbox)
_blit_artists = (list(_rob_pts.values())
                 + list(_rob_lbls.values())
                 + list(_rob_segs.values()))


def update_robot_plot(rob_pts):
    """อัปเดตกราฟ robot mm frame  (rob_pts: dict name→np.array([x,y]))"""
    global _last_plot_t
    now = time.time()
    if now - _last_plot_t < _PLOT_INTERVAL:
        return
    _last_plot_t = now

    # จุด A, B ตายตัว – ไม่ต้อง update
    all_pts = {"A": A_ROBOT, "B": B_ROBOT}
    all_pts.update(rob_pts)

    for name in ("C", "D", "E"):
        if name in rob_pts:
            xy = rob_pts[name]
            _rob_pts[name].set_data([xy[0]], [xy[1]])
            _rob_lbls[name].set_position(xy)
            _rob_lbls[name].set_text(f"{name} ({xy[0]:.1f},{xy[1]:.1f})")
        else:
            _rob_pts[name].set_data([], [])
            _rob_lbls[name].set_text("")

    for (k1, k2), seg in _rob_segs.items():
        if k1 in all_pts and k2 in all_pts:
            p1, p2 = all_pts[k1], all_pts[k2]
            seg.set_data([p1[0], p2[0]], [p1[1], p2[1]])
        else:
            seg.set_data([], [])

    # ─── blit: เร็วกว่า full draw_idle 5–10× ─────────────
    fig_rob.canvas.restore_region(_blit_bg)
    for art in _blit_artists:
        ax_rob.draw_artist(art)
    fig_rob.canvas.blit(ax_rob.bbox)
    fig_rob.canvas.flush_events()


# ─────────────────────────────────────────────────────────────
# 10. Loop หลัก  (real-time sync + threaded read)
# ─────────────────────────────────────────────────────────────
frame_idx     = 0
paused        = False
seek_step     = int(video_fps * 5)
frame         = None
last_pts_px   = {}   # จุดในหน้าจอของเฟรมล่าสุด ไว้แสดงขณะ PAUSED
detect_ctr    = 0    # นับเฟรมเพื่อ skip detection

# FPS counter
_fps_t0       = time.perf_counter()
_fps_cnt      = 0
_fps_display  = 0.0

# Real-time sync: wall-clock ที่เฟรมแรกเริ่มเล่น
_play_t0      = None

print("\n[Space] Pause/Play  |  [→/d] Forward 5s  |  [←/a] Back 5s  |  [q] Quit\n")

while True:
    # ── อ่านเฟรมจาก threaded queue ──────────────────────────
    if not paused:
        # Drain deque — keep only the latest item to stay in real-time sync
        new_item = None
        while _frame_q:
            new_item = _frame_q.popleft()
        if new_item is None and not _reader_t.is_alive():
            print("End of video")
            break
        if new_item is not None:
            frame, _pos_ms = new_item
            timestamp  = _pos_ms / 1000.0
            frame_idx  = int(round(_pos_ms * video_fps / 1000.0))
            if _play_t0 is None:
                _play_t0 = time.perf_counter()

    if frame is None:
        cv2.waitKey(1)
        continue

    # timestamp and frame_idx are set when a new frame arrives;
    # in paused state they retain the values from the last displayed frame.

    # ── detect (อาจ skip ตาม DETECT_INTERVAL) ──────────────
    detect_ctr += 1
    run_detect = (detect_ctr % DETECT_INTERVAL == 0)

    pts_px   = {}   # name → (u, v) pixel center
    tvec_3d  = {}   # name → tvec ravel  (for HUD depth display)

    if run_detect:
        if DETECTION_SCALE < 1.0:
            small = cv2.resize(frame, None, fx=DETECTION_SCALE, fy=DETECTION_SCALE,
                               interpolation=cv2.INTER_LINEAR)
            gray  = cv2.cvtColor(small, cv2.COLOR_BGR2GRAY)
        else:
            gray  = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        corners, ids, _ = detector.detectMarkers(gray)
        if DETECTION_SCALE < 1.0 and corners:
            corners = [c / DETECTION_SCALE for c in corners]

        if ids is not None:
            cv2.aruco.drawDetectedMarkers(frame, corners, ids)

            for i, corner in enumerate(corners):
                mid_id = int(ids[i][0])
                if mid_id not in ID_TO_POINT:
                    continue
                pt_name = ID_TO_POINT[mid_id]

                center          = corner[0].mean(axis=0).astype(np.int32)
                pts_px[pt_name] = tuple(center)

                if RUN_PNP:
                    ok, rvec, tvec = cv2.solvePnP(
                        OBJ_POINTS, corner[0],
                        camera_matrix, dist_coeffs,
                        flags=cv2.SOLVEPNP_IPPE_SQUARE,
                    )
                    if ok:
                        tvec_3d[pt_name] = tvec.ravel()
                        cv2.drawFrameAxes(frame, camera_matrix, dist_coeffs,
                                          rvec, tvec, 0.02)

    # ── คำนวณ E ใน pixel-space ──────────────────────────────
    E_px = None
    if all(k in pts_px for k in ('A', 'B', 'C', 'D')):
        E_px = find_end_effector_px(
            pts_px['A'], pts_px['B'],
            pts_px['C'], pts_px['D'],
        )
        if E_px:
            pts_px['E'] = E_px

    # ── แปลง pixel → robot mm และอัปเดตกราฟ ───────────────
    rob_pts = {}
    if 'A' in pts_px and 'B' in pts_px:
        for name in ('C', 'D', 'E'):
            if name in pts_px:
                r = px_to_robot(pts_px[name], pts_px['A'], pts_px['B'])
                if r is not None:
                    rob_pts[name] = r
        update_robot_plot(rob_pts)
        # ─── write log row when E is visible ────────────────
        if 'E' in rob_pts:
            ex, ey = rob_pts['E']
            _log_writer.writerow([f"{timestamp:.4f}", frame_idx,
                                   f"{ex:.3f}", f"{ey:.3f}"])

    # ── อัปเดต last_pts_px และวาด linkage ─────────────────
    if pts_px:
        last_pts_px = pts_px

    draw_linkage(frame, pts_px if pts_px else last_pts_px)

    # ── HUD ─────────────────────────────────────────────────
    ts_text = f"t={timestamp:.2f}s  f={frame_idx}/{total_frames}"
    cv2.rectangle(frame, (5, 5), (390, 32), (0, 0, 0), -1)
    cv2.putText(frame, ts_text, (8, 25),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2, cv2.LINE_AA)

    # แสดง pixel distance ของ link และ ratio CE/DE เพื่อ verify
    hud_y = 55
    for (k1, k2, lbl, color) in [('A', 'C', 'AC', COLOR_A_C),
                                   ('B', 'D', 'BD', COLOR_B_D)]:
        if k1 in pts_px and k2 in pts_px:
            dpx = np.linalg.norm(np.array(pts_px[k1], dtype=float)
                                  - np.array(pts_px[k2], dtype=float))
            cv2.putText(frame, f"{lbl}: {dpx:.1f}px (105mm)",
                        (8, hud_y), cv2.FONT_HERSHEY_SIMPLEX, 0.44,
                        color, 1, cv2.LINE_AA)
            hud_y += 20

    if E_px and 'C' in pts_px and 'D' in pts_px:
        d_CE = np.linalg.norm(np.array(E_px, dtype=float)
                               - np.array(pts_px['C'], dtype=float))
        d_DE = np.linalg.norm(np.array(E_px, dtype=float)
                               - np.array(pts_px['D'], dtype=float))
        cv2.putText(frame, f"CE:{d_CE:.1f}px  DE:{d_DE:.1f}px (145mm)",
                    (8, hud_y), cv2.FONT_HERSHEY_SIMPLEX, 0.44,
                    POINT_COLOR['E'], 1, cv2.LINE_AA)
        hud_y += 20

    # แสดง Z depth ของแต่ละ tag (เฉพาะ RUN_PNP)
    if RUN_PNP:
        for name in ('A', 'B', 'C', 'D'):
            if name in tvec_3d:
                cv2.putText(frame, f"{name}:Z={tvec_3d[name][2]:.3f}m",
                            (8, hud_y), cv2.FONT_HERSHEY_SIMPLEX, 0.44,
                            POINT_COLOR[name], 1, cv2.LINE_AA)
                hud_y += 20

    # ── FPS counter ─────────────────────────────────────────
    _fps_cnt += 1
    _fps_now  = time.perf_counter()
    if _fps_now - _fps_t0 >= 1.0:
        _fps_display = _fps_cnt / (_fps_now - _fps_t0)
        _fps_cnt     = 0
        _fps_t0      = _fps_now
    cv2.putText(frame, f"FPS: {_fps_display:.0f}",
                (vid_w - 160, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                (0, 255, 0), 2, cv2.LINE_AA)

    if paused:
        cv2.putText(frame, "PAUSED", (vid_w // 2 - 90, vid_h // 2),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.4, (0, 0, 255), 3, cv2.LINE_AA)

    # ── Legend ───────────────────────────────────────────────
    legend_items = [
        ("A-C  105mm", COLOR_A_C),
        ("B-D  105mm", COLOR_B_D),
        ("C-E  145mm", COLOR_C_E),
        ("D-E  145mm", COLOR_D_E),
        ("E  end-eff",  POINT_COLOR['E']),
    ]
    for li, (lname, lcolor) in enumerate(legend_items):
        ly_pos = vid_h - 130 + li * 24
        cv2.rectangle(frame, (8, ly_pos), (22, ly_pos + 14), lcolor, -1)
        cv2.putText(frame, lname, (28, ly_pos + 12),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.44, lcolor, 1, cv2.LINE_AA)

    # ── แสดงผล ──────────────────────────────────────────────
    display = cv2.resize(frame, (disp_w, disp_h), interpolation=cv2.INTER_LINEAR)
    cv2.imshow("5-Bar Linkage – AR Tag  (Space / a / d / q)", display)

    # ── real-time sync: waitKey เท่าที่เวลาเหลือ ────────────
    if paused:
        wait_ms = 0   # block จนกว่าจะกดปุ่ม
    elif _play_t0 is not None:
        elapsed   = time.perf_counter() - _play_t0
        ideal     = frame_idx * frame_period
        remaining = ideal - elapsed
        wait_ms   = max(1, int(remaining * 1000))
    else:
        wait_ms = 1

    key = cv2.waitKey(wait_ms) & 0xFF
    if key == ord('q'):
        break
    elif key == ord(' '):
        paused = not paused
        if not paused:
            # reset sync clock หลัง unpause
            _play_t0 = time.perf_counter() - frame_idx * frame_period
    elif key in (83, ord('d')):   # →
        _reader_run = False
        _reader_t.join(timeout=0.5)
        frame_idx = min(frame_idx + seek_step, total_frames - 1)
        cap.set(cv2.CAP_PROP_POS_FRAMES, frame_idx)
        _frame_q.clear()
        # อ่าน 1 เฟรมใหม่ใน main thread ก่อน restart reader
        ret, frame = cap.read()
        if not ret:
            break
        _reader_run = True
        _reader_t = threading.Thread(target=_reader_thread, args=(cap,), daemon=True)
        _reader_t.start()
        _play_t0 = time.perf_counter() - frame_idx * frame_period
    elif key in (81, ord('a')):   # ←
        _reader_run = False
        _reader_t.join(timeout=0.5)
        frame_idx = max(frame_idx - seek_step, 0)
        cap.set(cv2.CAP_PROP_POS_FRAMES, frame_idx)
        _frame_q.clear()
        ret, frame = cap.read()
        if not ret:
            break
        _reader_run = True
        _reader_t = threading.Thread(target=_reader_thread, args=(cap,), daemon=True)
        _reader_t.start()
        _play_t0 = time.perf_counter() - frame_idx * frame_period

# ─────────────────────────────────────────────────────────────
# 11. ปิด
# ─────────────────────────────────────────────────────────────
_reader_run = False
_reader_t.join(timeout=1.0)
cap.release()
cv2.destroyAllWindows()
_log_file.close()
print(f"Log saved: {_log_path}")
plt.ioff()
plt.show()
print("Exiting program")
