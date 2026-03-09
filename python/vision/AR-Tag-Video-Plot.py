"""
AR-Tag-Video-Plot.py
====================
วิเคราะห์ไฟล์วิดีโอเพื่อตรวจจับ ArUco Markers และพล็อตตำแหน่ง (X, Y, Z) ตามเวลา

ใช้งาน:
    python AR-Tag-Video-Plot.py                     # เลือกไฟล์ด้วย dialog
    python AR-Tag-Video-Plot.py path/to/video.mp4   # ระบุไฟล์โดยตรง
"""

import cv2
import numpy as np
import matplotlib
matplotlib.use("TkAgg")          # ใช้ TkAgg backend (เปลี่ยนเป็น "Qt5Agg" ถ้า TkAgg ไม่มี)
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from collections import defaultdict
import sys
import os
import tkinter as tk
from tkinter import filedialog
import time

# ─────────────────────────────────────────────────────────────
# 1. ตั้งค่า ArUco
# ─────────────────────────────────────────────────────────────
aruco_dict   = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
aruco_params = cv2.aruco.DetectorParameters()

aruco_params.adaptiveThreshWinSizeMin      = 3
aruco_params.adaptiveThreshWinSizeMax      = 23
aruco_params.adaptiveThreshWinSizeStep     = 10
aruco_params.minMarkerPerimeterRate        = 0.03
aruco_params.maxMarkerPerimeterRate        = 4.0
aruco_params.polygonalApproxAccuracyRate   = 0.05
aruco_params.cornerRefinementMethod        = cv2.aruco.CORNER_REFINE_NONE

detector = cv2.aruco.ArucoDetector(aruco_dict, aruco_params)

# ─────────────────────────────────────────────────────────────
# 2. ขนาด Marker (เมตร) และ Object Points
# ─────────────────────────────────────────────────────────────
MARKER_SIZE_METERS = 0.03125  # 3.125 cm

OBJ_POINTS = np.array([
    [-MARKER_SIZE_METERS / 2,  MARKER_SIZE_METERS / 2, 0],
    [ MARKER_SIZE_METERS / 2,  MARKER_SIZE_METERS / 2, 0],
    [ MARKER_SIZE_METERS / 2, -MARKER_SIZE_METERS / 2, 0],
    [-MARKER_SIZE_METERS / 2, -MARKER_SIZE_METERS / 2, 0],
], dtype=np.float32)

# ─────────────────────────────────────────────────────────────
# 3. โหลด Camera Calibration
# ─────────────────────────────────────────────────────────────
script_dir = os.path.dirname(os.path.abspath(__file__))
calib_file = os.path.join(script_dir, "camera_calibration",
                          "camera_params_sigma35f14dgdn_1080p60.npz")

try:
    calib_data    = np.load(calib_file)
    camera_matrix = calib_data["mtx"]
    dist_coeffs   = calib_data["dist"]
    print(f"โหลด Camera Parameters จาก: {calib_file}")
except FileNotFoundError:
    print(f"⚠  ไม่พบไฟล์ calibration: {calib_file}")
    print("   ใช้ค่า default (ความแม่นยำต่ำ) – กรุณาทำ camera calibration ก่อน")
    # ค่า default สำหรับ 1920x1080
    camera_matrix = np.array([[1400, 0, 960],
                               [0, 1400, 540],
                               [0,    0,   1]], dtype=np.float64)
    dist_coeffs   = np.zeros((5, 1), dtype=np.float64)

# ─────────────────────────────────────────────────────────────
# 4. เลือกไฟล์วิดีโอ
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
    print("ไม่ได้เลือกไฟล์หรือไม่พบไฟล์ – ออกจากโปรแกรม")
    sys.exit(1)

print(f"\nไฟล์วิดีโอ: {video_path}")

cap = cv2.VideoCapture(video_path)
if not cap.isOpened():
    print("ไม่สามารถเปิดไฟล์วิดีโอได้")
    sys.exit(1)

total_frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
video_fps    = cap.get(cv2.CAP_PROP_FPS) or 30.0
vid_w        = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
vid_h        = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
print(f"ความละเอียด: {vid_w}x{vid_h}  |  FPS: {video_fps:.1f}  |  เฟรมทั้งหมด: {total_frames}")

# ─────────────────────────────────────────────────────────────
# 5. ตั้งค่าหน้าต่างแสดงผล
# ─────────────────────────────────────────────────────────────
DISPLAY_SCALE = min(1280 / vid_w, 720 / vid_h, 1.0)
disp_w = int(vid_w * DISPLAY_SCALE)
disp_h = int(vid_h * DISPLAY_SCALE)

DETECTION_SCALE = 0.5          # Downsample ก่อน detect เพื่อความเร็ว
PLAYBACK_DELAY  = max(1, int(1000 / video_fps))   # ms ต่อเฟรม

# ─────────────────────────────────────────────────────────────
# 6. โครงสร้างเก็บข้อมูลตำแหน่ง
#    history[marker_id] = {"t": [], "x": [], "y": [], "z": []}
# ─────────────────────────────────────────────────────────────
history: dict[int, dict] = defaultdict(lambda: {"t": [], "x": [], "y": [], "z": []})

# ─────────────────────────────────────────────────────────────
# 7. ตั้งค่า Matplotlib (live plot แบบ interactive)
# ─────────────────────────────────────────────────────────────
COLORS = plt.rcParams["axes.prop_cycle"].by_key()["color"]
plt.ion()

fig, axes = plt.subplots(3, 1, figsize=(10, 7), sharex=True)
fig.suptitle("AR Tag Position vs Time", fontsize=14)
axes[0].set_ylabel("X (m)");  axes[0].grid(True, alpha=0.4)
axes[1].set_ylabel("Y (m)");  axes[1].grid(True, alpha=0.4)
axes[2].set_ylabel("Z (m)");  axes[2].grid(True, alpha=0.4)
axes[2].set_xlabel("Time (s)")
fig.tight_layout()
fig.canvas.draw()
plt.pause(0.001)

plot_lines: dict[int, dict] = {}   # plot_lines[id] = {"x": line, "y": line, "z": line}
last_plot_update = time.time()
PLOT_INTERVAL = 0.1   # อัพเดทกราฟทุก 100 ms เพื่อไม่ให้ช้า

def get_color(marker_id: int) -> str:
    """เลือกสีจาก cycle ตาม marker id"""
    idx = list(history.keys()).index(marker_id) % len(COLORS)
    return COLORS[idx]

def update_plot(force: bool = False):
    """วาดกราฟใหม่จากข้อมูลใน history"""
    global last_plot_update
    now = time.time()
    if not force and (now - last_plot_update) < PLOT_INTERVAL:
        return
    last_plot_update = now

    for marker_id, data in history.items():
        if len(data["t"]) < 2:
            continue
        color = get_color(marker_id)
        label = f"ID {marker_id}"

        if marker_id not in plot_lines:
            lx, = axes[0].plot(data["t"], data["x"], color=color, label=label, linewidth=1.2)
            ly, = axes[1].plot(data["t"], data["y"], color=color, label=label, linewidth=1.2)
            lz, = axes[2].plot(data["t"], data["z"], color=color, label=label, linewidth=1.2)
            plot_lines[marker_id] = {"x": lx, "y": ly, "z": lz}
            for ax in axes:
                ax.legend(loc="upper right", fontsize=8)
        else:
            plot_lines[marker_id]["x"].set_data(data["t"], data["x"])
            plot_lines[marker_id]["y"].set_data(data["t"], data["y"])
            plot_lines[marker_id]["z"].set_data(data["t"], data["z"])

    # ปรับ axis limits อัตโนมัติ
    for ax in axes:
        ax.relim()
        ax.autoscale_view()

    fig.canvas.draw_idle()
    fig.canvas.flush_events()

# ─────────────────────────────────────────────────────────────
# 8. Loop ประมวลผลวิดีโอ
# ─────────────────────────────────────────────────────────────
frame_idx   = 0
paused      = False
seek_step   = int(video_fps * 5)   # กด ← → กระโดด 5 วินาที

print("\n[Space] หยุด/เล่น  |  [← →] เดินหน้า/ถอยหลัง 5 วิ  |  [q] ออก\n")

while True:
    if not paused:
        ret, frame = cap.read()
        if not ret:
            print("สิ้นสุดวิดีโอ")
            break
        frame_idx = int(cap.get(cv2.CAP_PROP_POS_FRAMES)) - 1

    timestamp = frame_idx / video_fps

    # ── Detect ──────────────────────────────────────────────
    if DETECTION_SCALE < 1.0:
        small = cv2.resize(frame, None, fx=DETECTION_SCALE, fy=DETECTION_SCALE,
                           interpolation=cv2.INTER_LINEAR)
        gray  = cv2.cvtColor(small, cv2.COLOR_BGR2GRAY)
    else:
        gray  = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    corners, ids, _ = detector.detectMarkers(gray)

    if DETECTION_SCALE < 1.0 and corners:
        corners = [c / DETECTION_SCALE for c in corners]

    # ── Pose estimation & วาดผล ─────────────────────────────
    if ids is not None:
        cv2.aruco.drawDetectedMarkers(frame, corners, ids)

        for i, corner in enumerate(corners):
            marker_id = int(ids[i][0])
            success, rvec, tvec = cv2.solvePnP(
                OBJ_POINTS, corner[0],
                camera_matrix, dist_coeffs,
                flags=cv2.SOLVEPNP_IPPE_SQUARE,
            )
            if not success:
                continue

            tvec_flat = tvec.ravel()
            x, y, z   = float(tvec_flat[0]), float(tvec_flat[1]), float(tvec_flat[2])

            # บันทึกลง history (เฉพาะจุดใหม่เมื่อไม่ paused)
            if not paused:
                history[marker_id]["t"].append(timestamp)
                history[marker_id]["x"].append(x)
                history[marker_id]["y"].append(y)
                history[marker_id]["z"].append(z)

            # วาดแกนและข้อความ
            cv2.drawFrameAxes(frame, camera_matrix, dist_coeffs, rvec, tvec, 0.025)
            center = corner[0].mean(axis=0).astype(np.int32)
            cv2.putText(frame,
                        f"ID:{marker_id}  X:{x:.2f} Y:{y:.2f} Z:{z:.2f}m",
                        (center[0] - 60, center[1] - 15),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

    # ── Overlay ─────────────────────────────────────────────
    ts_text = f"t={timestamp:.2f}s  f={frame_idx}/{total_frames}"
    cv2.rectangle(frame, (5, 5), (320, 32), (0, 0, 0), -1)
    cv2.putText(frame, ts_text, (8, 25),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
    if paused:
        cv2.putText(frame, "PAUSED", (disp_w // 2 - 60, 40),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 3)

    # ── แสดงผล ──────────────────────────────────────────────
    display = cv2.resize(frame, (disp_w, disp_h), interpolation=cv2.INTER_LINEAR)
    cv2.imshow("AR Tag – Video Analysis  (Space/←/→/q)", display)

    # ── Live graph update ───────────────────────────────────
    update_plot()

    # ── รับ input ───────────────────────────────────────────
    key = cv2.waitKey(1 if paused else PLAYBACK_DELAY) & 0xFF
    if key == ord("q"):
        break
    elif key == ord(" "):
        paused = not paused
    elif key == 83 or key == ord("d"):   # → หรือ d
        frame_idx = min(frame_idx + seek_step, total_frames - 1)
        cap.set(cv2.CAP_PROP_POS_FRAMES, frame_idx)
        ret, frame = cap.read()
        if not ret:
            break
    elif key == 81 or key == ord("a"):   # ← หรือ a
        frame_idx = max(frame_idx - seek_step, 0)
        cap.set(cv2.CAP_PROP_POS_FRAMES, frame_idx)
        ret, frame = cap.read()
        if not ret:
            break

# ─────────────────────────────────────────────────────────────
# 9. สรุปผลและแสดงกราฟสุดท้ายแบบ Static
# ─────────────────────────────────────────────────────────────
cap.release()
cv2.destroyAllWindows()

print("\n─── สรุปผลการตรวจจับ ───")
for marker_id, data in sorted(history.items()):
    n = len(data["t"])
    if n == 0:
        continue
    xs, ys, zs = np.array(data["x"]), np.array(data["y"]), np.array(data["z"])
    print(f"  ID {marker_id:>3}:  {n} จุด  |  "
          f"X [{xs.min():.3f}, {xs.max():.3f}]  "
          f"Y [{ys.min():.3f}, {ys.max():.3f}]  "
          f"Z [{zs.min():.3f}, {zs.max():.3f}] m")

# ── วาดกราฟสุดท้าย ────────────────────────────────────────
update_plot(force=True)

# ── วาด 3D Trajectory เพิ่มเติม ───────────────────────────
if history:
    fig3d = plt.figure(figsize=(8, 6))
    ax3d  = fig3d.add_subplot(111, projection="3d")
    ax3d.set_title("3D Trajectory of AR Tags")
    ax3d.set_xlabel("X (m)"); ax3d.set_ylabel("Y (m)"); ax3d.set_zlabel("Z (m)")

    for marker_id, data in history.items():
        if len(data["t"]) < 2:
            continue
        color = get_color(marker_id)
        ax3d.plot(data["x"], data["y"], data["z"],
                  color=color, label=f"ID {marker_id}", linewidth=1.5)
        # จุดเริ่มต้น / จุดสุดท้าย
        ax3d.scatter(data["x"][0],  data["y"][0],  data["z"][0],
                     color=color, marker="o", s=50, zorder=5)
        ax3d.scatter(data["x"][-1], data["y"][-1], data["z"][-1],
                     color=color, marker="^", s=50, zorder=5)

    ax3d.legend()
    fig3d.tight_layout()

plt.ioff()
plt.show()
print("ปิดโปรแกรม")
