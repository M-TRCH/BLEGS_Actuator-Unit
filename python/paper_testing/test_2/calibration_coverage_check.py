"""
calibration_coverage_check.py
──────────────────────────────
ตรวจสอบว่า path ที่ทดสอบ (circular trajectory) ครอบคลุมอยู่ภายใน
พื้นที่ calibration grid หรือไม่

• โหลด calibration_grid.csv → world-space mm
• แทร็กวิดีโอด้วย ArUco (เหมือน vision_based_trajectory_eval.py)
• บันทึกวิดีโอ annotated (calibration grid + trajectory ทับบน frame)
• พล็อตกราฟ 2-D เปรียบเทียบ calibration area vs. tracked path
"""

import os
import csv

import cv2
import cv2.aruco as aruco
import numpy as np
import matplotlib
matplotlib.use("Agg")          # ป้องกัน GUI แฮงก์เวลา save ก่อน show
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from matplotlib.patches import Polygon as MplPolygon
from matplotlib.collections import PatchCollection

# ─────────────────────────────────────────
# 1. CONFIG  (แก้ค่าที่นี่)
# ─────────────────────────────────────────
VIDEO_PATH  = r"C:\Users\mteer\OneDrive\Desktop\comp_28_0_-190_5.MOV"
CALIB_FILE  = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                            "calibration_olympus25mm.npz")
GRID_CSV    = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                            "calibration_grid.csv")

TARGET_ID   = 4          # ArTag ที่ปลายเท้า (เคลื่อนที่)
FRAME_STEP  = 3         # ประมวลผลทุก N เฟรม

# พิกัดโลกจริง (mm) ของ motor joints (id0, id1) — ตรึงที่
REF_WORLD = {
    0: np.array([-42.5, 0.0]),
    1: np.array([ 42.5, 0.0]),
}

# วงโคจรอุดมคติ
CENTER_X_MM = 0.0
CENTER_Y_MM = -190.0
RADIUS_MM   = 28.0

# output paths
OUT_VIDEO   = os.path.join(os.path.dirname(os.path.abspath(VIDEO_PATH)),
                            "calibration_coverage_video.mp4")
OUT_PLOT    = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                            "calibration_coverage_plot.png")

# ─────────────────────────────────────────
# 2. โหลด calibration grid
# ─────────────────────────────────────────
def load_grid(csv_path):
    pts = []
    with open(csv_path, newline="", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        for row in reader:
            pts.append([float(row["target_x_mm"]), float(row["target_y_mm"])])
    return np.array(pts)          # shape (N, 2)

# ─────────────────────────────────────────
# 3. ArUco helpers  (ยืมจาก vision_based_trajectory_eval.py)
# ─────────────────────────────────────────
def _marker_centers(corners, ids):
    result = {}
    if ids is None:
        return result
    for i, mid in enumerate(ids.flatten()):
        c = corners[i][0]
        result[int(mid)] = np.array(
            [np.mean(c[:, 0]), np.mean(c[:, 1])], dtype=np.float32
        )
    return result


def _build_transform(centers):
    """คืน (px→mm, mm→px) tuple; คืน (None, None) ถ้า detect ไม่ได้"""
    if 0 not in centers or 1 not in centers:
        return None, None

    p0_px = centers[0].astype(np.float64)
    p1_px = centers[1].astype(np.float64)
    p0_mm = REF_WORLD[0].astype(np.float64)
    p1_mm = REF_WORLD[1].astype(np.float64)

    dx_px = p1_px[0] - p0_px[0];  dy_px = p1_px[1] - p0_px[1]
    dx_mm = p1_mm[0] - p0_mm[0];  dy_mm = p1_mm[1] - p0_mm[1]

    denom = dx_px**2 + dy_px**2
    a  = (dx_mm * dx_px - dy_mm * dy_px) / denom
    b  = (dx_mm * dy_px + dy_mm * dx_px) / denom
    tx = p0_mm[0] - a * p0_px[0] - b * p0_px[1]
    ty = p0_mm[1] - b * p0_px[0] + a * p0_px[1]

    M = np.array([[a,  b, tx],
                  [b, -a, ty]], dtype=np.float64)

    def px2mm(pt):
        return M @ np.array([float(pt[0]), float(pt[1]), 1.0])

    # inverse: solve M_2x2 * [x_px, y_px]^T = [x_mm-tx, y_mm-ty]^T
    # M_2x2 = [[a,b],[b,-a]],  det = -(a²+b²)
    scale2 = a**2 + b**2

    def mm2px(pt_mm):
        xd = float(pt_mm[0]) - tx
        yd = float(pt_mm[1]) - ty
        x_px = (a * xd + b * yd) / scale2
        y_px = (b * xd - a * yd) / scale2
        return np.array([x_px, y_px])

    return px2mm, mm2px


# ─────────────────────────────────────────
# 4. สีสำหรับ overlay
# ─────────────────────────────────────────
CLR_GRID_IN   = (0,   200,   0)    # จุด grid ที่อยู่ในวงโคจร — เขียว
CLR_GRID_OUT  = (255, 180,   0)    # จุด grid ที่อยู่นอกวงโคจร — ส้ม
CLR_TRAJ      = (0,   80,  255)    # trajectory — น้ำเงิน
CLR_IDEAL     = (255, 255, 255)    # ideal circle — ขาว
CLR_REF       = (255,   0,  255)   # motor joints — ม่วง

GRID_RADIUS_PX  = 8    # ขนาดจุด grid ใน video overlay
TRAJ_THICKNESS  = 2
IDEAL_THICKNESS = 2


# ─────────────────────────────────────────
# 5. ฟังก์ชันคำนวณ: ใน calibration area ไหม?
# ─────────────────────────────────────────
def in_calib_area(pt_mm, grid_pts, margin=0.0):
    """True ถ้าจุด pt_mm อยู่ภายใน bounding box ของ grid (+ margin)"""
    x_min, x_max = grid_pts[:, 0].min() - margin, grid_pts[:, 0].max() + margin
    y_min, y_max = grid_pts[:, 1].min() - margin, grid_pts[:, 1].max() + margin
    return (x_min <= pt_mm[0] <= x_max) and (y_min <= pt_mm[1] <= y_max)


# ─────────────────────────────────────────
# 6. วิดีโอ annotated + แทร็กพร้อมกัน
# ─────────────────────────────────────────
def process_video(video_path, calib_file, grid_pts):
    print(f"Opening video: {video_path}")

    with np.load(calib_file) as data:
        mtx, dist = data["camera_matrix"], data["dist_coeffs"]

    cap = cv2.VideoCapture(video_path)
    if not cap.isOpened():
        raise FileNotFoundError(f"Cannot open: {video_path}")

    fps  = cap.get(cv2.CAP_PROP_FPS) or 30.0
    w    = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    h    = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    new_mtx, _ = cv2.getOptimalNewCameraMatrix(mtx, dist, (w, h), 0)

    fourcc = cv2.VideoWriter_fourcc(*"mp4v")
    writer = cv2.VideoWriter(OUT_VIDEO, fourcc, fps / FRAME_STEP, (w, h))

    detector = aruco.ArucoDetector(
        aruco.getPredefinedDictionary(aruco.DICT_6X6_250),
        aruco.DetectorParameters(),
    )

    # วงโคจรอุดมคติ (ใน mm) สำหรับ overlay
    theta     = np.linspace(0, 2 * np.pi, 200)
    ideal_mm  = np.column_stack([
        CENTER_X_MM + RADIUS_MM * np.cos(theta),
        CENTER_Y_MM + RADIUS_MM * np.sin(theta),
    ])

    tracked_mm = []
    frame_count = 0
    skipped     = 0

    while cap.isOpened():
        ret, frame = cap.read()
        if not ret:
            break
        frame_count += 1
        if frame_count % FRAME_STEP != 0:
            continue

        img = cv2.undistort(frame, mtx, dist, None, new_mtx)
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = detector.detectMarkers(gray)

        centers        = _marker_centers(corners, ids)
        px2mm, mm2px   = _build_transform(centers)

        if mm2px is not None:
            # ─ วาด ideal circle
            ideal_px = np.array([mm2px(p) for p in ideal_mm], dtype=np.int32)
            cv2.polylines(img, [ideal_px.reshape(-1, 1, 2)],
                          isClosed=True, color=CLR_IDEAL,
                          thickness=IDEAL_THICKNESS, lineType=cv2.LINE_AA)

            # ─ วาด motor joints
            for mid, pos_mm in REF_WORLD.items():
                px = mm2px(pos_mm).astype(int)
                cv2.drawMarker(img, tuple(px), CLR_REF,
                               cv2.MARKER_SQUARE, 14, 2)
                cv2.putText(img, f"id{mid}", (px[0]+8, px[1]-6),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, CLR_REF, 1, cv2.LINE_AA)

            # ─ วาด calibration grid points
            for gpt in grid_pts:
                gpx = mm2px(gpt).astype(int)
                inside = in_calib_area(gpt, grid_pts)   # ทุกจุด grid อยู่ใน grid เองอยู่แล้ว
                # แยกสีตาม: อยู่ใน 'วงโคจรอุดมคติ' หรือเปล่า?
                dist_to_center = np.linalg.norm(gpt - np.array([CENTER_X_MM, CENTER_Y_MM]))
                clr = CLR_GRID_IN if dist_to_center <= RADIUS_MM + 1e-6 else CLR_GRID_OUT
                cv2.circle(img, tuple(gpx), GRID_RADIUS_PX, clr, -1, cv2.LINE_AA)

            # ─ วาด trajectory สะสม
            if TARGET_ID in centers and px2mm is not None:
                pt_mm = px2mm(centers[TARGET_ID])
                tracked_mm.append(pt_mm)
            else:
                skipped += 1

            if len(tracked_mm) > 1:
                traj_px = np.array([mm2px(p) for p in tracked_mm], dtype=np.int32)
                cv2.polylines(img, [traj_px.reshape(-1, 1, 2)],
                              isClosed=False, color=CLR_TRAJ,
                              thickness=TRAJ_THICKNESS, lineType=cv2.LINE_AA)

            # ─ วาดจุดปัจจุบัน
            if tracked_mm:
                cur_px = mm2px(tracked_mm[-1]).astype(int)
                cv2.circle(img, tuple(cur_px), 6, CLR_TRAJ, -1, cv2.LINE_AA)

        else:
            skipped += 1

        # ─ Legend ใน video
        cv2.rectangle(img, (10, 10), (280, 120), (30, 30, 30), -1)
        cv2.putText(img, "CAL COVERAGE CHECK", (15, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (230,230,230), 1, cv2.LINE_AA)
        cv2.circle(img, (25, 50), 7, CLR_IDEAL, -1)
        cv2.putText(img, "Ideal circle", (38, 55),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (230,230,230), 1, cv2.LINE_AA)
        cv2.circle(img, (25, 70), 7, CLR_GRID_IN, -1)
        cv2.putText(img, "Grid pt (inside orbit)", (38, 75),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (230,230,230), 1, cv2.LINE_AA)
        cv2.circle(img, (25, 90), 7, CLR_GRID_OUT, -1)
        cv2.putText(img, "Grid pt (outside orbit)", (38, 95),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (230,230,230), 1, cv2.LINE_AA)
        cv2.circle(img, (25, 110), 7, CLR_TRAJ, -1)
        cv2.putText(img, f"Tracked ({len(tracked_mm)} pts)", (38, 115),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (230,230,230), 1, cv2.LINE_AA)

        writer.write(img)

        if frame_count % 30 == 0:
            processed = frame_count // FRAME_STEP
            print(f"  Frame {frame_count} | tracked {len(tracked_mm)} | skipped {skipped}",
                  end="\r", flush=True)

    cap.release()
    writer.release()
    print(f"\n[OK] Tracked {len(tracked_mm)} points.  Saved: {OUT_VIDEO}")
    return np.array(tracked_mm) if tracked_mm else np.empty((0, 2))


# ─────────────────────────────────────────
# 7. Static plot: calibration area vs trajectory
# ─────────────────────────────────────────
def plot_coverage(grid_pts, tracked_mm):
    fig, ax = plt.subplots(figsize=(9, 9))

    # ─ bounding box ของ calibration grid
    x_min, x_max = grid_pts[:, 0].min(), grid_pts[:, 0].max()
    y_min, y_max = grid_pts[:, 1].min(), grid_pts[:, 1].max()
    rect = plt.Rectangle(
        (x_min, y_min), x_max - x_min, y_max - y_min,
        linewidth=2, edgecolor="goldenrod", facecolor="lightyellow",
        alpha=0.4, label="Calibration area (bounding box)",
        zorder=1,
    )
    ax.add_patch(rect)

    # ─ จุด grid แยกสี
    dist_to_center = np.sqrt(
        (grid_pts[:, 0] - CENTER_X_MM)**2 + (grid_pts[:, 1] - CENTER_Y_MM)**2
    )
    inside_orbit = dist_to_center <= RADIUS_MM
    ax.scatter(grid_pts[~inside_orbit, 0], grid_pts[~inside_orbit, 1],
               c="orange", s=60, zorder=3, label="Cal point (outside orbit)")
    ax.scatter(grid_pts[ inside_orbit, 0], grid_pts[ inside_orbit, 1],
               c="limegreen", s=60, zorder=4, label="Cal point (inside orbit)")

    # ─ วงโคจรอุดมคติ
    theta   = np.linspace(0, 2 * np.pi, 300)
    ideal_x = CENTER_X_MM + RADIUS_MM * np.cos(theta)
    ideal_y = CENTER_Y_MM + RADIUS_MM * np.sin(theta)
    ax.plot(ideal_x, ideal_y, "k--", linewidth=2, zorder=5, label=f"Ideal orbit (R={RADIUS_MM:.0f} mm)")

    # ─ tracked trajectory
    if len(tracked_mm) > 0:
        ax.scatter(tracked_mm[:, 0], tracked_mm[:, 1],
                   c="steelblue", s=8, alpha=0.5, zorder=6, label="Tracked trajectory")

    # ─ motor joints
    for mid, pos_mm in REF_WORLD.items():
        ax.plot(*pos_mm, "ms", markersize=12, zorder=7)
        ax.annotate(f"id{mid}\n(motor)", pos_mm,
                    textcoords="offset points", xytext=(6, 4),
                    fontsize=8, color="purple")

    # ─ coverage stats
    n_inside = int(inside_orbit.sum())
    n_total  = len(grid_pts)
    pct      = 100.0 * n_inside / n_total

    if len(tracked_mm) > 0:
        in_area = np.array([in_calib_area(p, grid_pts) for p in tracked_mm])
        traj_pct = 100.0 * in_area.sum() / len(tracked_mm)
    else:
        traj_pct = float("nan")

    stats_text = (
        f"Calibration grid\n"
        f"  X: {x_min:.0f} → {x_max:.0f} mm\n"
        f"  Y: {y_min:.0f} → {y_max:.0f} mm\n"
        f"  Points: {n_total}\n"
        f"  Inside orbit: {n_inside}/{n_total} ({pct:.1f}%)\n\n"
        f"Trajectory\n"
        f"  Points tracked: {len(tracked_mm)}\n"
        f"  Inside cal area: {traj_pct:.1f}%"
    )
    ax.text(0.02, 0.98, stats_text,
            transform=ax.transAxes, fontsize=9, verticalalignment="top",
            fontfamily="monospace",
            bbox=dict(boxstyle="round,pad=0.5", facecolor="white",
                      alpha=0.88, edgecolor="gray"))

    ax.set_title("Calibration Grid Coverage vs. Trajectory Path", fontsize=13)
    ax.set_xlabel("X (mm)")
    ax.set_ylabel("Y (mm)")
    ax.set_aspect("equal")
    ax.grid(True, linestyle=":", alpha=0.6)
    ax.legend(loc="lower right", fontsize=9)

    plt.tight_layout()
    plt.savefig(OUT_PLOT, dpi=300)
    print(f"[OK] Static plot saved: {OUT_PLOT}")
    plt.show()


# ─────────────────────────────────────────
# 8. MAIN
# ─────────────────────────────────────────
if __name__ == "__main__":
    grid_pts = load_grid(GRID_CSV)
    print(f"Loaded {len(grid_pts)} calibration grid points")
    print(f"  X range: {grid_pts[:,0].min()} → {grid_pts[:,0].max()} mm")
    print(f"  Y range: {grid_pts[:,1].min()} → {grid_pts[:,1].max()} mm")

    tracked_mm = process_video(VIDEO_PATH, CALIB_FILE, grid_pts)

    plot_coverage(grid_pts, tracked_mm)
