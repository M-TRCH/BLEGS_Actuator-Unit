"""
calibration_coverage_check.py  (v2 — multi-video)
──────────────────────────────────────────────────
ประมวลผล 6 วิดีโอพร้อมกัน:

• Pass 1  : scan แต่ละวิดีโอ → เก็บ (frame_no, pt_mm)
           → trim ส่วนที่กลไกอยู่นิ่งช่วงต้น/ท้ายออกอัตโนมัติ
• Pass 2  : เขียน annotated frames เฉพาะช่วง trim → รวมเป็น video เดียว
• Static plot : trajectory ทั้ง 6 ชุดบนกราฟเดียวกัน
"""

import os
import csv

import cv2
import cv2.aruco as aruco
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.colors as mcolors

# ─────────────────────────────────────────
# 1. CONFIG
# ─────────────────────────────────────────
VIDEOS = [
    (r"D:\THESIS\3kg\original.MOV",     "Original",      "dimgray"),
    (r"D:\THESIS\3kg\model_mlp.MOV",    "MLP",           "royalblue"),
    (r"D:\THESIS\3kg\model_poly3.MOV",  "Poly-3",        "darkorange"),
    (r"D:\THESIS\3kg\model_poly4.MOV",  "Poly-4",        "purple"),
    (r"D:\THESIS\3kg\model_forest.MOV", "Random Forest", "green"),
    (r"D:\THESIS\3kg\model_svr.MOV",    "SVR",           "crimson"),
]

CALIB_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                           "output", "params", "calibration_olympus25f1.2.npz")
GRID_CSV   = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                           "output", "data", "workspace_grid.csv")

TARGET_ID  = 4          # ArTag ที่ปลายเท้า (เคลื่อนที่)
FRAME_STEP = 4          # ประมวลผลทุก N เฟรม

# การตัดส่วนนิ่ง
MOTION_THRESHOLD_MM = 2.0   # displacement ต่ำกว่านี้ถือว่านิ่ง (mm)
MOTION_WINDOW       = 15    # rolling window (จำนวน tracked-points)

# พิกัดโลกจริง (mm) ของ motor joints
REF_WORLD = {
    0: np.array([-42.5, 0.0]),
    1: np.array([ 42.5, 0.0]),
}

# วงโคจรอุดมคติ
CENTER_X_MM = 0.0
CENTER_Y_MM = -175.0
RADIUS_MM   = 30.0

# output paths
_VIDEO_DIR = os.path.dirname(VIDEOS[0][0])
OUT_VIDEO  = os.path.join(_VIDEO_DIR, "calibration_coverage_combined.mp4")
OUT_PLOT   = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                           "output", "plots", "calibration_coverage_plot.png")

# ─ Stabilized crop (portrait 9:16) ─
OUT_W            = 720       # output width  (px) — เปลี่ยนได้ตาม resolution
OUT_H            = 1280      # output height (px) — 9:16
CROP_CENTER_MM   = np.array([0.0, -100.0])  # จุดกึ่งกลาง crop ในระนาบ world (mm)
                              #   (0, 0) = กึ่งกลาง id0-id1,  (0,-175) = ศูนย์กลางวงโคจร
STAB_ALPHA       = 0.15      # EMA smoothing (0 = นิ่งสนิท, 1 = ไม่ smooth)
CROP_ZOOM        = 3.0       # zoom-out factor: crop พื้นที่ใหญ่ขึ้น N เท่า แล้ว resize
                              #   1.0 = ไม่ zoom out,  2.0 = เห็นกว้างขึ้น 2x,  3.0 = 3x

# ─────────────────────────────────────────
# 2. สีสำหรับ overlay (คงที่)
# ─────────────────────────────────────────
CLR_GRID_IN   = (0,   200,   0)   # จุด grid ภายในวงโคจร — เขียว
CLR_GRID_OUT  = (255, 180,   0)   # จุด grid นอกวงโคจร  — ส้ม
CLR_IDEAL     = (255, 255, 255)   # ideal circle         — ขาว
CLR_REF       = (255,   0, 255)   # motor joints         — ม่วง

GRID_RADIUS_PX  = 8
TRAJ_THICKNESS  = 2
IDEAL_THICKNESS = 2


# ─────────────────────────────────────────
# 3. Utility helpers
# ─────────────────────────────────────────
def mpl_to_bgr(color_name):
    """แปลง matplotlib color name → BGR tuple สำหรับ OpenCV"""
    r, g, b = mcolors.to_rgb(color_name)
    return (int(b * 255), int(g * 255), int(r * 255))


def load_grid(csv_path):
    pts = []
    with open(csv_path, newline="", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        for row in reader:
            pts.append([float(row["target_x_mm"]), float(row["target_y_mm"])])
    return np.array(pts)


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
    """คืน (px→mm, mm→px); คืน (None, None) ถ้า detect ไม่ได้"""
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

    M      = np.array([[a, b, tx], [b, -a, ty]], dtype=np.float64)
    scale2 = a**2 + b**2

    def px2mm(pt):
        return M @ np.array([float(pt[0]), float(pt[1]), 1.0])

    def mm2px(pt_mm):
        xd = float(pt_mm[0]) - tx
        yd = float(pt_mm[1]) - ty
        return np.array([(a * xd + b * yd) / scale2,
                         (b * xd - a * yd) / scale2])

    return px2mm, mm2px


def in_calib_area(pt_mm, grid_pts, margin=0.0):
    x_min = grid_pts[:, 0].min() - margin
    x_max = grid_pts[:, 0].max() + margin
    y_min = grid_pts[:, 1].min() - margin
    y_max = grid_pts[:, 1].max() + margin
    return (x_min <= pt_mm[0] <= x_max) and (y_min <= pt_mm[1] <= y_max)


def _stabilize_crop(img, stab_mid, stab_angle, cc_orig, frame_w, frame_h):
    """หมุน img ให้ id0→id1 แนวนอน แล้ว crop portrait OUT_W × OUT_H

    - stab_mid   : midpoint px ที่ smooth แล้ว (EMA), หรือ None
    - stab_angle : มุมหมุน (deg) ที่ smooth แล้ว, หรือ None
    - cc_orig    : crop center ใน undistorted frame (px), หรือ None
    """
    if stab_mid is not None and cc_orig is not None:
        # getRotationMatrix2D(center, angle, scale):
        #   หมุน features ของ image ทวนเข็มนาฬิกา (display) ด้วย angle
        #   → ใช้ stab_angle เพื่อ align id0→id1 ให้แนวนอน
        M_rot = cv2.getRotationMatrix2D(
            (float(stab_mid[0]), float(stab_mid[1])),
            float(stab_angle), 1.0,
        )
        # แปลง crop center จาก original → rotated frame
        cc_h     = np.array([float(cc_orig[0]), float(cc_orig[1]), 1.0])
        cc_r     = M_rot @ cc_h
        cx, cy   = float(cc_r[0]), float(cc_r[1])
        rotated  = cv2.warpAffine(img, M_rot, (frame_w, frame_h),
                                   flags=cv2.INTER_LINEAR,
                                   borderMode=cv2.BORDER_CONSTANT,
                                   borderValue=(0, 0, 0))
    else:
        # fallback: ไม่หมุน, crop กึ่งกลาง frame
        cx, cy  = frame_w / 2.0, frame_h / 2.0
        rotated = img

    # crop_w/h เป็นพื้นที่ที่จะตัดจริง (ใหญ่กว่า OUT_W/H ตาม CROP_ZOOM)
    crop_w = int(round(OUT_W * CROP_ZOOM))
    crop_h = int(round(OUT_H * CROP_ZOOM))

    x1 = int(round(cx)) - crop_w // 2
    y1 = int(round(cy)) - crop_h // 2
    x2 = x1 + crop_w
    y2 = y1 + crop_h

    pad_l = max(0, -x1);  pad_t = max(0, -y1)
    pad_r = max(0, x2 - frame_w);  pad_b = max(0, y2 - frame_h)

    if pad_l or pad_t or pad_r or pad_b:
        rotated = cv2.copyMakeBorder(rotated, pad_t, pad_b, pad_l, pad_r,
                                      cv2.BORDER_CONSTANT, value=(0, 0, 0))
        x1 += pad_l;  x2 += pad_l
        y1 += pad_t;  y2 += pad_t

    cropped = rotated[y1:y2, x1:x2]
    if CROP_ZOOM != 1.0:
        cropped = cv2.resize(cropped, (OUT_W, OUT_H), interpolation=cv2.INTER_AREA)
    return cropped


# ─────────────────────────────────────────
# 4. Pass 1 — scan วิดีโอ, เก็บ tracking data
# ─────────────────────────────────────────
def scan_video(video_path, calib_file):
    """อ่านวิดีโอ track TARGET_ID ทุก FRAME_STEP frames
    คืน: list of (frame_no, pt_mm), fps, (w, h)
    """
    print(f"  Scanning: {os.path.basename(video_path)}", flush=True)

    with np.load(calib_file) as data:
        mtx, dist = data["camera_matrix"], data["dist_coeffs"]

    cap = cv2.VideoCapture(video_path)
    if not cap.isOpened():
        raise FileNotFoundError(f"Cannot open: {video_path}")

    fps = cap.get(cv2.CAP_PROP_FPS) or 30.0
    w   = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    h   = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    new_mtx, _ = cv2.getOptimalNewCameraMatrix(mtx, dist, (w, h), 0)

    detector = aruco.ArucoDetector(
        aruco.getPredefinedDictionary(aruco.DICT_6X6_250),
        aruco.DetectorParameters(),
    )

    tracked = []
    fc = 0
    while cap.isOpened():
        ret, frame = cap.read()
        if not ret:
            break
        fc += 1
        if fc % FRAME_STEP != 0:
            continue
        img  = cv2.undistort(frame, mtx, dist, None, new_mtx)
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = detector.detectMarkers(gray)
        centers = _marker_centers(corners, ids)
        px2mm, _ = _build_transform(centers)
        if px2mm is not None and TARGET_ID in centers:
            tracked.append((fc, px2mm(centers[TARGET_ID])))

    cap.release()
    print(f"    → {len(tracked)} points tracked", flush=True)
    return tracked, fps, (w, h)


# ─────────────────────────────────────────
# 5. Trim ส่วนนิ่งช่วงต้น/ท้าย
# ─────────────────────────────────────────
def trim_tracked(tracked):
    """ตัดส่วนที่กลไกอยู่นิ่ง (displacement < MOTION_THRESHOLD_MM)
    ออกจากต้น/ท้ายของ tracked list
    """
    n = len(tracked)
    if n < 2 * MOTION_WINDOW + 1:
        return tracked

    pts  = np.array([t[1] for t in tracked])
    W    = MOTION_WINDOW
    # rolling displacement: pts[i+W] − pts[i]
    disp = np.zeros(n)
    disp[:n - W] = np.linalg.norm(pts[W:] - pts[:n - W], axis=1)

    # หาจุดเริ่ม: index แรกที่เคลื่อนที่เกิน threshold
    start = 0
    for i in range(n):
        if disp[i] > MOTION_THRESHOLD_MM:
            start = i
            break

    # หาจุดสิ้นสุด: index สุดท้ายที่เคลื่อนที่เกิน threshold
    end = n
    for i in range(n - W - 1, -1, -1):
        if disp[i] > MOTION_THRESHOLD_MM:
            end = min(i + W, n)
            break

    trimmed = tracked[start:end]
    if trimmed:
        print(f"    → trim: {n} → {len(trimmed)} pts "
              f"(frame {trimmed[0][0]}–{trimmed[-1][0]})", flush=True)
    else:
        print(f"    → trim: ไม่พบการเคลื่อนที่", flush=True)
    return trimmed


# ─────────────────────────────────────────
# 6. Pass 2 — เขียน annotated frames ลง writer
# ─────────────────────────────────────────
def write_annotated_segment(video_path, calib_file, grid_pts,
                             frame_start, frame_end,
                             label, bgr_color, writer):
    """อ่านวิดีโอเฉพาะช่วง [frame_start, frame_end]
    annotate → align/rotate (id0-id1 แนวนอน) → crop portrait 9:16 → เขียนลง writer
    """
    with np.load(calib_file) as data:
        mtx, dist = data["camera_matrix"], data["dist_coeffs"]

    cap = cv2.VideoCapture(video_path)
    w   = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    h   = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    new_mtx, _ = cv2.getOptimalNewCameraMatrix(mtx, dist, (w, h), 0)

    detector = aruco.ArucoDetector(
        aruco.getPredefinedDictionary(aruco.DICT_6X6_250),
        aruco.DetectorParameters(),
    )

    theta    = np.linspace(0, 2 * np.pi, 200)
    ideal_mm = np.column_stack([
        CENTER_X_MM + RADIUS_MM * np.cos(theta),
        CENTER_Y_MM + RADIUS_MM * np.sin(theta),
    ])

    fc           = 0
    traj_mm_live = []
    written      = 0

    # ── EMA stabilization state ──
    stab_mid     = None   # midpoint px ที่ smooth แล้ว
    stab_angle   = None   # มุมหมุน (deg) ที่ smooth แล้ว
    last_cc_orig = None   # crop center ใน undistorted frame (px)

    while cap.isOpened():
        ret, frame = cap.read()
        if not ret:
            break
        fc += 1
        if fc > frame_end:
            break
        if fc < frame_start or fc % FRAME_STEP != 0:
            continue

        img  = cv2.undistort(frame, mtx, dist, None, new_mtx)
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = detector.detectMarkers(gray)
        centers  = _marker_centers(corners, ids)
        px2mm, mm2px = _build_transform(centers)

        # ── อัปเดต EMA stabilization เมื่อ detect id0, id1 ได้ ──
        if 0 in centers and 1 in centers:
            p0 = centers[0].astype(float)
            p1 = centers[1].astype(float)
            cur_mid   = (p0 + p1) / 2.0
            cur_angle = np.degrees(np.arctan2(
                p1[1] - p0[1], p1[0] - p0[0]
            ))
            if stab_mid is None:
                stab_mid   = cur_mid.copy()
                stab_angle = cur_angle
            else:
                stab_mid  = STAB_ALPHA * cur_mid + (1.0 - STAB_ALPHA) * stab_mid
                da         = (cur_angle - stab_angle + 180.0) % 360.0 - 180.0
                stab_angle = stab_angle + STAB_ALPHA * da

        if mm2px is not None:
            last_cc_orig = mm2px(CROP_CENTER_MM)

        # ── วาด overlays บน undistorted frame (ก่อน crop) ──
        if mm2px is not None:
            # ideal circle
            ideal_px = np.array([mm2px(p) for p in ideal_mm], dtype=np.int32)
            cv2.polylines(img, [ideal_px.reshape(-1, 1, 2)],
                          True, CLR_IDEAL, IDEAL_THICKNESS, cv2.LINE_AA)

            # motor joints
            for mid_id, pos_mm in REF_WORLD.items():
                px = mm2px(pos_mm).astype(int)
                cv2.drawMarker(img, tuple(px), CLR_REF, cv2.MARKER_SQUARE, 14, 2)
                cv2.putText(img, f"id{mid_id}", (px[0] + 8, px[1] - 6),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, CLR_REF, 1, cv2.LINE_AA)

            # calibration grid points
            for gpt in grid_pts:
                gpx = mm2px(gpt).astype(int)
                d2c = np.linalg.norm(gpt - np.array([CENTER_X_MM, CENTER_Y_MM]))
                clr = CLR_GRID_IN if d2c <= RADIUS_MM + 1e-6 else CLR_GRID_OUT
                cv2.circle(img, tuple(gpx), GRID_RADIUS_PX, clr, -1, cv2.LINE_AA)

            # track + draw trajectory
            if TARGET_ID in centers:
                traj_mm_live.append(px2mm(centers[TARGET_ID]))
            if len(traj_mm_live) > 1:
                tpx = np.array([mm2px(p) for p in traj_mm_live], dtype=np.int32)
                cv2.polylines(img, [tpx.reshape(-1, 1, 2)],
                              False, bgr_color, TRAJ_THICKNESS, cv2.LINE_AA)
            if traj_mm_live:
                cur = mm2px(traj_mm_live[-1]).astype(int)
                cv2.circle(img, tuple(cur), 6, bgr_color, -1, cv2.LINE_AA)

        # ── Align (id0→id1 แนวนอน) + crop portrait 9:16 ──
        frame_out = _stabilize_crop(img, stab_mid, stab_angle, last_cc_orig, w, h)

        # ── วาด label banner บน output frame ──
        cv2.rectangle(frame_out, (10, 10), (OUT_W - 10, 52), (30, 30, 30), -1)
        cv2.putText(frame_out, label, (20, 40),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.9, bgr_color, 2, cv2.LINE_AA)

        writer.write(frame_out)
        written += 1

    cap.release()
    print(f"    → wrote {written} frames", flush=True)


# ─────────────────────────────────────────
# 7. Static plot — trajectory ทั้ง 6 ชุด
# ─────────────────────────────────────────
def plot_coverage(grid_pts, results):
    """results: list of (tracked_mm_array, label, mpl_color)"""
    fig, ax = plt.subplots(figsize=(10, 10))

    # bounding box ของ calibration grid
    x_min, x_max = grid_pts[:, 0].min(), grid_pts[:, 0].max()
    y_min, y_max = grid_pts[:, 1].min(), grid_pts[:, 1].max()
    rect = plt.Rectangle(
        (x_min, y_min), x_max - x_min, y_max - y_min,
        linewidth=2, edgecolor="goldenrod", facecolor="lightyellow",
        alpha=0.4, label="Calibration area", zorder=1,
    )
    ax.add_patch(rect)

    # จุด grid แยกสี
    dist_to_center = np.sqrt(
        (grid_pts[:, 0] - CENTER_X_MM)**2 + (grid_pts[:, 1] - CENTER_Y_MM)**2
    )
    inside_orbit = dist_to_center <= RADIUS_MM
    ax.scatter(grid_pts[~inside_orbit, 0], grid_pts[~inside_orbit, 1],
               c="orange", s=50, zorder=3, label="Cal point (outside orbit)")
    ax.scatter(grid_pts[ inside_orbit, 0], grid_pts[ inside_orbit, 1],
               c="limegreen", s=50, zorder=4, label="Cal point (inside orbit)")

    # ideal orbit
    theta   = np.linspace(0, 2 * np.pi, 300)
    ideal_x = CENTER_X_MM + RADIUS_MM * np.cos(theta)
    ideal_y = CENTER_Y_MM + RADIUS_MM * np.sin(theta)
    ax.plot(ideal_x, ideal_y, "k--", linewidth=2, zorder=5,
            label=f"Ideal orbit (R={RADIUS_MM:.0f} mm)")

    # trajectory แต่ละ model
    for tracked_mm, label, color in results:
        if len(tracked_mm) > 0:
            pts = np.array(tracked_mm)
            ax.plot(pts[:, 0], pts[:, 1], "-", color=color,
                    linewidth=1.5, alpha=0.85, label=label, zorder=6)

    # motor joints
    for mid, pos_mm in REF_WORLD.items():
        ax.plot(*pos_mm, "ms", markersize=12, zorder=7)
        ax.annotate(f"id{mid}\n(motor)", pos_mm,
                    textcoords="offset points", xytext=(6, 4),
                    fontsize=8, color="purple")

    # coverage stats (grid)
    n_inside = int(inside_orbit.sum())
    n_total  = len(grid_pts)
    pct      = 100.0 * n_inside / n_total
    stats_text = (
        f"Calibration grid\n"
        f"  X: {x_min:.0f} → {x_max:.0f} mm\n"
        f"  Y: {y_min:.0f} → {y_max:.0f} mm\n"
        f"  Points: {n_total}\n"
        f"  Inside orbit: {n_inside}/{n_total} ({pct:.1f}%)"
    )
    ax.text(0.02, 0.98, stats_text,
            transform=ax.transAxes, fontsize=9, verticalalignment="top",
            fontfamily="monospace",
            bbox=dict(boxstyle="round,pad=0.5", facecolor="white",
                      alpha=0.88, edgecolor="gray"))

    ax.set_title("Calibration Grid Coverage vs. Trajectory Paths (6 models)", fontsize=13)
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

    # ── Pass 1: scan + trim ──────────────────────────
    print("\n=== Pass 1: scanning videos ===")
    all_trimmed = []
    vid_fps  = None
    vid_size = None

    for video_path, label, color in VIDEOS:
        print(f"\n[{label}]")
        tracked, fps, size = scan_video(video_path, CALIB_FILE)
        if vid_fps is None:
            vid_fps  = fps
            vid_size = size
        trimmed = trim_tracked(tracked)
        all_trimmed.append(trimmed)

    # ── Pass 2: เขียน annotated video ─────────────────
    print("\n=== Pass 2: writing annotated video ===")
    os.makedirs(os.path.dirname(OUT_VIDEO), exist_ok=True) if os.path.dirname(OUT_VIDEO) else None
    fourcc  = cv2.VideoWriter_fourcc(*"mp4v")
    out_fps = (vid_fps or 30.0) / FRAME_STEP
    writer  = cv2.VideoWriter(OUT_VIDEO, fourcc, out_fps, (OUT_W, OUT_H))

    results_for_plot = []
    for (video_path, label, color), trimmed in zip(VIDEOS, all_trimmed):
        print(f"\n[{label}]")
        if trimmed:
            bgr = mpl_to_bgr(color)
            write_annotated_segment(
                video_path, CALIB_FILE, grid_pts,
                trimmed[0][0], trimmed[-1][0],
                label, bgr, writer,
            )
            tracked_mm_arr = np.array([t[1] for t in trimmed])
        else:
            print(f"    → ข้ามเนื่องจากไม่มีข้อมูล track", flush=True)
            tracked_mm_arr = np.empty((0, 2))
        results_for_plot.append((tracked_mm_arr, label, color))

    writer.release()
    print(f"\n[OK] Combined video saved: {OUT_VIDEO}")

    # ── Static plot ───────────────────────────────────
    os.makedirs(os.path.dirname(OUT_PLOT), exist_ok=True)
    plot_coverage(grid_pts, results_for_plot)
