"""
5-Bar Linkage ArTag Analysis — Video Version
Author: M-TRCH
Date: May 2026

วิเคราะห์วิดีโอกลไก 5-Bar linkage ทีละเฟรม:
  - ตรวจจับ ArUco markers (ID 0–4) ในแต่ละเฟรม
  - คำนวณพิกัดจริง (mm) ในระบบพิกัดขา
  - ส่งออกวิดีโอ annotated และ CSV รายเฟรม

ระบบพิกัดขา (Leg Frame):
    Origin  = จุดกึ่งกลางระหว่าง Motor A และ Motor B
    x (+)   = ทิศจาก A → B (ขวา)
    y (−)   = ลงจากแนวมอเตอร์ (foot อยู่ที่ y ลบ)

ArUco Marker IDs:
    0 = A  (Motor Left  / ซ้าย)
    1 = B  (Motor Right / ขวา)
    2 = C  (Knee Left   / ข้อเข่าซ้าย)
    3 = D  (Knee Right  / ข้อเข่าขวา)
    4 = E  (End-Effector / ปลายขา)

Link lengths (mm):
    A–C = B–D = 105 mm
    C–E = D–E = 145 mm
"""

import cv2
import cv2.aruco as aruco
import numpy as np
import os
import csv
import sys
import time
import threading
from collections import deque
from concurrent.futures import ThreadPoolExecutor
import queue

_tls = threading.local()   # thread-local storage สำหรับ ArUco detector ต่อ thread

# ============================================================================
# CONFIGURATION
# ============================================================================

VIDEO_INPUT   = r'C:\Users\mteer\OneDrive\Desktop\P1000329.MOV'
SCRIPT_DIR    = os.path.dirname(os.path.abspath(__file__))
_VIDEO_DIR    = os.path.dirname(os.path.abspath(VIDEO_INPUT))
_VIDEO_STEM   = os.path.splitext(os.path.basename(VIDEO_INPUT))[0]
OUTPUT_FOLDER = _VIDEO_DIR                                                   # export ไปโฟลเดอร์เดียวกับวิดีโอต้นฉบับ
CALIB_PATH    = os.path.join(SCRIPT_DIR, 'calibration.npz')
OUTPUT_VIDEO  = os.path.join(OUTPUT_FOLDER, f'{_VIDEO_STEM}_tracked.mp4')
OUTPUT_CSV    = os.path.join(OUTPUT_FOLDER, f'{_VIDEO_STEM}_analysis.csv')

# ─── ตัวเลือกการประมวลผล ───────────────────────────────────────────────────
# ข้ามทุก N เฟรม (1 = ทุกเฟรม, 2 = ทุก 2 เฟรม, ...)
# วิดีโอ 4K 120fps มีขนาดใหญ่ แนะนำ FRAME_SKIP = 1 สำหรับวิเคราะห์เต็ม
# หรือ 2–4 เพื่อความเร็ว
FRAME_SKIP     = 1

# สเกลเอาต์พุต: 1.0 = เต็ม 4K, 0.5 = 2K, 0.25 = 1080p
OUTPUT_SCALE   = 0.5

# ตัดเฉพาะช่วงวิดีโอ (วินาที) — ตั้งเป็น None เพื่อประมวลผลทั้งหมด
START_SEC      = None   # เช่น 5.0
END_SEC        = None   # เช่น 30.0

# Codec สำหรับวิดีโอ output ('mp4v' หรือ 'avc1' หรือ 'XVID')
OUTPUT_CODEC   = 'mp4v'

# ─── ArUco ────────────────────────────────────────────────────────────────
ARUCO_DICT  = aruco.getPredefinedDictionary(aruco.DICT_6X6_250)
PARAMETERS  = aruco.DetectorParameters()

# Joint labels
JOINT_LABEL = {
    0: "A (Motor-L)",
    1: "B (Motor-R)",
    2: "C (Knee-L)",
    3: "D (Knee-R)",
    4: "E (Foot)",
}

# ความยาว link (mm)
LINK_LENGTHS = {
    (0, 2): 105.0,   # A–C
    (1, 3): 105.0,   # B–D
    (2, 4): 145.0,   # C–E
    (3, 4): 145.0,   # D–E
}

LINK_PAIRS = [(0, 2), (1, 3), (2, 4), (3, 4)]

# สี annotation (BGR)
COLOR_LINK      = (0,  220,  0)
COLOR_JOINT     = (0,   0, 220)
COLOR_JOINT_E   = (0, 165, 255)
COLOR_TEXT      = (255, 255,  50)
COLOR_INFO      = (0,  200, 255)
COLOR_TRACK     = (255, 165,   0)   # เส้นเส้นทาง E
COLOR_BG        = (30,   30,  30)   # พื้นหลัง overlay

# จำนวนจุดย้อนหลังที่แสดง trail ของ E
TRAIL_LEN = 120   # ~1 วินาทีที่ 120fps

# ─── ประสิทธิภาพการประมวลผลแบบขนาน ─────────────────────────────────────────
# None = อัตโนมัติ (cpu_count − 1)  |  กำหนดเอง เช่น 4, 6, 8
N_WORKERS      = None
# จำนวนเฟรมที่ queue ล่วงหน้าต่อ worker (None = N_WORKERS × 2)
PREFETCH_DEPTH = None

# ============================================================================
# CAMERA CALIBRATION
# ============================================================================

def load_calibration(npz_path: str):
    """โหลด camera_matrix และ dist_coeffs จากไฟล์ .npz"""
    data = np.load(npz_path)
    keys = list(data.keys())
    mtx_key  = next((k for k in keys if k in ('camera_matrix', 'mtx', 'K', 'cameraMatrix')), None)
    dist_key = next((k for k in keys if k in ('dist_coeffs', 'dist', 'd', 'distCoeffs',
                                               'distortion_coefficients')), None)
    if mtx_key is None or dist_key is None:
        raise KeyError(f'ไม่พบ key ที่ต้องการในไฟล์ .npz\nKey ที่มี: {keys}')
    print(f'✅ โหลดค่าสอบเทียบสำเร็จ  (key: {mtx_key!r}, {dist_key!r})')
    return data[mtx_key], data[dist_key]


# ============================================================================
# GEOMETRY
# ============================================================================

def estimate_scale(detected: dict) -> float | None:
    scales = []
    for (id1, id2), length_mm in LINK_LENGTHS.items():
        if id1 in detected and id2 in detected:
            d_px = np.linalg.norm(detected[id2] - detected[id1])
            if d_px > 1.0:
                scales.append(d_px / length_mm)
    return float(np.mean(scales)) if scales else None


def build_legframe_axes(a_px: np.ndarray, b_px: np.ndarray):
    M_px = (a_px + b_px) / 2.0
    v_AB = b_px - a_px
    v_x  = v_AB / np.linalg.norm(v_AB)
    v_y_down = np.array([-v_x[1], v_x[0]])
    return M_px, v_x, v_y_down


def px_to_legframe_mm(point_px, origin_px, v_x, v_y_down, px_per_mm):
    d    = point_px - origin_px
    x_mm = float(np.dot(d, v_x)       / px_per_mm)
    y_mm = float(-np.dot(d, v_y_down) / px_per_mm)
    return x_mm, y_mm


# ============================================================================
# FRAME PROCESSING
# ============================================================================

def process_frame(
    frame: np.ndarray,
    frame_idx: int,
    timestamp_sec: float,
    camera_matrix,
    dist_coeffs,
    new_mtx,
    roi,
    trail_snapshot: list,   # snapshot ณ เวลา submit (read-only, ไม่แก้ไข)
    out_w: int = 0,         # ขนาด output ที่ต้องการ (0 = ใช้ขนาดเต็มจาก undistort+crop)
    out_h: int = 0,         # ขนาด output ที่ต้องการ (0 = ใช้ขนาดเต็มจาก undistort+crop)
) -> tuple:
    """
    ประมวลผล 1 เฟรม — thread-safe
    คืน (frame_annotated, row_dict)
    """
    # สร้าง detector ต่อ thread (thread-local) — ป้องกัน race condition
    if not hasattr(_tls, 'detector'):
        _tls.detector = aruco.ArucoDetector(ARUCO_DICT, PARAMETERS)
    detector = _tls.detector
    # ─── แก้ความเพี้ยนเลนส์ ────────────────────────────────────────────
    img_u = cv2.undistort(frame, camera_matrix, dist_coeffs, None, new_mtx)
    rx, ry, rw, rh = roi
    if rw > 0 and rh > 0:
        img_u = img_u[ry:ry+rh, rx:rx+rw]

    # ─── ตรวจจับ ArUco ─────────────────────────────────────────────────
    gray = cv2.cvtColor(img_u, cv2.COLOR_BGR2GRAY)
    corners, ids, _ = detector.detectMarkers(gray)

    detected: dict[int, np.ndarray] = {}
    if ids is not None:
        for i, id_arr in enumerate(ids):
            id_val = int(id_arr[0])
            if id_val in JOINT_LABEL:
                c  = corners[i][0]
                cx = float(np.mean(c[:, 0]))
                cy = float(np.mean(c[:, 1]))
                detected[id_val] = np.array([cx, cy])

    n_det     = len(detected)
    px_per_mm = estimate_scale(detected)

    # ─── สร้าง result dict ─────────────────────────────────────────────
    row: dict = {
        'frame':         frame_idx,
        'time_sec':      round(timestamp_sec, 4),
        'n_detected':    n_det,
        'px_per_mm':     round(px_per_mm, 4) if px_per_mm else None,
        'ab_dist_mm':    None,
        'detected_x_mm': None,
        'detected_y_mm': None,
    }

    # พิกัด pixel
    for jid in range(5):
        row[f'px_id{jid}_x'] = round(detected[jid][0], 1) if jid in detected else None
        row[f'px_id{jid}_y'] = round(detected[jid][1], 1) if jid in detected else None

    # พิกัด leg-frame
    for jid in range(5):
        row[f'lf_id{jid}_x_mm'] = None
        row[f'lf_id{jid}_y_mm'] = None

    lf_available = False
    M_px = v_x = v_y_down = None

    if 0 in detected and 1 in detected and px_per_mm:
        M_px, v_x, v_y_down = build_legframe_axes(detected[0], detected[1])
        for jid, pt in detected.items():
            x_mm, y_mm = px_to_legframe_mm(pt, M_px, v_x, v_y_down, px_per_mm)
            row[f'lf_id{jid}_x_mm'] = round(x_mm, 2)
            row[f'lf_id{jid}_y_mm'] = round(y_mm, 2)
        lf_available = True

        a_x, a_y = row['lf_id0_x_mm'], row['lf_id0_y_mm']
        b_x, b_y = row['lf_id1_x_mm'], row['lf_id1_y_mm']
        row['ab_dist_mm'] = round(np.hypot(b_x - a_x, b_y - a_y), 2)

    if lf_available and 4 in detected:
        row['detected_x_mm'] = row['lf_id4_x_mm']
        row['detected_y_mm'] = row['lf_id4_y_mm']

    # ─── Scale เฟรมและพิกัดไปยังขนาด output ก่อน annotate ───────────────
    src_h_u, src_w_u = img_u.shape[:2]
    _ow = out_w if out_w > 0 else src_w_u
    _oh = out_h if out_h > 0 else src_h_u
    if _ow != src_w_u or _oh != src_h_u:
        _sx    = _ow / src_w_u
        _sy    = _oh / src_h_u
        img_u  = cv2.resize(img_u, (_ow, _oh), interpolation=cv2.INTER_AREA)
    else:
        _sx, _sy = 1.0, 1.0
    # พิกัด pixel สำหรับ draw (scale ลงมาพร้อม image)
    det_draw   = {jid: np.array([pt[0] * _sx, pt[1] * _sy]) for jid, pt in detected.items()}
    trail_draw = [(int(x * _sx), int(y * _sy)) for (x, y) in trail_snapshot]

    # ─── Annotate frame ────────────────────────────────────────────────
    scale_f  = max(1, _ow // 1920)   # 1 @ ≤1920px, 2 @ 4K
    lw       = 2 * scale_f
    cr       = 10 * scale_f
    ft       = 0.5 * scale_f
    ft_thick = max(1, 1 * scale_f)

    # เส้น trail ของ E (ใช้ snapshot ที่ส่งมา — ไม่แก้ไข shared state)
    if len(trail_draw) >= 2:
        n_trail = len(trail_draw)
        for ti in range(1, n_trail):
            alpha = ti / n_trail
            color = (
                int(COLOR_TRACK[0] * alpha),
                int(COLOR_TRACK[1] * alpha),
                int(COLOR_TRACK[2] * alpha),
            )
            cv2.line(img_u, trail_draw[ti - 1], trail_draw[ti], color, max(1, lw))

    # เส้น link
    for p1_id, p2_id in LINK_PAIRS:
        if p1_id in det_draw and p2_id in det_draw:
            pt1 = tuple(det_draw[p1_id].astype(int))
            pt2 = tuple(det_draw[p2_id].astype(int))
            cv2.line(img_u, pt1, pt2, COLOR_LINK, lw + 2)
            d_mm = LINK_LENGTHS.get((p1_id, p2_id))
            mid  = ((det_draw[p1_id] + det_draw[p2_id]) / 2).astype(int)
            cv2.putText(img_u, f'{d_mm:.0f}mm', tuple(mid + np.array([5, -8])),
                        cv2.FONT_HERSHEY_SIMPLEX, ft * 0.75, (200, 255, 200), ft_thick)

    # จุด joint
    for jid, pos in det_draw.items():
        pt    = tuple(pos.astype(int))
        color = COLOR_JOINT_E if jid == 4 else COLOR_JOINT
        cv2.circle(img_u, pt, cr, color, -1)
        cv2.circle(img_u, pt, cr, (255, 255, 255), max(1, lw - 1))

        x_lf = row.get(f'lf_id{jid}_x_mm')
        y_lf = row.get(f'lf_id{jid}_y_mm')
        if x_lf is not None:
            label = f'{JOINT_LABEL[jid]}  ({x_lf:+.1f}, {y_lf:+.1f}) mm'
        else:
            label = JOINT_LABEL[jid]
        cv2.putText(img_u, label, (pt[0] + cr + 5, pt[1] - cr),
                    cv2.FONT_HERSHEY_SIMPLEX, ft, COLOR_TEXT, ft_thick)

    # ─── HUD (info overlay ด้านบน) ─────────────────────────────────────
    hud_lines = [
        f'Frame: {frame_idx:6d}   Time: {timestamp_sec:7.3f} s',
        f'Detected: {n_det}/5   Scale: '
        + (f'{px_per_mm:.3f} px/mm' if px_per_mm else 'N/A'),
    ]
    if lf_available and 4 in detected:
        hud_lines.append(
            f'E pos: ({row["detected_x_mm"]:+7.2f}, {row["detected_y_mm"]:+7.2f}) mm'
        )
    if row['ab_dist_mm'] is not None:
        hud_lines.append(f'A-B dist: {row["ab_dist_mm"]:.2f} mm')

    pad    = 10 * scale_f
    line_h = 30 * scale_f
    box_h  = pad * 2 + line_h * len(hud_lines)
    box_w  = 520 * scale_f

    # พื้นหลัง semi-transparent
    overlay = img_u.copy()
    cv2.rectangle(overlay, (0, 0), (box_w, box_h), COLOR_BG, -1)
    cv2.addWeighted(overlay, 0.55, img_u, 0.45, 0, img_u)

    for li, txt in enumerate(hud_lines):
        y_pos = pad + (li + 1) * line_h - 5
        cv2.putText(img_u, txt, (pad, y_pos),
                    cv2.FONT_HERSHEY_SIMPLEX, ft * 1.0, COLOR_INFO, ft_thick + 1)

    return img_u, row


# ============================================================================
# CSV
# ============================================================================

CSV_COLUMNS = [
    'frame', 'time_sec',
    'n_detected', 'px_per_mm', 'ab_dist_mm',
    'detected_x_mm', 'detected_y_mm',
    'px_id0_x', 'px_id0_y',
    'px_id1_x', 'px_id1_y',
    'px_id2_x', 'px_id2_y',
    'px_id3_x', 'px_id3_y',
    'px_id4_x', 'px_id4_y',
    'lf_id0_x_mm', 'lf_id0_y_mm',
    'lf_id1_x_mm', 'lf_id1_y_mm',
    'lf_id2_x_mm', 'lf_id2_y_mm',
    'lf_id3_x_mm', 'lf_id3_y_mm',
    'lf_id4_x_mm', 'lf_id4_y_mm',
]


def save_csv(rows: list, csv_path: str):
    with open(csv_path, 'w', newline='', encoding='utf-8-sig') as f:
        writer = csv.DictWriter(f, fieldnames=CSV_COLUMNS, extrasaction='ignore')
        writer.writeheader()
        for row in rows:
            writer.writerow({k: ('' if row.get(k) is None else row[k]) for k in CSV_COLUMNS})
    print(f'\n💾 บันทึก CSV: {os.path.abspath(csv_path)}  ({len(rows)} แถว)')


# ============================================================================
# MAIN
# ============================================================================

def process_video():
    print('=' * 65)
    print('  5-Bar Linkage ArTag Analysis — Video')
    print('=' * 65)

    # ─── โหลดค่าสอบเทียบ ───────────────────────────────────────────────
    try:
        camera_matrix, dist_coeffs = load_calibration(CALIB_PATH)
    except FileNotFoundError:
        print(f'❌ ไม่พบไฟล์สอบเทียบ: {CALIB_PATH}')
        return
    except KeyError as e:
        print(f'❌ {e}')
        return

    # ─── เปิดวิดีโอ ────────────────────────────────────────────────────
    cap = cv2.VideoCapture(VIDEO_INPUT)
    if not cap.isOpened():
        print(f'❌ เปิดไฟล์วิดีโอไม่ได้: {VIDEO_INPUT}')
        return

    total_frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    fps          = cap.get(cv2.CAP_PROP_FPS)
    src_w        = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    src_h        = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

    print(f'\n📹 วิดีโอ : {os.path.basename(VIDEO_INPUT)}')
    print(f'   ขนาด  : {src_w} × {src_h}  ({src_w/1920:.1f}K)')
    print(f'   FPS   : {fps:.2f}')
    print(f'   เฟรม  : {total_frames:,}')
    print(f'   ความยาว: {total_frames/fps:.1f} วินาที')

    # ─── ตัดช่วง ────────────────────────────────────────────────────────
    start_frame = int(START_SEC * fps) if START_SEC is not None else 0
    end_frame   = int(END_SEC   * fps) if END_SEC   is not None else total_frames - 1
    end_frame   = min(end_frame, total_frames - 1)

    if start_frame > 0:
        cap.set(cv2.CAP_PROP_POS_FRAMES, start_frame)
        print(f'   ตัดช่วง: เฟรม {start_frame} – {end_frame}')

    proc_frames = (end_frame - start_frame) // FRAME_SKIP + 1
    print(f'   เฟรมที่ประมวลผล: {proc_frames:,}  (skip={FRAME_SKIP})')

    # ─── คำนวณ new_mtx / roi จากเฟรมแรก เพื่อไม่ต้องทำซ้ำทุกเฟรม ──────
    ret0, frame0 = cap.read()
    if not ret0:
        print('❌ อ่านเฟรมแรกไม่ได้')
        cap.release()
        return
    h0, w0 = frame0.shape[:2]
    new_mtx, roi = cv2.getOptimalNewCameraMatrix(
        camera_matrix, dist_coeffs, (w0, h0), alpha=0)

    # คำนวณขนาด output จริงหลัง undistort crop
    rx, ry, rw, rh = roi
    out_src_w = rw if rw > 0 else w0
    out_src_h = rh if rh > 0 else h0
    out_w = max(1, int(out_src_w * OUTPUT_SCALE))
    out_h = max(1, int(out_src_h * OUTPUT_SCALE))

    # ─── VideoWriter с codec auto-selection ───────────────────────────
    os.makedirs(OUTPUT_FOLDER, exist_ok=True)
    # ปัด fps ไปค่า standard ที่ codec รองรับ (ป้องกัน fractional/VFR fps)
    _std_fps    = [24, 25, 30, 48, 50, 60, 90, 100, 120, 240]
    out_fps_raw = fps / FRAME_SKIP
    _near       = min(_std_fps, key=lambda x: abs(x - out_fps_raw))
    out_fps     = float(_near) if abs(_near - out_fps_raw) / max(out_fps_raw, 1e-6) < 0.02 \
                  else round(out_fps_raw, 3)

    # ลำดับ codec: high-fps → ลอง avc1 ก่อน (รองรับ 120fps ดีกว่า mp4v)
    def _try_codec(codec: str) -> cv2.VideoWriter | None:
        w = cv2.VideoWriter(OUTPUT_VIDEO,
                            cv2.VideoWriter_fourcc(*codec),
                            out_fps, (out_w, out_h))
        return w if w.isOpened() else None

    _codec_order = ([OUTPUT_CODEC] if OUTPUT_CODEC != 'mp4v' or out_fps <= 60
                    else ['avc1', 'mp4v'])
    vwriter    = None
    used_codec = OUTPUT_CODEC
    for _c in _codec_order:
        vwriter = _try_codec(_c)
        if vwriter is not None:
            used_codec = _c
            break
    if vwriter is None:
        print(f'❌ VideoWriter เปิดไม่ได้ (ลอง codec: {_codec_order}, fps={out_fps}, size={out_w}×{out_h})')
        cap.release()
        return

    print(f'\n🎬 เอาต์พุตวิดีโอ : {os.path.basename(OUTPUT_VIDEO)}')
    print(f'   ขนาด output  : {out_w} × {out_h}  (scale={OUTPUT_SCALE})')
    print(f'   Codec        : {used_codec}  FPS={out_fps:.2f}')
    if used_codec != OUTPUT_CODEC:
        print(f'   ℹ️  เปลี่ยน codec จาก {OUTPUT_CODEC} → {used_codec} อัตโนมัติ (รองรับ {out_fps:.0f}fps)')
    print()

    rows: list[dict] = []
    trail_pts: list  = []

    n_workers      = N_WORKERS      if N_WORKERS      is not None else max(2, (os.cpu_count() or 4) - 1)
    prefetch_depth = PREFETCH_DEPTH if PREFETCH_DEPTH is not None else n_workers * 2
    print(f'   Workers      : {n_workers}   prefetch={prefetch_depth}')
    print()

    written      = 0
    t_start      = time.perf_counter()
    t_last_log   = t_start
    LOG_INTERVAL = 30.0
    BAR_WIDTH    = 28

    # ─── Reader thread: อ่านเฟรมแบบ sequential ใน thread แยก ──────────────
    # แยก I/O (cap.read/grab) ออกจาก main thread เพื่อไม่ให้บล็อก writer
    raw_q: queue.Queue = queue.Queue(maxsize=prefetch_depth + n_workers)

    def _reader() -> None:
        """อ่าน/skip เฟรมจาก VideoCapture ทีละเฟรมตามลำดับอย่างเคร่งครัด"""
        cap.set(cv2.CAP_PROP_POS_FRAMES, start_frame)   # reset สู่จุดเริ่ม
        idx        = start_frame
        err_streak = 0
        while idx <= end_frame:
            if (idx - start_frame) % FRAME_SKIP == 0:
                ret, raw = cap.read()
                if ret:
                    err_streak = 0
                    raw_q.put((idx, raw))   # บล็อกถ้า queue เต็ม (backpressure)
                else:
                    err_streak += 1
                    if err_streak >= 5:     # EOF หรือ error ต่อเนื่องหลายเฟรม
                        break
                    # ลอง seek ข้ามเฟรมที่ decode ไม่ได้
                    cap.set(cv2.CAP_PROP_POS_FRAMES, idx + 1)
            else:
                if not cap.grab():  # grab ไม่ต้อง decode — เร็วกว่า read
                    break           # EOF
            idx += 1
        raw_q.put(None)   # sentinel: อ่านเสร็จแล้ว

    rdr = threading.Thread(target=_reader, daemon=True, name='frame-reader')
    rdr.start()

    # ─── Pipeline: pool workers → ordered future_deque → writer ──────────
    future_deque: deque = deque()   # (frame_idx, Future) เรียงตามลำดับ submit
    reading_done = False

    def _fill_pool() -> None:
        """pop raw_q แล้ว submit ไป pool จนเต็ม prefetch_depth"""
        nonlocal reading_done
        while not reading_done and len(future_deque) < prefetch_depth:
            try:
                item = raw_q.get(timeout=10.0)
            except queue.Empty:
                print('\n⚠️  Reader timeout — หยุดรอเฟรม')
                reading_done = True
                return
            if item is None:        # sentinel
                reading_done = True
                return
            fidx, raw = item
            snap = list(trail_pts)  # snapshot trail ณ เวลา submit
            fut  = pool.submit(
                process_frame, raw, fidx, fidx / fps,
                camera_matrix, dist_coeffs, new_mtx, roi, snap,
                out_w, out_h,
            )
            future_deque.append((fidx, fut))

    with ThreadPoolExecutor(max_workers=n_workers) as pool:
        _fill_pool()   # เติม pipeline ครั้งแรก

        while future_deque:
            fidx, fut      = future_deque.popleft()
            annotated, row = fut.result()   # รอผลตามลำดับเฟรม (ordered)

            # ─── อัปเดต trail ตามลำดับ — main thread เท่านั้น ───────────
            px4_x = row.get('px_id4_x')
            px4_y = row.get('px_id4_y')
            if px4_x is not None:
                trail_pts.append((int(px4_x), int(px4_y)))
                if len(trail_pts) > TRAIL_LEN:
                    trail_pts.pop(0)

            # ─── เขียน VideoWriter ตามลำดับ (frame ถูก resize ใน process_frame) ──
            vwriter.write(annotated)
            rows.append(row)
            written += 1

            # ─── inline progress bar ─────────────────────────────────────
            elapsed  = time.perf_counter() - t_start
            pct      = (fidx - start_frame) / max(1, end_frame - start_frame) * 100
            filled   = int(BAR_WIDTH * pct / 100)
            bar      = '█' * filled + '░' * (BAR_WIDTH - filled)
            fps_proc = written / max(elapsed, 1e-6)
            eta_sec  = (elapsed / max(written, 1)) * max(0, proc_frames - written)
            n_det    = row['n_detected']
            sys.stdout.write(
                f'\r  [{bar}] {pct:5.1f}%  '
                f'Fr {fidx}/{end_frame}  '
                f'{fps_proc:.1f}fps  '
                f'ETA {int(eta_sec//60):02d}:{int(eta_sec%60):02d}  '
                f'det {n_det}/5  '
            )
            sys.stdout.flush()

            # ─── checkpoint log ทุก LOG_INTERVAL วินาทีจริง ─────────────
            t_now = time.perf_counter()
            if t_now - t_last_log >= LOG_INTERVAL or fidx == end_frame:
                det_pct = sum(1 for r in rows if r['n_detected'] > 0) / max(1, written) * 100
                print(f'\n  ✔ {pct:5.1f}%  เฟรม {fidx}/{end_frame}  '
                      f'เขียน {written:,}  '
                      f'{fps_proc:.1f} fps proc  '
                      f'ETA {int(eta_sec//60):02d}:{int(eta_sec%60):02d}  '
                      f'det {det_pct:.0f}%')
                t_last_log = t_now

            # ─── เติม pool ───────────────────────────────────────────────
            _fill_pool()

    rdr.join(timeout=5.0)
    sys.stdout.write('\n')

    cap.release()
    vwriter.release()

    # ─── บันทึก CSV ─────────────────────────────────────────────────────
    if rows:
        save_csv(rows, OUTPUT_CSV)

    # ─── สรุปผล ─────────────────────────────────────────────────────────
    elapsed_total = time.perf_counter() - t_start
    detected_e    = [r for r in rows if r.get('detected_x_mm') is not None]
    det_any       = [r for r in rows if r['n_detected'] > 0]

    print(f'\n{"="*65}')
    print(f'  สรุปผล')
    print(f'{"="*65}')
    print(f'  เฟรมที่ประมวลผล       : {len(rows):,}  (คาดหวัง {proc_frames:,})')
    print(f'  ตรวจจับ marker ≥1    : {len(det_any):,}  ({len(det_any)/max(1,len(rows))*100:.1f}%)')
    print(f'  ตรวจจับ E (ID 4) ได้  : {len(detected_e):,}  ({len(detected_e)/max(1,len(rows))*100:.1f}%)')
    if detected_e:
        xs = [r['detected_x_mm'] for r in detected_e]
        ys = [r['detected_y_mm'] for r in detected_e]
        print(f'  E x range  : {min(xs):+.2f} ~ {max(xs):+.2f} mm')
        print(f'  E y range  : {min(ys):+.2f} ~ {max(ys):+.2f} mm')
    print(f'  เวลาประมวลผลรวม      : {elapsed_total:.1f} s')
    print(f'  วิดีโอ output        : {os.path.abspath(OUTPUT_VIDEO)}')
    print(f'  CSV                  : {os.path.abspath(OUTPUT_CSV)}')
    print('=' * 65)


if __name__ == '__main__':
    process_video()
