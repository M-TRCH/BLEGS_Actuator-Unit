"""
Video 5-Bar Linkage Grid Analysis — 49 Point Tracking
Author: M-TRCH
Date: May 2026

อ่านวิดีโอที่แสดงการเคลื่อนที่ของปลายขาครบ 49 จุด
ตรวจจับตำแหน่งปลายขา E จาก ArUco marker ID=4
เปรียบเทียบกับ target จาก grid_log.csv
แล้วเพิ่มคอลัมน์ผลลัพธ์เข้า grid_log.csv โดยตรง

วิธีระบุจุดทดสอบ:
    วิดีโอเริ่มจาก Home position → กลไกเคลื่อนไปจุดที่ 1 → 2 → ... → 49
    ตรวจจับช่วงที่ E หยุดนิ่ง (dwell) แล้วเทียบตำแหน่งกับ target[1] ใน grid_log.csv
    เพื่อข้าม Home/setup dwells และเริ่มนับจากจุดทดสอบจริงจุดแรก

ระบบพิกัดขา (Leg Frame):
    Origin = กึ่งกลางระหว่าง Motor A และ Motor B
    x (+)  = ทิศ A → B (ขวา)
    y (−)  = ลงจากแนวมอเตอร์ (foot อยู่ที่ y ลบ)

ArUco IDs:
    0 = Motor A  (fixed: −42.5, 0 mm)
    1 = Motor B  (fixed: +42.5, 0 mm)
    4 = End-effector E  (target ที่ track)
"""

import cv2
import cv2.aruco as aruco
import numpy as np
import os
import csv

# ============================================================================
# CONFIGURATION  ← ปรับที่นี่ก่อนรัน
# ============================================================================

VIDEO_PATH    = r"C:\Users\mteer\OneDrive\Desktop\grid_49point.MOV"
GRID_LOG_CSV  = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'grid_log.csv')
CALIB_PATH    = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'calibration_olympus25mm.npz')
OUTPUT_VIDEO  = os.path.join(os.path.dirname(os.path.abspath(VIDEO_PATH)), 'video_tracked.mp4')

# Fixed motor positions in leg frame (mm)
MOTOR_SPACING = 85.0
P_A_LF = np.array([-42.5, 0.0])   # Motor A (ArUco ID 0)
P_B_LF = np.array([+42.5, 0.0])   # Motor B (ArUco ID 1)

N_POINTS = 49   # จำนวนจุดทดสอบ

# ─── Stability detection ─────────────────────────────────────────────────────
# ถ้า detect segment ได้ไม่ครบ 49:
#   น้อยเกินไป → ลด STABLE_VEL_MM_S หรือลด MIN_STABLE_FRAMES
#   มากเกินไป  → เพิ่ม STABLE_VEL_MM_S หรือเพิ่ม MIN_STABLE_FRAMES

STABLE_VEL_MM_S    = 5.0    # mm/s  ต่ำกว่านี้ถือว่า E "หยุดนิ่ง"
MIN_STABLE_FRAMES  = 15     # จำนวน frame ต่อเนื่องขั้นต่ำของแต่ละ dwell
DWELL_SAMPLE_FRAC  = 0.6    # เลือก sample จาก fraction ท้ายของ dwell (motor settle แล้ว)
SKIP_INITIAL_FRAMES = 5     # ข้าม frame แรกเริ่มสุด (กันช่วง setup)
MAX_FRAME_GAP      = 10     # gap สูงสุดระหว่าง frame ที่ตรวจจับ E ได้ (px velocity)
MERGE_GAP_FRAMES   = 5      # ยอมรับ gap ใน stable segment ได้ไม่เกินนี้ (frame)

# ─── Sequence-start detection ────────────────────────────────────────────────
# เปรียบเทียบ dwell แรกๆ กับ target จุดที่ 1 จาก grid_log.csv
# เพื่อข้ามช่วง Home หรือ setup ที่อยู่ก่อนจุดทดสอบจริง

HOME_MATCH_TOL_MM  = 15.0   # mm — ระยะ tolerance เมื่อจับคู่ dwell กับ target จุดที่ 1
SEQ_MATCH_TOL_MM   = 20.0   # mm — tolerance สำหรับ greedy matching จุดที่ 2–49
MAX_SKIP_DWELLS    = 8      # จำนวน dwell สูงสุดที่ข้ามได้เพื่อหา target ถัดไป

# ─── Manual labeling ─────────────────────────────────────────────────────────
MANUAL_MODE          = True  # True → ดูวิดีโอแล้วกด Space เพื่อ mark จุดด้วยตนเอง
MANUAL_WINDOW_FRAMES = 5      # เฉลี่ย E position จาก ±N frames รอบจุดที่ mark

# ─── ArUco ───────────────────────────────────────────────────────────────────
ARUCO_DICT = aruco.getPredefinedDictionary(aruco.DICT_6X6_250)
PARAMETERS = aruco.DetectorParameters()


# ============================================================================
# HELPERS
# ============================================================================

def load_calibration(path: str):
    """โหลด camera_matrix, dist_coeffs จาก .npz  (คืน None, None ถ้าไม่พบ)"""
    try:
        data = np.load(path)
        keys = list(data.keys())
        mk = next((k for k in keys if k in
                   ('camera_matrix', 'mtx', 'K', 'cameraMatrix')), None)
        dk = next((k for k in keys if k in
                   ('dist_coeffs', 'dist', 'd', 'distCoeffs',
                    'distortion_coefficients')), None)
        if mk and dk:
            print(f'✅ Calibration โหลดสำเร็จ  ({mk!r}, {dk!r})')
            return data[mk].astype(np.float64), data[dk].astype(np.float64)
        print(f'⚠️  ไม่พบ key ที่ต้องการใน {path}  keys={keys}')
    except FileNotFoundError:
        print(f'⚠️  ไม่พบ {path} — ข้ามการแก้ distortion')
    return None, None


def build_undistort_maps(camera_matrix, dist_coeffs, w, h):
    """สร้าง undistort maps สำหรับ cv2.remap"""
    if camera_matrix is None:
        return None, None, None
    new_mtx, _ = cv2.getOptimalNewCameraMatrix(
        camera_matrix, dist_coeffs, (w, h), alpha=0)
    m1, m2 = cv2.initUndistortRectifyMap(
        camera_matrix, dist_coeffs, None, new_mtx, (w, h), cv2.CV_16SC2)
    return m1, m2, new_mtx


def detect_markers(gray, detector) -> dict[int, np.ndarray]:
    """ตรวจจับ ArUco → {id: center_px (float)}"""
    corners, ids, _ = detector.detectMarkers(gray)
    out = {}
    if ids is not None:
        for i, id_arr in enumerate(ids):
            c = corners[i][0]
            out[int(id_arr[0])] = np.array(
                [float(np.mean(c[:, 0])), float(np.mean(c[:, 1]))])
    return out


def build_transform(a_px: np.ndarray, b_px: np.ndarray):
    """
    สร้าง transform จากตำแหน่ง pixel ของ Motor A และ B

    คืน (origin_px, v_x, v_y_down, px_per_mm)
        origin_px  : จุดกึ่งกลาง A–B (pixel)
        v_x        : unit vector ทิศ +x (A→B)
        v_y_down   : unit vector ทิศลงในภาพ (= ทิศ y−ใน leg frame)
        px_per_mm  : scale
    """
    origin = (a_px + b_px) / 2.0
    v_AB   = b_px - a_px
    dist   = np.linalg.norm(v_AB)
    vx     = v_AB / dist
    vy_dn  = np.array([-vx[1], vx[0]])   # 90° CCW ใน image space → ลง
    ppmm   = dist / MOTOR_SPACING
    return origin, vx, vy_dn, ppmm


def px_to_legframe(pt_px, origin, vx, vy_dn, ppmm) -> tuple[float, float]:
    """pixel → (x_mm, y_mm) ในระบบพิกัดขา"""
    d = pt_px - origin
    return (float(np.dot(d, vx) / ppmm),
            float(-np.dot(d, vy_dn) / ppmm))


# ============================================================================
# PHASE 1: SCAN VIDEO
# ============================================================================

def scan_video(video_path, camera_matrix, dist_coeffs, detector,
               show_preview: bool = False):
    """
    อ่านทุก frame, ตรวจจับ markers → list ของ per-frame dicts + transform

    Returns:
        frames      : list[dict]  — {frame_idx, e_px, a_px, b_px}
        transform   : (origin, vx, vy_dn, ppmm)
        fps         : float
    """
    cap = cv2.VideoCapture(video_path)
    if not cap.isOpened():
        raise FileNotFoundError(f'ไม่สามารถเปิดวิดีโอ: {video_path}')

    total = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    fps   = cap.get(cv2.CAP_PROP_FPS)
    w     = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    h     = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    print(f'📹 {os.path.basename(video_path)}')
    print(f'   {total} frames  |  {fps:.2f} fps  |  {w}×{h}  |  {total/fps:.1f} s')

    # Build undistort maps once
    map1, map2, _ = build_undistort_maps(camera_matrix, dist_coeffs, w, h)
    if map1 is not None:
        print('   Undistort maps พร้อม')

    SCAN_WIN = 'Scanning... (กำลังสแกนวิดีโอ — กรุณารอ)'
    if show_preview:
        prev_w = min(w, 960)
        prev_h = int(h * prev_w / w)
        cv2.namedWindow(SCAN_WIN, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(SCAN_WIN, prev_w, prev_h)
        print('   🖥️  เปิดหน้าต่าง preview ระหว่างสแกน...')

    frames       = []
    motor_a_all  = []
    motor_b_all  = []
    n_E = n_A = n_B = 0

    print('⏳ สแกนวิดีโอ...')
    for fi in range(total):
        ret, img = cap.read()
        if not ret:
            break

        if fi % 300 == 0:
            pct = fi / total * 100
            print(f'   [{pct:5.1f}%] frame {fi}/{total}', end='\r', flush=True)

        if map1 is not None:
            img = cv2.remap(img, map1, map2, cv2.INTER_LINEAR)

        # แสดง preview ทุก 30 frame ระหว่างสแกน
        if show_preview and fi % 30 == 0:
            pv = img.copy()
            cv2.putText(pv, f'Scanning {fi}/{total} ({fi/total*100:.0f}%)  — กรุณารอ...',
                        (10, 42), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 230, 0), 2)
            cv2.imshow(SCAN_WIN, pv)
            cv2.waitKey(1)

        gray     = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        detected = detect_markers(gray, detector)

        entry = {
            'frame_idx': fi,
            'e_px':      detected.get(4),
            'a_px':      detected.get(0),
            'b_px':      detected.get(1),
        }
        frames.append(entry)

        if entry['a_px'] is not None:
            motor_a_all.append(entry['a_px']); n_A += 1
        if entry['b_px'] is not None:
            motor_b_all.append(entry['b_px']); n_B += 1
        if entry['e_px'] is not None:
            n_E += 1

    cap.release()
    if show_preview:
        cv2.destroyWindow(SCAN_WIN)
    print(f'\n   เสร็จ: {len(frames)} frames  |  '
          f'E={n_E}  A={n_A}  B={n_B} frames detected')

    if not motor_a_all or not motor_b_all:
        raise RuntimeError(
            'ตรวจจับ Motor A (ID 0) หรือ Motor B (ID 1) ไม่ได้เลย\n'
            '→ ตรวจสอบ ArUco marker และ ARUCO_DICT')

    a_med = np.median(motor_a_all, axis=0)
    b_med = np.median(motor_b_all, axis=0)
    transform = build_transform(a_med, b_med)
    origin, vx, vy_dn, ppmm = transform

    # sanity check: motor A ใน leg frame ควรใกล้ (-42.5, 0)
    a_lf = px_to_legframe(a_med, origin, vx, vy_dn, ppmm)
    b_lf = px_to_legframe(b_med, origin, vx, vy_dn, ppmm)
    print(f'   Scale     : {ppmm:.3f} px/mm')
    print(f'   Motor A   : pixel ({a_med[0]:.0f}, {a_med[1]:.0f})'
          f'  →  leg ({a_lf[0]:+.2f}, {a_lf[1]:+.2f}) mm  [expect (-42.50, 0.00)]')
    print(f'   Motor B   : pixel ({b_med[0]:.0f}, {b_med[1]:.0f})'
          f'  →  leg ({b_lf[0]:+.2f}, {b_lf[1]:+.2f}) mm  [expect (+42.50, 0.00)]')

    return frames, transform, fps


# ============================================================================
# PHASE 2: DETECT DWELL SEGMENTS
# ============================================================================

def detect_dwells(
    frames,
    transform,
    fps: float,
    stable_vel_mm_s: float = STABLE_VEL_MM_S,
    min_stable_frames: int  = MIN_STABLE_FRAMES,
    skip_initial: int       = SKIP_INITIAL_FRAMES,
    max_gap: int            = MAX_FRAME_GAP,
    merge_gap: int          = MERGE_GAP_FRAMES,
) -> tuple[list[tuple[int, int]], dict[int, np.ndarray]]:
    """
    หาช่วงที่ E หยุดนิ่ง (dwell) โดยวัด velocity ระหว่าง frame ที่ตรวจจับ E ได้

    Logic:
        1. คำนวณ velocity (px/frame) ระหว่าง consecutive E detections
        2. frame ที่ vel ≤ threshold → stable
        3. Merge stable frames ที่ใกล้กัน (gap ≤ merge_gap)
        4. กรองเอาเฉพาะ segment ที่ยาว ≥ min_stable_frames

    Returns:
        segments    : list[(frame_start, frame_end)]  — เรียงตามเวลา
        e_by_frame  : dict{frame_idx: e_px}
    """
    _, _, _, ppmm = transform
    vel_thresh_px = (stable_vel_mm_s * ppmm) / fps
    print(f'   vel threshold = {stable_vel_mm_s} mm/s'
          f' × {ppmm:.2f} px/mm / {fps:.1f} fps'
          f' = {vel_thresh_px:.3f} px/frame')

    # ─── สร้าง E time series ──────────────────────────────────────────
    e_track = [
        (d['frame_idx'], d['e_px'])
        for d in frames
        if d['e_px'] is not None and d['frame_idx'] >= skip_initial
    ]
    e_by_frame = {fi: pt for fi, pt in e_track}

    if len(e_track) < min_stable_frames * 2:
        print(f'   ⚠️  E ตรวจพบเพียง {len(e_track)} frames — น้อยเกินไป')
        return [], e_by_frame

    # ─── คำนวณ velocity ───────────────────────────────────────────────
    vel_map: dict[int, float] = {}
    for i in range(1, len(e_track)):
        fi_p, pt_p = e_track[i - 1]
        fi_c, pt_c = e_track[i]
        dt = fi_c - fi_p
        if 0 < dt <= max_gap:
            vel_map[fi_c] = float(np.linalg.norm(pt_c - pt_p) / dt)
        else:
            # gap ใหญ่ = ไม่สามารถประเมิน velocity ได้ → ถือว่า moving
            vel_map[fi_c] = float('inf')

    stable_set = {fi for fi, v in vel_map.items() if v <= vel_thresh_px}

    # ─── Merge เป็น segments ──────────────────────────────────────────
    raw_segs: list[tuple[int, int]] = []
    sorted_stable = sorted(stable_set)
    if not sorted_stable:
        print(f'   ⚠️  ไม่พบ stable frames  (threshold {vel_thresh_px:.2f} px/frame)')
        return [], e_by_frame

    s0 = e0 = sorted_stable[0]
    for fi in sorted_stable[1:]:
        if fi - e0 <= merge_gap:
            e0 = fi
        else:
            raw_segs.append((s0, e0))
            s0 = e0 = fi
    raw_segs.append((s0, e0))

    # ─── กรอง min duration ────────────────────────────────────────────
    segments = [(s, e) for s, e in raw_segs if (e - s + 1) >= min_stable_frames]

    n_stable_frames = len(stable_set)
    print(f'   stable frames : {n_stable_frames}/{len(e_track)}  '
          f'({n_stable_frames/len(e_track)*100:.1f}%)')
    print(f'   raw segments  : {len(raw_segs)}'
          f'  →  after min_duration filter: {len(segments)}')

    return segments, e_by_frame


# ============================================================================
# PHASE 3: EXTRACT REPRESENTATIVE E POSITION PER DWELL
# ============================================================================

def extract_positions(
    segments: list[tuple[int, int]],
    e_by_frame: dict[int, np.ndarray],
    transform,
    fps: float,
    sample_frac: float = DWELL_SAMPLE_FRAC,
) -> list[dict]:
    """
    ดึง E position ที่ตัวแทนของแต่ละ dwell
    เลือก sample จาก fraction ท้ายของ segment (หลัง motor settle)

    Returns:
        list of dict: {point_id, frame_start, frame_end, duration_s,
                       e_px_median, actual_x_mm, actual_y_mm}
    """
    origin, vx, vy_dn, ppmm = transform
    results = []

    for pid_0, (seg_start, seg_end) in enumerate(segments):
        # ดึง E ใน segment
        in_seg = [
            (fi, e_by_frame[fi])
            for fi in range(seg_start, seg_end + 1)
            if fi in e_by_frame
        ]
        if not in_seg:
            continue

        # เลือก sample จาก fraction ท้าย (motor settle แล้ว)
        cut = int(seg_start + (seg_end - seg_start) * sample_frac)
        late = [p for p in in_seg if p[0] >= cut] or in_seg
        pts  = np.array([p[1] for p in late])
        med  = np.median(pts, axis=0)

        x_mm, y_mm = px_to_legframe(med, origin, vx, vy_dn, ppmm)
        results.append({
            'point_id':    pid_0 + 1,
            'frame_start': seg_start,
            'frame_end':   seg_end,
            'duration_s':  round((seg_end - seg_start + 1) / fps, 2),
            'e_px_median': med,
            'actual_x_mm': round(x_mm, 3),
            'actual_y_mm': round(y_mm, 3),
        })

    return results


# ============================================================================
# PHASE 4: UPDATE grid_log.csv
# ============================================================================

NEW_COLS = [
    'video_act_x_mm',
    'video_act_y_mm',
    'video_err_x_mm',
    'video_err_y_mm',
    'video_err_dist_mm',
]


def load_targets(csv_path: str) -> list[dict]:
    """อ่าน target_x_mm, target_y_mm จาก grid_log.csv → list[{target_x_mm, target_y_mm}]"""
    targets = []
    try:
        with open(csv_path, newline='', encoding='utf-8') as f:
            reader = csv.DictReader(f)
            for row in reader:
                targets.append({
                    'target_x_mm': float(row['target_x_mm']),
                    'target_y_mm': float(row['target_y_mm']),
                })
        print(f'   โหลด targets: {len(targets)} จุด จาก {os.path.basename(csv_path)}')
    except FileNotFoundError:
        print(f'   ⚠️  ไม่พบ {csv_path} — ไม่สามารถค้นหาจุดเริ่มต้น sequence ได้')
    except KeyError as exc:
        print(f'   ⚠️  ไม่พบคอลัมน์ {exc} ใน CSV — ตรวจสอบ header ของ grid_log.csv')
    return targets


def find_sequence_start(
    dwell_results: list[dict],
    targets: list[dict],
    tol_mm: float = HOME_MATCH_TOL_MM,
) -> int:
    """
    ค้นหา dwell แรกที่ตรงกับ target จุดที่ 1 (ข้าม Home / setup dwells)

    Logic:
        วนหา dwell ตัวแรก (เรียงตามเวลา) ที่ห่างจาก target[0] ไม่เกิน tol_mm
        ถ้าไม่มีใดอยู่ใน tolerance → เลือก dwell ที่ใกล้ที่สุด (พร้อมแจ้งเตือน)

    Returns:
        int — index ใน dwell_results ที่เริ่มต้น sequence จริง
    """
    if not targets or not dwell_results:
        return 0

    tx = targets[0]['target_x_mm']
    ty = targets[0]['target_y_mm']

    best_idx  = 0
    best_dist = float('inf')

    print(f'   ค้นหา dwell ที่ตรงกับ target[1] = ({tx:+.2f}, {ty:+.2f}) mm')
    print(f'   tolerance = {tol_mm} mm')

    for i, d in enumerate(dwell_results):
        dist = float(np.hypot(d['actual_x_mm'] - tx, d['actual_y_mm'] - ty))
        if dist < best_dist:
            best_dist = dist
            best_idx  = i
        if dist <= tol_mm:
            print(f'   ✅ พบจุดที่ 1 ที่ dwell #{i + 1}  '
                  f'dist={dist:.2f} mm  '
                  f'actual=({d["actual_x_mm"]:+.2f}, {d["actual_y_mm"]:+.2f}) mm')
            return i

    # ไม่มี dwell ใดอยู่ใน tolerance → ใช้ closest พร้อมแจ้งเตือน
    d = dwell_results[best_idx]
    print(f'   ⚠️  ไม่พบ dwell ที่อยู่ใน {tol_mm} mm ของ target[1]')
    print(f'      closest = dwell #{best_idx + 1}  '
          f'dist={best_dist:.2f} mm  '
          f'actual=({d["actual_x_mm"]:+.2f}, {d["actual_y_mm"]:+.2f}) mm')
    print(f'      → ใช้ dwell ที่ใกล้ที่สุด (เพิ่ม HOME_MATCH_TOL_MM หากต้องการ)')
    return best_idx


def match_dwells_to_targets(
    dwell_results: list[dict],
    targets: list[dict],
    start_idx: int,
    tol_mm: float = SEQ_MATCH_TOL_MM,
    max_skip: int = MAX_SKIP_DWELLS,
) -> list[dict]:
    """
    Greedy forward matching — จับคู่ dwell กับ target แต่ละจุดตามลำดับ

    Algorithm:
        สำหรับแต่ละ target[i]:
            สแกน dwell จาก dwell_ptr ไปข้างหน้าไม่เกิน max_skip ตัว
            เลือก dwell ที่ใกล้ target[i] ที่สุดและอยู่ใน tol_mm
            ถ้าพบ → บันทึก, เลื่อน dwell_ptr ไปถัดจาก dwell นั้น
            ถ้าไม่พบ → ข้าม target นี้ (mark missing), ไม่เลื่อน ptr

    Returns:
        list[dict] — dwells ที่ match แล้ว พร้อม point_id ตรงกับ target index
    """
    matched   = []
    dwell_ptr = start_idx
    n_dwells  = len(dwell_results)
    n_missing = 0

    print(f'   Greedy matching: {len(targets)} targets  |  '
          f'{n_dwells - start_idx} dwells ที่เหลือ  |  '
          f'tol={tol_mm} mm  max_skip={max_skip}')

    for t_idx, target in enumerate(targets):
        if dwell_ptr >= n_dwells:
            print(f'   ⚠️  target {t_idx + 1}–{len(targets)}: หมด dwell แล้ว')
            n_missing += len(targets) - t_idx
            break

        tx = target['target_x_mm']
        ty = target['target_y_mm']

        best_i    = None
        best_dist = float('inf')
        limit     = min(dwell_ptr + max_skip, n_dwells)

        for j in range(dwell_ptr, limit):
            d    = dwell_results[j]
            dist = float(np.hypot(d['actual_x_mm'] - tx,
                                  d['actual_y_mm'] - ty))
            if dist < best_dist:
                best_dist = dist
                best_i    = j

        if best_i is not None and best_dist <= tol_mm:
            n_skipped = best_i - dwell_ptr
            if n_skipped:
                print(f'   ℹ️  target {t_idx + 1:2d}: ข้าม {n_skipped} spurious dwell')
            entry             = dwell_results[best_i].copy()
            entry['point_id'] = t_idx + 1
            matched.append(entry)
            dwell_ptr = best_i + 1
        else:
            closest_str = (f'  closest={best_dist:.1f} mm @ dwell #{best_i + 1}'
                           if best_i is not None else '')
            print(f'   ⚠️  target {t_idx + 1:2d} ({tx:+6.1f},{ty:+6.1f}): '
                  f'ไม่พบ dwell ใน {tol_mm} mm{closest_str}')
            n_missing += 1
            # ไม่เลื่อน dwell_ptr → dwell ถัดไปยังมีโอกาส match target หน้า

    print(f'   Match สำเร็จ: {len(matched)}/{len(targets)}  '
          f'(missing={n_missing})')
    return matched


def update_grid_log(
    csv_path: str,
    dwell_results: list[dict],
) -> list[float]:
    """
    อ่าน grid_log.csv → เพิ่มคอลัมน์ใหม่ → เขียนกลับ (in-place)

    Columns เพิ่ม:
        video_act_x_mm    — ตำแหน่ง E จริง (x) จากวิดีโอ (mm)
        video_act_y_mm    — ตำแหน่ง E จริง (y) จากวิดีโอ (mm)
        video_err_x_mm    — error x = act − target (mm)
        video_err_y_mm    — error y = act − target (mm)
        video_err_dist_mm — Euclidean error distance (mm)

    Returns:
        list ของ error_dist_mm สำหรับแถวที่ match
    """
    with open(csv_path, newline='', encoding='utf-8') as f:
        reader = csv.DictReader(f)
        orig_fields = list(reader.fieldnames)
        rows = list(reader)

    # เพิ่ม field ถ้ายังไม่มี
    out_fields = orig_fields.copy()
    for col in NEW_COLS:
        if col not in out_fields:
            out_fields.append(col)

    # เคลียร์คอลัมน์ใหม่ทุกแถว (รองรับการรันซ้ำ)
    for row in rows:
        for col in NEW_COLS:
            row[col] = ''

    n_match = min(len(dwell_results), len(rows))
    errors: list[float] = []

    for i in range(n_match):
        row   = rows[i]
        dwell = dwell_results[i]
        ax    = dwell['actual_x_mm']
        ay    = dwell['actual_y_mm']
        tx    = float(row['target_x_mm'])
        ty    = float(row['target_y_mm'])
        ex    = round(ax - tx, 3)
        ey    = round(ay - ty, 3)
        ed    = round(float(np.hypot(ex, ey)), 3)

        row['video_act_x_mm']   = f'{ax}'
        row['video_act_y_mm']   = f'{ay}'
        row['video_err_x_mm']   = f'{ex}'
        row['video_err_y_mm']   = f'{ey}'
        row['video_err_dist_mm'] = f'{ed}'
        errors.append(ed)

    with open(csv_path, 'w', newline='', encoding='utf-8') as f:
        writer = csv.DictWriter(f, fieldnames=out_fields, extrasaction='ignore')
        writer.writeheader()
        writer.writerows(rows)

    if len(dwell_results) != len(rows):
        print(f'   ⚠️  match {n_match}/{len(rows)} แถว  '
              f'(dwell={len(dwell_results)}, csv={len(rows)})')
    return errors


# ============================================================================
# PHASE 5: ANNOTATED VIDEO (optional)
# ============================================================================

def save_annotated_video(
    video_path: str,
    frames: list[dict],
    transform,
    fps: float,
    dwell_results: list[dict],
    out_path: str,
    camera_matrix=None,
    dist_coeffs=None,
):
    """
    สร้างวิดีโอ annotated แสดง:
    - Trajectory trail ของ E (สี gradient)
    - วงกลมที่ตำแหน่ง E ปัจจุบัน
    - หมายเลขจุดทดสอบระหว่าง dwell
    - ตำแหน่ง leg-frame (mm) ที่ตำแหน่ง E
    """
    origin, vx, vy_dn, ppmm = transform
    origin_int = origin.astype(int)

    cap = cv2.VideoCapture(video_path)
    w   = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    h   = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    writer = cv2.VideoWriter(out_path, fourcc, fps, (w, h))

    # สร้าง undistort maps ถ้ามี
    map1, map2, _ = build_undistort_maps(camera_matrix, dist_coeffs, w, h)

    # map frame_idx → dwell point_id
    seg_to_pid: dict[int, int] = {}
    for d in dwell_results:
        for fi in range(d['frame_start'], d['frame_end'] + 1):
            seg_to_pid[fi] = d['point_id']

    # map frame_idx → e_px
    e_map = {
        d['frame_idx']: d['e_px']
        for d in frames
        if d['e_px'] is not None
    }

    TRAIL_LEN = int(fps * 2.0)
    total = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    fi = 0

    while True:
        ret, img = cap.read()
        if not ret:
            break

        if map1 is not None:
            img = cv2.remap(img, map1, map2, cv2.INTER_LINEAR)

        # ─── Trajectory trail ────────────────────────────────────────
        trail = [
            e_map[j].astype(int)
            for j in range(max(0, fi - TRAIL_LEN), fi + 1)
            if j in e_map
        ]
        for k in range(1, len(trail)):
            alpha = k / len(trail)
            col = (0, int(200 * alpha), int(255 * (1 - alpha)))
            cv2.line(img, tuple(trail[k - 1]), tuple(trail[k]), col, 2)

        # ─── E marker ────────────────────────────────────────────────
        if fi in e_map:
            pt     = tuple(e_map[fi].astype(int))
            x_mm, y_mm = px_to_legframe(e_map[fi], origin, vx, vy_dn, ppmm)
            cv2.circle(img, pt, 10, (0, 165, 255), -1)
            cv2.circle(img, pt, 10, (255, 255, 255), 2)
            cv2.putText(img, f'({x_mm:+.1f}, {y_mm:+.1f}) mm',
                        (pt[0] + 14, pt[1] - 8),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 50), 2)

        # ─── Motor origin ─────────────────────────────────────────────
        cv2.circle(img, tuple(origin_int), 5, (200, 100, 0), -1)

        # ─── Point ID overlay ────────────────────────────────────────
        pid = seg_to_pid.get(fi)
        if pid is not None:
            cv2.putText(img, f'Point {pid}/{N_POINTS}', (30, 65),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.8, (0, 255, 0), 3)

        # ─── Frame counter ───────────────────────────────────────────
        cv2.putText(img, f'frame {fi}', (30, h - 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (180, 180, 180), 1)

        writer.write(img)
        fi += 1
        if fi % 300 == 0:
            print(f'   เขียน frame {fi}/{total}', end='\r', flush=True)

    cap.release()
    writer.release()
    print(f'\n✅ บันทึก annotated video: {out_path}')


# ============================================================================
# MANUAL LABEL MODE
# ============================================================================

def _extract_e_near_frame(
    fi: int,
    e_map: dict,
    transform,
    window: int,
) -> dict | None:
    """ดึง E position เฉลี่ยจาก ±window frames รอบ fi → dict หรือ None"""
    origin, vx, vy_dn, ppmm = transform
    pts = [e_map[j] for j in range(fi - window, fi + window + 1) if j in e_map]
    if not pts:
        return None
    med = np.median(pts, axis=0)
    x_mm, y_mm = px_to_legframe(med, origin, vx, vy_dn, ppmm)
    return {
        'frame_idx':   fi,
        'e_px_median': med,
        'actual_x_mm': round(x_mm, 3),
        'actual_y_mm': round(y_mm, 3),
    }


def manual_label_video(
    video_path: str,
    frames: list[dict],
    transform,
    fps: float,
    n_points: int,
    camera_matrix=None,
    dist_coeffs=None,
    window_frames: int = MANUAL_WINDOW_FRAMES,
) -> list[dict]:
    """
    Manual labeling — เปิดวิดีโอ กด Space/Enter เพื่อ mark ตำแหน่ง E ของแต่ละจุดทดสอบ

    Controls:
        Space / Enter  — mark frame นี้เป็น point ถัดไป (auto-pause หลัง mark)
        Backspace      — ยกเลิก mark ล่าสุด
        p              — pause / resume
        , / .          — ถอยหลัง / เดินหน้า 1 frame  (ขณะ pause)
        [ / ]          — ถอยหลัง / เดินหน้า 10 frames (ขณะ pause)
        q / Esc        — เสร็จสิ้น ดำเนินการต่อ
    """
    origin, vx, vy_dn, ppmm = transform
    origin_int = tuple(origin.astype(int))

    probe = cv2.VideoCapture(video_path)
    w     = int(probe.get(cv2.CAP_PROP_FRAME_WIDTH))
    h     = int(probe.get(cv2.CAP_PROP_FRAME_HEIGHT))
    total = int(probe.get(cv2.CAP_PROP_FRAME_COUNT))
    probe.release()

    map1, map2, _ = build_undistort_maps(camera_matrix, dist_coeffs, w, h)
    e_map = {d['frame_idx']: d['e_px'] for d in frames if d['e_px'] is not None}

    labels: list[dict] = []
    paused = False
    fi     = 0
    delay  = max(1, int(1000 / fps))

    FONT = cv2.FONT_HERSHEY_SIMPLEX
    WIN  = 'Manual Label  (q = Done)'
    win_w = min(w, 1280)
    win_h = int(h * win_w / w)
    cv2.namedWindow(WIN, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(WIN, win_w, win_h)

    print('  Controls: Space/Enter=Mark  BackSpace=Undo  '
          'p=Pause  , .=Step1  [ ]=Step10  q=Done')

    cap        = cv2.VideoCapture(video_path)
    cap_fi     = 0    # frame ถัดไปที่ VideoCapture จะอ่าน
    cached_fi  = -1   # frame index ของ cached_raw
    cached_raw = None # เก็บ frame ที่ undistort แล้ว (ลด decode ซ้ำขณะ pause)

    while fi < total:
        # decode ใหม่เฉพาะเมื่อ fi เปลี่ยน — ลด seek เมื่อเล่นต่อเนื่อง
        if fi != cached_fi:
            if fi != cap_fi:
                cap.set(cv2.CAP_PROP_POS_FRAMES, fi)
                cap_fi = fi
            ret, raw = cap.read()
            if not ret:
                break
            cap_fi += 1
            if map1 is not None:
                raw = cv2.remap(raw, map1, map2, cv2.INTER_LINEAR)
            cached_raw = raw
            cached_fi  = fi

        img = cached_raw.copy()

        # ─── E marker ────────────────────────────────────────────────
        e_ok = fi in e_map
        if e_ok:
            pt     = tuple(e_map[fi].astype(int))
            xm, ym = px_to_legframe(e_map[fi], origin, vx, vy_dn, ppmm)
            cv2.circle(img, pt, 12, (0, 165, 255), -1)
            cv2.circle(img, pt, 12, (255, 255, 255), 2)
            cv2.putText(img, f'({xm:+.1f}, {ym:+.1f}) mm',
                        (pt[0] + 16, pt[1] - 10), FONT, 0.6, (255, 255, 50), 2)
        cv2.circle(img, origin_int, 6, (200, 100, 0), -1)

        # ─── Top bar ─────────────────────────────────────────────────
        next_pid = len(labels) + 1
        e_warn   = '  ⚠ E not detected' if not e_ok else ''
        status   = 'PAUSED' if paused else 'PLAYING'
        top_txt  = (f'{status}  frame {fi}/{total - 1}'
                    f'  |  Next: Point {next_pid}/{n_points}'
                    f'  |  Marked: {len(labels)}{e_warn}')
        cv2.rectangle(img, (0, 0), (w, 36), (20, 20, 20), -1)
        cv2.putText(img, top_txt, (8, 25), FONT, 0.62, (220, 220, 220), 1)

        # ─── Last 5 marks ─────────────────────────────────────────────
        for k, lbl in enumerate(labels[-5:]):
            txt = (f'  #{lbl["point_id"]:2d}  f={lbl["frame_idx"]}'
                   f'  ({lbl["actual_x_mm"]:+.1f}, {lbl["actual_y_mm"]:+.1f}) mm')
            cv2.putText(img, txt, (8, 56 + k * 22), FONT, 0.48, (80, 220, 80), 1)

        # ─── Bottom bar ───────────────────────────────────────────────
        ctrl = 'Space/Enter=Mark  BackSpace=Undo  p=Pause  , .=Step1  [ ]=Step10  q=Done'
        cv2.rectangle(img, (0, h - 30), (w, h), (20, 20, 20), -1)
        cv2.putText(img, ctrl, (8, h - 10), FONT, 0.45, (160, 160, 160), 1)

        cv2.imshow(WIN, img)
        key = cv2.waitKeyEx(0 if paused else delay)

        # ─── Key handling ─────────────────────────────────────────────
        if key in (ord('q'), 27):                     # q / Esc
            break

        elif key == ord('p'):                          # p = pause/resume
            paused = not paused

        elif key in (32, 13):                          # Space / Enter = mark
            if next_pid <= n_points:
                e_pos = _extract_e_near_frame(fi, e_map, transform, window_frames)
                if e_pos is not None:
                    e_pos.update({'point_id': next_pid,
                                  'frame_start': fi, 'frame_end': fi,
                                  'duration_s': 0.0})
                    labels.append(e_pos)
                    print(f'   ✓ Mark Point {next_pid:2d}  frame={fi}'
                          f'  ({e_pos["actual_x_mm"]:+.3f},'
                          f' {e_pos["actual_y_mm"]:+.3f}) mm')
                    paused = True          # auto-pause หลัง mark
                else:
                    print(f'   ⚠️  E ไม่ถูกตรวจจับใกล้ frame {fi} — เลื่อนไปเฟรมอื่น')
            else:
                print(f'   ℹ️  ครบ {n_points} จุดแล้ว — กด q เพื่อดำเนินการต่อ')

        elif key == 8:                                 # Backspace = undo
            if labels:
                r = labels.pop()
                print(f'   ✗ Undo Point {r["point_id"]}  frame={r["frame_idx"]}')

        # Navigation (step) — sets paused=True implicitly
        elif key in (ord(','), 2424832, 65361):        # ← / ,
            fi = max(0, fi - 1);     paused = True;  continue
        elif key in (ord('.'), 2555904, 65363):        # → / .
            fi = min(total - 1, fi + 1); paused = True; continue
        elif key == ord('['):                           # [ = -10
            fi = max(0, fi - 10);    paused = True;  continue
        elif key == ord(']'):                           # ] = +10
            fi = min(total - 1, fi + 10); paused = True; continue

        if not paused:
            fi += 1

    cap.release()
    cv2.destroyWindow(WIN)
    print(f'\n  Manual labels: {len(labels)}/{n_points} จุด')
    return labels


# ============================================================================
# DIAGNOSTICS — แสดง segment list ทั้งหมด
# ============================================================================

def print_segment_table(dwell_results: list[dict], n_show: int = 10):
    print(f'\n  {"ID":>3}  {"frames":>15}  {"dur(s)":>7}'
          f'  {"actual_x":>10}  {"actual_y":>10}')
    for d in dwell_results[:n_show]:
        print(f'  {d["point_id"]:>3}  '
              f'{d["frame_start"]:6d}–{d["frame_end"]:<6d}  '
              f'{d["duration_s"]:>7.2f}  '
              f'{d["actual_x_mm"]:>10.3f}  '
              f'{d["actual_y_mm"]:>10.3f}')
    if len(dwell_results) > n_show:
        print(f'  ... ({len(dwell_results) - n_show} จุดที่เหลือ)')


# ============================================================================
# MAIN
# ============================================================================

def main():
    print('=' * 65)
    print('  Video 5-Bar Linkage Grid Tracking')
    print('=' * 65)
    print(f'  Video : {VIDEO_PATH}')
    print(f'  CSV   : {GRID_LOG_CSV}')
    print('=' * 65)

    # ─── โหลด Calibration ────────────────────────────────────────────
    camera_matrix, dist_coeffs = load_calibration(CALIB_PATH)
    detector = aruco.ArucoDetector(ARUCO_DICT, PARAMETERS)

    # ─── Phase 1: Scan ───────────────────────────────────────────────
    print('\n[ Phase 1: Scan Video ]')
    try:
        frames, transform, fps = scan_video(
            VIDEO_PATH, camera_matrix, dist_coeffs, detector,
            show_preview=MANUAL_MODE)
    except (FileNotFoundError, RuntimeError) as e:
        print(f'\n❌ {e}')
        return

    # ─── Phase 2 / Manual Label → dwell_results ──────────────────────
    if MANUAL_MODE:
        print('\n[ Manual Labeling — กด q เมื่อ mark ครบแล้ว ]')
        dwell_results = manual_label_video(
            VIDEO_PATH, frames, transform, fps, N_POINTS,
            camera_matrix, dist_coeffs,
            window_frames=MANUAL_WINDOW_FRAMES,
        )
        if not dwell_results:
            print('  ❌ ไม่มี labels — ยกเลิก')
            return
    else:
        # ─── Phase 2: Detect Dwells ──────────────────────────────────
        print('\n[ Phase 2: Detect Dwell Segments ]')
        segments, e_by_frame = detect_dwells(
            frames, transform, fps,
            stable_vel_mm_s   = STABLE_VEL_MM_S,
            min_stable_frames = MIN_STABLE_FRAMES,
            skip_initial      = SKIP_INITIAL_FRAMES,
            max_gap           = MAX_FRAME_GAP,
            merge_gap         = MERGE_GAP_FRAMES,
        )

        n_seg = len(segments)
        print(f'\n  พบ {n_seg} dwell segments  (ต้องการ {N_POINTS})')

        if n_seg == 0:
            print('  ❌ ไม่พบ dwell segment เลย')
            print('     → ลด STABLE_VEL_MM_S หรือลด MIN_STABLE_FRAMES')
            return

        if n_seg != N_POINTS:
            print()
            if n_seg < N_POINTS:
                print(f'  ⚠️  น้อยกว่า {N_POINTS}  '
                      f'→ ลด STABLE_VEL_MM_S ({STABLE_VEL_MM_S})'
                      f' หรือลด MIN_STABLE_FRAMES ({MIN_STABLE_FRAMES})')
            else:
                print(f'  ⚠️  มากกว่า {N_POINTS}  '
                      f'(อาจมี spurious dwells — greedy matching จะกรองออกเอง)')
            print()

        # ─── Phase 3: Extract Positions ──────────────────────────────
        print('\n[ Phase 3: Extract E Positions ]')
        dwell_results = extract_positions(
            segments, e_by_frame, transform, fps, DWELL_SAMPLE_FRAC)
        print(f'  ดึงค่าสำเร็จ: {len(dwell_results)} จุด')
        print_segment_table(dwell_results)

        # ─── จับคู่ dwell กับ target (greedy forward matching) ───────
        print('\n[ จับคู่ dwell กับ target positions (greedy matching) ]')
        targets = load_targets(GRID_LOG_CSV)
        if targets:
            start_idx = find_sequence_start(dwell_results, targets)
            if start_idx > 0:
                print(f'  → ข้าม {start_idx} dwell แรก (Home/pre-sequence)'
                      f'  เริ่มจาก dwell #{start_idx + 1}')
            dwell_results = match_dwells_to_targets(
                dwell_results, targets, start_idx,
                tol_mm=SEQ_MATCH_TOL_MM, max_skip=MAX_SKIP_DWELLS,
            )
        else:
            print('  ⚠️  ไม่สามารถโหลด targets — ใช้ dwell ทั้งหมดตามเดิม')
            dwell_results = dwell_results[:N_POINTS]

    # ─── Phase 4: Update grid_log.csv ────────────────────────────────
    print(f'\n[ Phase 4: Update grid_log.csv ]')
    errors = update_grid_log(GRID_LOG_CSV, dwell_results)
    print(f'✅ บันทึก {len(errors)} แถวลง {os.path.basename(GRID_LOG_CSV)}')

    # ─── Summary ─────────────────────────────────────────────────────
    if errors:
        print(f'\n{"="*65}')
        print(f'  สรุป Error  (video actual vs target)')
        print(f'{"="*65}')
        print(f'  จุดที่วิเคราะห์    : {len(errors)}/{N_POINTS}')
        print(f'  Mean  dist error  : {np.mean(errors):.3f} mm')
        print(f'  Std   dist error  : {np.std(errors):.3f} mm')
        print(f'  Max   dist error  : {np.max(errors):.3f} mm')
        print(f'  Min   dist error  : {np.min(errors):.3f} mm')
        print('=' * 65)

    # ─── Phase 5: Annotated Video (optional) ─────────────────────────
    ans = input('\n  สร้าง annotated video? (y/N): ').strip().lower()
    if ans == 'y':
        print(f'\n[ Phase 5: Annotated Video ]')
        save_annotated_video(
            VIDEO_PATH, frames, transform, fps, dwell_results,
            OUTPUT_VIDEO, camera_matrix, dist_coeffs,
        )

    print('\n✅ เสร็จสิ้น')


if __name__ == '__main__':
    main()
