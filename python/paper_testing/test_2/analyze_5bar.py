"""
5-Bar Linkage ArTag Analysis
Author: M-TRCH
Date: May 2026

วิเคราะห์ภาพถ่ายกลไก 5-Bar linkage:
  - ตรวจจับ ArUco markers (ID 0–4) และคำนวณพิกัดจริง (mm) ในระบบพิกัดขา
  - แยกข้อมูลมอเตอร์จากชื่อไฟล์  (รูปแบบจาก capture N ใน single_leg_xy_control.py)
  - ส่งออกผลลัพธ์ทั้งหมดเป็น CSV และภาพ annotated

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

ชื่อไฟล์ภาพที่รองรับ (จาก capture mode):
    p{id:03d}_x{x}_y{y}_cA{tA_cmd}_cB{tB_cmd}_aA{tA_act}_aB{tB_act}.jpg
    ตัวอย่าง:  p010_xp15p0_yn180p0_cAn63p5_cBn85p2_aAn63p1_aBn85p0.jpg
"""

import cv2
import cv2.aruco as aruco
import numpy as np
import os
import glob
import re
import csv

# ============================================================================
# CONFIGURATION
# ============================================================================

ARUCO_DICT    = aruco.getPredefinedDictionary(aruco.DICT_6X6_250)
PARAMETERS    = aruco.DetectorParameters()
INPUT_FOLDER  = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'capture_image')
OUTPUT_FOLDER = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'output_image')
CALIB_PATH    = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'calibration.npz')
OUTPUT_CSV    = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'analysis_results.csv')

IMAGE_EXTS = ("*.jpg", "*.jpeg", "*.png", "*.JPG", "*.JPEG", "*.PNG")

# ArUco marker ID → ชื่อ joint
JOINT_LABEL = {
    0: "A (Motor-L)",
    1: "B (Motor-R)",
    2: "C (Knee-L)",
    3: "D (Knee-R)",
    4: "E (Foot)",
}

# ความยาว link ที่ทราบ (mm)
LINK_LENGTHS = {
    (0, 2): 105.0,   # A–C
    (1, 3): 105.0,   # B–D
    (2, 4): 145.0,   # C–E
    (3, 4): 145.0,   # D–E
}

# คู่ joint ที่ต้องวาดเส้นเชื่อม
LINK_PAIRS = [(0, 2), (1, 3), (2, 4), (3, 4)]

# สีสำหรับ annotation  (BGR)
COLOR_LINK    = (0,  220,  0)    # เขียว
COLOR_JOINT   = (0,   0, 220)    # แดง
COLOR_JOINT_E = (0, 165, 255)    # ส้ม  (end-effector)
COLOR_TEXT    = (255, 255,  50)  # เหลือง
COLOR_INFO    = (0,  200, 255)   # ฟ้า
COLOR_ERROR   = (50,  50, 255)   # แดงอ่อน

# ============================================================================
# FILENAME PARSER
# ============================================================================

def _decode_angle_str(s: str) -> float | None:
    """
    ถอดรหัสสตริงมุมจากชื่อไฟล์
        'p15p0'  → +15.0
        'n63p5'  → -63.5
        'nan'    → None
    """
    if s == 'nan':
        return None
    sign = +1.0 if s[0] == 'p' else -1.0
    return sign * float(s[1:].replace('p', '.'))


# regex สำหรับแยก metadata จากชื่อไฟล์
_FNAME_RE = re.compile(
    r'^p(\d+)'
    r'_x([pn][0-9p]+|nan)'
    r'_y([pn][0-9p]+|nan)'
    r'_cA([pn][0-9p]+|nan)'
    r'_cB([pn][0-9p]+|nan)'
    r'_aA([pn][0-9p]+|nan)'
    r'_aB([pn][0-9p]+|nan)'
    r'\.[^.]+$'
)


def parse_filename(filename: str) -> dict | None:
    """
    แยกข้อมูลมอเตอร์จากชื่อไฟล์  → dict หรือ None ถ้าไม่ตรงรูปแบบ
    """
    m = _FNAME_RE.match(os.path.basename(filename))
    if not m:
        return None
    return {
        'point_id':       int(m.group(1)),
        'target_x_mm':    _decode_angle_str(m.group(2)),
        'target_y_mm':    _decode_angle_str(m.group(3)),
        'cmd_thetaA_deg': _decode_angle_str(m.group(4)),
        'cmd_thetaB_deg': _decode_angle_str(m.group(5)),
        'act_thetaA_deg': _decode_angle_str(m.group(6)),
        'act_thetaB_deg': _decode_angle_str(m.group(7)),
    }


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

def estimate_scale(detected: dict[int, np.ndarray]) -> float | None:
    """
    ประมาณ scale (px/mm) จากทุก link ที่มีทั้งสองปลายถูกตรวจจับ
    → ค่าเฉลี่ยของทุก link ที่ใช้ได้
    """
    scales = []
    for (id1, id2), length_mm in LINK_LENGTHS.items():
        if id1 in detected and id2 in detected:
            d_px = np.linalg.norm(detected[id2] - detected[id1])
            if d_px > 1.0:
                scales.append(d_px / length_mm)
    return float(np.mean(scales)) if scales else None


def build_legframe_axes(a_px: np.ndarray, b_px: np.ndarray) -> tuple:
    """
    สร้างแกน leg frame จากตำแหน่ง pixel ของ A และ B

    คืน (M_px, v_x, v_y_down):
        M_px     : origin (จุดกึ่งกลาง A–B) ใน pixel
        v_x      : unit vector ทิศ +x (A→B)
        v_y_down : unit vector ทิศลง (ตั้งฉากกับ v_x ชี้ลงในภาพ)
                   ทิศลงในภาพ = y ลบในระบบพิกัดขา
    """
    M_px = (a_px + b_px) / 2.0
    v_AB = b_px - a_px
    v_x  = v_AB / np.linalg.norm(v_AB)
    # 90° CCW ในระบบ image (y ลงบวก) = ทิศลง เมื่อ A อยู่ซ้าย–B ขวา
    v_y_down = np.array([-v_x[1], v_x[0]])
    return M_px, v_x, v_y_down


def px_to_legframe_mm(
    point_px: np.ndarray,
    origin_px: np.ndarray,
    v_x: np.ndarray,
    v_y_down: np.ndarray,
    px_per_mm: float,
) -> tuple[float, float]:
    """
    แปลงพิกัด pixel → (x_mm, y_mm) ในระบบพิกัดขา

    y_leg = -(ระยะลงในภาพ) เพราะ ลงในภาพ = y ลบในระบบพิกัดขา
    """
    d = point_px - origin_px
    x_mm = float(np.dot(d, v_x)      / px_per_mm)
    y_mm = float(-np.dot(d, v_y_down) / px_per_mm)
    return x_mm, y_mm


# ============================================================================
# IMAGE PROCESSING
# ============================================================================

def process_image(
    image_path: str,
    camera_matrix,
    dist_coeffs,
    detector,
    output_folder: str,
) -> dict | None:
    """
    ประมวลผล 1 ภาพ → คืน dict ข้อมูลทั้งหมด หรือ None ถ้าอ่านไม่ได้
    """
    filename = os.path.basename(image_path)
    meta     = parse_filename(filename)

    print(f'\n{"─"*60}')
    print(f'📂 {filename}')
    if meta:
        print(f'   ID={meta["point_id"]}  '
              f'target=({meta["target_x_mm"]:+.1f}, {meta["target_y_mm"]:+.1f}) mm  '
              f'cmdA={meta["cmd_thetaA_deg"]:+.1f}°  '
              f'actA={meta["act_thetaA_deg"]:+.1f}°')

    img = cv2.imread(image_path)
    if img is None:
        print('  ⚠️  อ่านไฟล์ไม่ได้ ข้ามไป')
        return None

    # ─── แก้ความเพี้ยนเลนส์ ────────────────────────────────────────────
    h, w = img.shape[:2]
    new_mtx, roi = cv2.getOptimalNewCameraMatrix(camera_matrix, dist_coeffs, (w, h), alpha=0)
    img_u = cv2.undistort(img, camera_matrix, dist_coeffs, None, new_mtx)
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

    n_det = len(detected)
    print(f'  ตรวจพบ {n_det}/5 จุด: {sorted(detected.keys())}')

    # ─── คำนวณ scale และพิกัด leg frame ────────────────────────────────
    px_per_mm = estimate_scale(detected)

    # สร้าง result dict (เริ่มจาก metadata)
    result: dict = {'filename': filename}
    _empty_meta = {
        'point_id': None, 'target_x_mm': None, 'target_y_mm': None,
        'cmd_thetaA_deg': None, 'cmd_thetaB_deg': None,
        'act_thetaA_deg': None, 'act_thetaB_deg': None,
    }
    result.update(meta if meta else _empty_meta)
    result['n_detected'] = n_det
    result['px_per_mm']  = round(px_per_mm, 4) if px_per_mm else None

    # พิกัด pixel ของแต่ละ joint
    for jid in range(5):
        result[f'px_id{jid}_x'] = round(detected[jid][0], 1) if jid in detected else None
        result[f'px_id{jid}_y'] = round(detected[jid][1], 1) if jid in detected else None

    # พิกัด leg-frame (mm) ของแต่ละ joint
    for jid in range(5):
        result[f'lf_id{jid}_x_mm'] = None
        result[f'lf_id{jid}_y_mm'] = None

    lf_available = False
    M_px = v_x = v_y_down = None

    if 0 in detected and 1 in detected and px_per_mm:
        M_px, v_x, v_y_down = build_legframe_axes(detected[0], detected[1])
        for jid, pt in detected.items():
            x_mm, y_mm = px_to_legframe_mm(pt, M_px, v_x, v_y_down, px_per_mm)
            result[f'lf_id{jid}_x_mm'] = round(x_mm, 2)
            result[f'lf_id{jid}_y_mm'] = round(y_mm, 2)
        lf_available = True

    # ตรวจสอบความสอดคล้อง A/B กับระยะที่ทราบ (sanity check)
    if lf_available:
        a_x, a_y = result['lf_id0_x_mm'], result['lf_id0_y_mm']
        b_x, b_y = result['lf_id1_x_mm'], result['lf_id1_y_mm']
        ab_dist  = np.hypot(b_x - a_x, b_y - a_y)
        result['ab_dist_mm'] = round(ab_dist, 2)

    # End-effector E (ID 4) → detected_x/y + error
    if lf_available and 4 in detected:
        e_x = result['lf_id4_x_mm']
        e_y = result['lf_id4_y_mm']
        result['detected_x_mm'] = e_x
        result['detected_y_mm'] = e_y
        if meta and meta['target_x_mm'] is not None and meta['target_y_mm'] is not None:
            result['error_x_mm']    = round(e_x - meta['target_x_mm'], 2)
            result['error_y_mm']    = round(e_y - meta['target_y_mm'], 2)
            result['error_dist_mm'] = round(np.hypot(
                e_x - meta['target_x_mm'], e_y - meta['target_y_mm']), 2)
            print(f'  E detected : ({e_x:+.2f}, {e_y:+.2f}) mm')
            print(f'  E target   : ({meta["target_x_mm"]:+.2f}, {meta["target_y_mm"]:+.2f}) mm')
            print(f'  E error    : ({result["error_x_mm"]:+.2f}, {result["error_y_mm"]:+.2f}) mm'
                  f'  dist={result["error_dist_mm"]:.2f} mm')
        else:
            result['error_x_mm']    = None
            result['error_y_mm']    = None
            result['error_dist_mm'] = None
    else:
        result.setdefault('detected_x_mm', None)
        result.setdefault('detected_y_mm', None)
        result.setdefault('error_x_mm',    None)
        result.setdefault('error_y_mm',    None)
        result.setdefault('error_dist_mm', None)

    result.setdefault('ab_dist_mm', None)

    # ─── Annotate image ─────────────────────────────────────────────────
    scale_f = max(1, img_u.shape[1] // 1000)   # ปรับขนาดตามความละเอียดภาพ
    lw       = 2 * scale_f    # line width
    cr       = 10 * scale_f   # circle radius
    ft       = 0.55 * scale_f # font scale
    ft_thick = max(1, 2 * scale_f)

    # เส้น link
    for p1_id, p2_id in LINK_PAIRS:
        if p1_id in detected and p2_id in detected:
            pt1 = tuple(detected[p1_id].astype(int))
            pt2 = tuple(detected[p2_id].astype(int))
            cv2.line(img_u, pt1, pt2, COLOR_LINK, lw + 2)
            # แสดงความยาว link (px)
            d_px  = np.linalg.norm(detected[p2_id] - detected[p1_id])
            d_mm  = LINK_LENGTHS.get((p1_id, p2_id), LINK_LENGTHS.get((p2_id, p1_id)))
            mid   = ((detected[p1_id] + detected[p2_id]) / 2).astype(int)
            cv2.putText(img_u, f'{d_mm:.0f}mm', tuple(mid + np.array([5, -8])),
                        cv2.FONT_HERSHEY_SIMPLEX, ft * 0.8, (200, 255, 200), ft_thick - 1)

    # จุด joint
    for jid, pos in detected.items():
        pt    = tuple(pos.astype(int))
        color = COLOR_JOINT_E if jid == 4 else COLOR_JOINT
        cv2.circle(img_u, pt, cr, color, -1)
        cv2.circle(img_u, pt, cr, (255, 255, 255), max(1, lw - 1))

        # ชื่อ + พิกัด mm
        x_lf = result.get(f'lf_id{jid}_x_mm')
        y_lf = result.get(f'lf_id{jid}_y_mm')
        if x_lf is not None:
            label_txt = f'{JOINT_LABEL[jid]}  ({x_lf:+.1f}, {y_lf:+.1f}) mm'
        else:
            label_txt = JOINT_LABEL[jid]
        cv2.putText(img_u, label_txt,
                    (pt[0] + cr + 5, pt[1] - cr),
                    cv2.FONT_HERSHEY_SIMPLEX, ft, COLOR_TEXT, ft_thick)

    # ข้อมูล scale
    if px_per_mm:
        cv2.putText(img_u, f'Scale: {px_per_mm:.3f} px/mm',
                    (20, 55 * scale_f),
                    cv2.FONT_HERSHEY_SIMPLEX, ft * 1.1, COLOR_INFO, ft_thick + 1)

    # ข้อมูล error ของ E
    err_dist = result.get('error_dist_mm')
    if err_dist is not None:
        cv2.putText(img_u,
                    f'E err: {result["error_x_mm"]:+.2f}, {result["error_y_mm"]:+.2f} mm'
                    f'  |  dist={err_dist:.2f} mm',
                    (20, 105 * scale_f),
                    cv2.FONT_HERSHEY_SIMPLEX, ft * 1.0, COLOR_ERROR, ft_thick + 1)

    # ข้อมูล metadata (ชื่อไฟล์/target) ที่ด้านล่าง
    if meta:
        info_str = (f"ID={meta['point_id']}  "
                    f"tgt=({meta['target_x_mm']:+.1f},{meta['target_y_mm']:+.1f})mm  "
                    f"cA={meta['cmd_thetaA_deg']:+.1f} cB={meta['cmd_thetaB_deg']:+.1f}  "
                    f"aA={meta['act_thetaA_deg']:+.1f} aB={meta['act_thetaB_deg']:+.1f}deg")
        margin = 15 * scale_f
        cv2.putText(img_u, info_str,
                    (20, img_u.shape[0] - margin),
                    cv2.FONT_HERSHEY_SIMPLEX, ft * 0.85,
                    (200, 200, 200), ft_thick)

    # ─── บันทึกภาพ ──────────────────────────────────────────────────────
    out_name = os.path.splitext(filename)[0] + '_tracked.jpg'
    out_path = os.path.join(output_folder, out_name)
    cv2.imwrite(out_path, img_u, [cv2.IMWRITE_JPEG_QUALITY, 95])
    print(f'  💾 บันทึก: {out_name}')

    return result


# ============================================================================
# CSV OUTPUT
# ============================================================================

CSV_COLUMNS = [
    # metadata จากชื่อไฟล์
    'filename', 'point_id',
    'target_x_mm', 'target_y_mm',
    'cmd_thetaA_deg', 'cmd_thetaB_deg',
    'act_thetaA_deg', 'act_thetaB_deg',
    # ArTag detection
    'n_detected', 'px_per_mm', 'ab_dist_mm',
    # ตำแหน่ง E ที่ตรวจจับได้ + error
    'detected_x_mm', 'detected_y_mm',
    'error_x_mm', 'error_y_mm', 'error_dist_mm',
    # พิกัด pixel ของแต่ละ joint
    'px_id0_x', 'px_id0_y',
    'px_id1_x', 'px_id1_y',
    'px_id2_x', 'px_id2_y',
    'px_id3_x', 'px_id3_y',
    'px_id4_x', 'px_id4_y',
    # พิกัด leg-frame (mm) ของแต่ละ joint
    'lf_id0_x_mm', 'lf_id0_y_mm',
    'lf_id1_x_mm', 'lf_id1_y_mm',
    'lf_id2_x_mm', 'lf_id2_y_mm',
    'lf_id3_x_mm', 'lf_id3_y_mm',
    'lf_id4_x_mm', 'lf_id4_y_mm',
]


def save_csv(rows: list[dict], csv_path: str):
    with open(csv_path, 'w', newline='', encoding='utf-8-sig') as f:
        writer = csv.DictWriter(f, fieldnames=CSV_COLUMNS, extrasaction='ignore')
        writer.writeheader()
        for row in rows:
            writer.writerow({k: ('' if row.get(k) is None else row[k]) for k in CSV_COLUMNS})
    print(f'\n💾 บันทึก CSV: {os.path.abspath(csv_path)}  ({len(rows)} แถว)')


# ============================================================================
# MAIN
# ============================================================================

def process_all():
    print('=' * 65)
    print('  5-Bar Linkage ArTag Analysis')
    print('=' * 65)

    # โหลดค่าสอบเทียบกล้อง
    try:
        camera_matrix, dist_coeffs = load_calibration(CALIB_PATH)
    except FileNotFoundError:
        print(f'❌ ไม่พบไฟล์สอบเทียบ: {CALIB_PATH}')
        return
    except KeyError as e:
        print(f'❌ {e}')
        return

    # รวบรวมไฟล์ภาพ (ไม่รวม output)
    image_files: list[str] = []
    for ext in IMAGE_EXTS:
        image_files.extend(glob.glob(os.path.join(INPUT_FOLDER, ext)))

    abs_output = os.path.abspath(OUTPUT_FOLDER)
    image_files = sorted(set(
        f for f in image_files
        if not os.path.abspath(f).startswith(abs_output)
    ))

    if not image_files:
        print(f'❌ ไม่พบไฟล์ภาพในโฟลเดอร์: {os.path.abspath(INPUT_FOLDER)}')
        return

    print(f'\n🔍 พบไฟล์ภาพ {len(image_files)} ไฟล์')
    os.makedirs(OUTPUT_FOLDER, exist_ok=True)

    detector = aruco.ArucoDetector(ARUCO_DICT, PARAMETERS)

    rows: list[dict] = []
    failed           = 0
    for img_path in image_files:
        row = process_image(img_path, camera_matrix, dist_coeffs, detector, OUTPUT_FOLDER)
        if row:
            rows.append(row)
        else:
            failed += 1

    # เรียงลำดับ point_id ก่อน export CSV
    rows.sort(key=lambda r: (r['point_id'] is None, r['point_id'] or 0, r['filename']))

    if rows:
        save_csv(rows, OUTPUT_CSV)

    # สรุปผล
    detected_e = [r for r in rows if r.get('detected_x_mm') is not None]
    error_rows  = [r for r in detected_e if r.get('error_dist_mm') is not None]

    print(f'\n{"="*65}')
    print(f'  สรุปผล')
    print(f'{"="*65}')
    print(f'  ไฟล์ทั้งหมด          : {len(image_files)}')
    print(f'  ประมวลผลสำเร็จ       : {len(rows)}')
    print(f'  ตรวจจับ E ได้         : {len(detected_e)}')
    if error_rows:
        errs = [r['error_dist_mm'] for r in error_rows]
        print(f'  Error (dist) เฉลี่ย  : {np.mean(errs):.2f} mm')
        print(f'  Error (dist) สูงสุด  : {np.max(errs):.2f} mm')
        print(f'  Error (dist) ต่ำสุด   : {np.min(errs):.2f} mm')
    print(f'  ภาพ annotated        : {abs_output}')
    print(f'  CSV                  : {os.path.abspath(OUTPUT_CSV)}')
    print('=' * 65)


if __name__ == '__main__':
    process_all()
