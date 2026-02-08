import cv2
import numpy as np
import threading
import queue
import time

# --- 1. ตั้งค่าพื้นฐาน ---
# เลือก Dictionary ให้ตรงกับ Tag ที่คุณสร้าง (DICT_6X6_250)
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
aruco_params = cv2.aruco.DetectorParameters()

# --- ปรับ ArUco Parameters สำหรับความเร็วสูงสุด ---
aruco_params.adaptiveThreshWinSizeMin = 3
aruco_params.adaptiveThreshWinSizeMax = 23
aruco_params.adaptiveThreshWinSizeStep = 10  # เพิ่มจาก default 10 -> ข้ามขนาดหน้าต่างมากขึ้น
aruco_params.minMarkerPerimeterRate = 0.03  # ลดขนาด marker ขั้นต่ำ
aruco_params.maxMarkerPerimeterRate = 4.0
aruco_params.polygonalApproxAccuracyRate = 0.05  # เพิ่มความเร็วในการ approximate polygon
aruco_params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_NONE  # ปิด corner refinement เพื่อความเร็ว

# สร้าง Detector (สำหรับ OpenCV 4.7+)
detector = cv2.aruco.ArucoDetector(aruco_dict, aruco_params)

# --- 2. ตั้งค่าขนาด Marker (สำคัญมาก!) ---
# วัดขนาดจริงของ Marker ที่คุณ "พิมพ์" ออกมา (วัดเฉพาะส่วนสีดำ)
# หน่วยเป็น "เมตร"
MARKER_SIZE_METERS = 0.024  # 24 มิลลิเมตร = 0.024 เมตร

# Pre-compute object points (ประหยัดเวลาไม่ต้องสร้างใหม่ทุก frame)
OBJ_POINTS = np.array([
    [-MARKER_SIZE_METERS/2, MARKER_SIZE_METERS/2, 0],
    [MARKER_SIZE_METERS/2, MARKER_SIZE_METERS/2, 0],
    [MARKER_SIZE_METERS/2, -MARKER_SIZE_METERS/2, 0],
    [-MARKER_SIZE_METERS/2, -MARKER_SIZE_METERS/2, 0]
], dtype=np.float32)

# --- 3. โหลด Camera Calibration Parameters ---
# ใช้ path แบบ absolute หรือ relative จากตำแหน่งสคริปต์
import os
script_dir = os.path.dirname(os.path.abspath(__file__))
calib_file = os.path.join(script_dir, "camera_calibration", "camera_params_ef1635f4l_1080p60.npz")

try:
    calib_data = np.load(calib_file)
    camera_matrix = calib_data['mtx']
    dist_coeffs = calib_data['dist']
    print(f"โหลด Camera Parameters จาก: {calib_file}")
    print("\nCamera Matrix (K):")
    print(camera_matrix)
    print("\nDistortion Coefficients (D):")
    print(dist_coeffs)
except FileNotFoundError:
    print(f"ไม่พบไฟล์ {calib_file}")
    print("กรุณาทำ camera calibration ก่อนใช้งาน")
    exit()
#----------------------------------------------------

# --- 4. เปิดกล้อง ---
cap = cv2.VideoCapture(1)   # ใช้กล้องตัวที่สอง (1)
if not cap.isOpened():
    print("ไม่สามารถเปิดกล้องได้")
    exit()

# ตั้งค่าความละเอียดกล้องเป็น 1920x1080 @ 60fps (ตรงกับ calibration)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)
cap.set(cv2.CAP_PROP_FPS, 60)

# อ่านความละเอียดจริงที่ได้
actual_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
actual_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
print(f"\nความละเอียดกล้อง: {actual_width}x{actual_height}")

print("\nกำลังเปิดกล้อง... กด 'q' เพื่อออกจากโปรแกรม")

# --- ตั้งค่าการแสดงผลให้พอดีกับจอ ---
# จอแสดงผล: 1920x1080 pixels
DISPLAY_WIDTH = 1280
DISPLAY_HEIGHT = 720

# คำนวณขนาดสำหรับแสดงผล (เหลือพื้นที่ขอบ 10%)
display_scale = min((DISPLAY_WIDTH * 0.9) / actual_width, (DISPLAY_HEIGHT * 0.9) / actual_height)
output_width = int(actual_width * display_scale)
output_height = int(actual_height * display_scale)

print(f"ขนาดการแสดงผล: {output_width}x{output_height} (Scale: {display_scale:.2f})")

# --- ตัวแปรสำหรับ Multi-threading ---
frame_queue = queue.Queue(maxsize=2)  # Queue เล็กลง = latency ต่ำลง
result_queue = queue.Queue(maxsize=2)  # Queue เล็กลง = latency ต่ำลง
stop_threads = threading.Event()  # สัญญาณหยุด threads
fps_lock = threading.Lock()  # Lock สำหรับ thread-safe FPS counter
fps_counter = 0
fps_start_time = time.time()
current_fps = 0.0  # FPS สำหรับแสดงผลบนหน้าจอ
camera_fps = 0.0  # Camera FPS
display_fps_counter = 0  # นับเฟรมสำหรับ display
display_fps_start_time = time.time()  # เวลาเริ่มต้นสำหรับ display FPS
processing_fps_counter = 0  # นับเฟรมที่ประมวลผล
processing_fps = 0.0  # Processing FPS
processing_lock = threading.Lock()  # Lock สำหรับ processing counter
verbose_print = False  # ตั้งเป็น True เพื่อแสดง marker positions ใน console

# --- ตั้งค่า Detection Optimization ---
DETECTION_SCALE = 0.5  # ย่อขนาด frame ลง 50% ก่อน detect (เพิ่มความเร็ว 4 เท่า)

# --- ฟังก์ชันสำหรับ Thread การรับภาพจากกล้อง ---
def camera_thread():
    """Thread สำหรับรับภาพจากกล้อง"""
    global fps_counter
    while not stop_threads.is_set():
        ret, frame = cap.read()
        if not ret:
            continue  # ลองใหม่แทนการ break
        
        # ใส่ frame ใน queue (ถ้าเต็มจะทิ้ง frame เก่า)
        with fps_lock:
            fps_counter += 1
        
        # ถ้า queue เต็ม ให้ทิ้ง frame เก่าและใส่ใหม่
        if frame_queue.full():
            try:
                frame_queue.get_nowait()  # ทิ้ง frame เก่า
            except queue.Empty:
                pass
        
        try:
            frame_queue.put_nowait(frame)
        except queue.Full:
            pass
    
    print("Camera thread หยุดทำงาน")

# --- ฟังก์ชันสำหรับ Thread การประมวลผล ---
def processing_thread():
    """Thread สำหรับประมวลผล ArUco detection"""
    global processing_fps_counter
    while not stop_threads.is_set():
        try:
            frame = frame_queue.get_nowait()
        except queue.Empty:
            time.sleep(0.001)  # หน่วง 1ms ถ้าไม่มี frame
            continue
        
        # Downsample frame สำหรับ detection (เพิ่มความเร็วมาก)
        if DETECTION_SCALE < 1.0:
            small_frame = cv2.resize(frame, None, fx=DETECTION_SCALE, fy=DETECTION_SCALE, 
                                    interpolation=cv2.INTER_LINEAR)
            gray = cv2.cvtColor(small_frame, cv2.COLOR_BGR2GRAY)
        else:
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
        # ตรวจจับ Markers
        corners, ids, _ = detector.detectMarkers(gray)
        
        # ปรับ corners กลับเป็น scale เดิม
        if DETECTION_SCALE < 1.0 and corners is not None:
            corners = [corner / DETECTION_SCALE for corner in corners]
        
        # สร้างผลลัพธ์ (ไม่ copy frame ถ้าไม่จำเป็น)
        result = {
            'frame': frame,
            'corners': corners,
            'ids': ids
        }
        
        # ถ้าเจอ markers ให้คำนวณ pose
        if ids is not None:
            num_markers = len(ids)
            markers_data = [None] * num_markers  # Pre-allocate list
            
            for i in range(num_markers):
                # ใช้ pre-computed object points
                success, rvec, tvec = cv2.solvePnP(
                    OBJ_POINTS,
                    corners[i][0],
                    camera_matrix,
                    dist_coeffs,
                    flags=cv2.SOLVEPNP_IPPE_SQUARE
                )
                
                if success:
                    markers_data[i] = {
                        'id': ids[i][0],
                        'rvec': rvec,
                        'tvec': tvec,
                        'center': corners[i][0].mean(axis=0).astype(np.int32)  # Pre-compute center
                    }
            
            # กรอง None ออก
            result['markers_data'] = [m for m in markers_data if m is not None]
        
        # นับ processing FPS หลังประมวลผลเสร็จ (นับทุก frame ไม่ว่าจะเจอ marker หรือไม่)
        with processing_lock:
            processing_fps_counter += 1
        
        # ใส่ผลลัพธ์ใน queue (ทิ้ง frame เก่าถ้าเต็ม)
        if result_queue.full():
            try:
                result_queue.get_nowait()
            except queue.Empty:
                pass
        
        try:
            result_queue.put_nowait(result)
        except queue.Full:
            pass
    
    print("Processing thread หยุดทำงาน")

# --- เริ่ม Threads ---
camera_t = threading.Thread(target=camera_thread, daemon=True)
processing_t = threading.Thread(target=processing_thread, daemon=True)

camera_t.start()
processing_t.start()

print("เริ่ม Multi-threading ArUco Detection...")
print("กด 'v' เพื่อเปิด/ปิด verbose mode, 'q' เพื่อออก")

# --- 5. ลูปหลัก (Display Thread) ---
last_result = None
got_new_result = False  # ตัวแปรเช็คว่าได้ result ใหม่หรือไม่

while True:
    try:
        result = result_queue.get_nowait()
        last_result = result
        got_new_result = True  # ได้ result ใหม่
    except queue.Empty:
        # ใช้ผลลัพธ์ล่าสุดถ้ามี
        if last_result is not None:
            result = last_result
            got_new_result = False  # ใช้ result เดิม
        else:
            time.sleep(0.001)
            continue
    
    frame = result['frame']
    corners = result.get('corners')
    ids = result.get('ids')
    markers_data = result.get('markers_data', [])
    
    # ถ้าเจออย่างน้อย 1 Marker
    if ids is not None and len(markers_data) > 0:
        # วาดกรอบสี่เหลี่ยมรอบ Marker ที่เจอ
        cv2.aruco.drawDetectedMarkers(frame, corners, ids)
        
        # วาดแกนและแสดงข้อมูลจาก markers_data ที่คำนวณแล้ว
        for marker_data in markers_data:
            marker_id = marker_data['id']
            rvec = marker_data['rvec']
            tvec = marker_data['tvec']
            center = marker_data['center']  # ใช้ pre-computed center
            
            # วาดแกน (แกนสี แดง=X, เขียว=Y, น้ำเงิน=Z)
            cv2.drawFrameAxes(frame, camera_matrix, dist_coeffs, rvec, tvec, 0.025)
            
            # แสดงข้อมูลบนหน้าจอ
            tvec_flat = tvec.ravel()  # ravel() เร็วกว่า flatten()
            cv2.putText(frame, f"ID:{marker_id} Z:{tvec_flat[2]:.2f}m", 
                       (center[0], center[1]), cv2.FONT_HERSHEY_SIMPLEX, 
                       0.5, (0, 255, 0), 2)
            
            # แสดงใน console เฉพาะเมื่อเปิด verbose
            if verbose_print:
                print(f"ID:{marker_id} X:{tvec_flat[0]:.3f} Y:{tvec_flat[1]:.3f} Z:{tvec_flat[2]:.3f}m")

    # คำนวณ Display FPS เฉพาะเมื่อได้ result ใหม่
    if got_new_result:
        display_fps_counter += 1
    
    current_time = time.time()
    elapsed = current_time - display_fps_start_time
    if elapsed >= 0.5:  # อัพเดททุก 0.5 วินาที (เร็วกว่าเดิม)
        current_fps = display_fps_counter / elapsed
        display_fps_counter = 0
        display_fps_start_time = current_time
        
        # คำนวณ Camera FPS แบบ thread-safe
        with fps_lock:
            camera_fps = fps_counter / elapsed if elapsed > 0 else 0
            fps_counter = 0
        
        # คำนวณ Processing FPS แบบ thread-safe
        with processing_lock:
            processing_fps = processing_fps_counter / elapsed if elapsed > 0 else 0
            processing_fps_counter = 0

    # แสดง FPS ที่มุมขวาบน (แสดง 3 ค่า)
    fps_text = f"CAM:{camera_fps:.0f} PROC:{processing_fps:.0f} DISP:{current_fps:.0f}"
    cv2.rectangle(frame, (frame.shape[1]-280, 5), (frame.shape[1]-5, 35), (0, 0, 0), -1)
    cv2.putText(frame, fps_text, (frame.shape[1]-275, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 2)

    # ปรับขนาดของ frame สำหรับการแสดงผล (ใช้ INTER_NEAREST เร็วที่สุด)
    display_frame = cv2.resize(frame, (output_width, output_height), interpolation=cv2.INTER_NEAREST)
    
    # แสดงผลลัพธ์
    cv2.imshow("ArUco Detection", display_frame)
    
    # ตรวจสอบปุ่มกด
    key = cv2.waitKey(1) & 0xFF
    if key == ord('q'):
        print("กำลังหยุดโปรแกรม...")
        stop_threads.set()
        break
    elif key == ord('v'):
        verbose_print = not verbose_print
        print(f"Verbose mode: {'ON' if verbose_print else 'OFF'}")

# รอให้ threads หยุดการทำงาน
camera_t.join(timeout=1.0)
processing_t.join(timeout=1.0)

# --- 6. คืนค่ากล้องและปิดหน้าต่าง ---
cap.release()
cv2.destroyAllWindows()
print("ปิดโปรแกรม")