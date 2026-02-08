"""
ArUco Marker Detection with Multi-threading Optimization
=========================================================
โปรแกรมตรวจจับ ArUco Markers แบบ Real-time พร้อม Pose Estimation
ออกแบบมาเพื่อประสิทธิภาพสูงสุดด้วย Multi-threading

Author: BLEGS Analysis Unit
Version: 2.0
"""

import cv2
import numpy as np
import threading
import queue
import time
import os
from dataclasses import dataclass
from typing import Optional, List, Tuple

# =============================================================================
# CONFIGURATION
# =============================================================================

@dataclass
class Config:
    """การตั้งค่าทั้งหมดของโปรแกรม"""
    
    # --- Camera Settings ---
    CAMERA_INDEX: int = 1              # Index ของกล้อง (0=default, 1=external)
    CAMERA_WIDTH: int = 1920           # ความกว้างกล้อง (pixels)
    CAMERA_HEIGHT: int = 1080          # ความสูงกล้อง (pixels)
    CAMERA_FPS: int = 60               # FPS ของกล้อง
    
    # --- Display Settings ---
    DISPLAY_WIDTH: int = 1280          # ความกว้างหน้าจอแสดงผล
    DISPLAY_HEIGHT: int = 720          # ความสูงหน้าจอแสดงผล
    DISPLAY_MARGIN: float = 0.9        # เหลือขอบ 10%
    
    # --- ArUco Settings ---
    ARUCO_DICT: int = cv2.aruco.DICT_6X6_250  # Dictionary ของ ArUco
    MARKER_SIZE_METERS: float = 0.024  # ขนาด Marker (เมตร) = 24mm
    
    # --- Performance Settings ---
    DETECTION_SCALE: float = 0.5       # ย่อ frame ก่อน detect (0.5 = 50%)
    QUEUE_SIZE: int = 2                # ขนาด Queue (เล็ก = latency ต่ำ)
    FPS_UPDATE_INTERVAL: float = 0.5   # อัพเดท FPS ทุกกี่วินาที


# =============================================================================
# ARUCO DETECTOR
# =============================================================================

class ArucoDetector:
    """จัดการการตรวจจับ ArUco Markers"""
    
    def __init__(self, config: Config):
        self.config = config
        self._setup_detector()
        self._setup_object_points()
    
    def _setup_detector(self) -> None:
        """ตั้งค่า ArUco Detector พร้อม parameters ที่ optimize แล้ว"""
        aruco_dict = cv2.aruco.getPredefinedDictionary(self.config.ARUCO_DICT)
        params = cv2.aruco.DetectorParameters()
        
        # Optimize parameters สำหรับความเร็วสูงสุด
        params.adaptiveThreshWinSizeMin = 3
        params.adaptiveThreshWinSizeMax = 23
        params.adaptiveThreshWinSizeStep = 10
        params.minMarkerPerimeterRate = 0.03
        params.maxMarkerPerimeterRate = 4.0
        params.polygonalApproxAccuracyRate = 0.05
        params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_NONE
        
        self.detector = cv2.aruco.ArucoDetector(aruco_dict, params)
    
    def _setup_object_points(self) -> None:
        """Pre-compute object points (ประหยัดเวลาไม่ต้องสร้างใหม่ทุก frame)"""
        half_size = self.config.MARKER_SIZE_METERS / 2
        self.obj_points = np.array([
            [-half_size,  half_size, 0],
            [ half_size,  half_size, 0],
            [ half_size, -half_size, 0],
            [-half_size, -half_size, 0]
        ], dtype=np.float32)
    
    def detect(self, gray_frame: np.ndarray) -> Tuple[list, Optional[np.ndarray]]:
        """ตรวจจับ markers จาก grayscale frame"""
        corners, ids, _ = self.detector.detectMarkers(gray_frame)
        return corners, ids
    
    def estimate_pose(self, corners: np.ndarray, camera_matrix: np.ndarray, 
                      dist_coeffs: np.ndarray) -> Optional[dict]:
        """คำนวณ pose ของ marker"""
        success, rvec, tvec = cv2.solvePnP(
            self.obj_points,
            corners,
            camera_matrix,
            dist_coeffs,
            flags=cv2.SOLVEPNP_IPPE_SQUARE
        )
        
        if success:
            return {
                'rvec': rvec,
                'tvec': tvec,
                'center': corners.mean(axis=0).astype(np.int32)
            }
        return None


# =============================================================================
# CAMERA CALIBRATION
# =============================================================================

class CameraCalibration:
    """จัดการ Camera Calibration Parameters"""
    
    def __init__(self, calib_file: str):
        self.camera_matrix: Optional[np.ndarray] = None
        self.dist_coeffs: Optional[np.ndarray] = None
        self._load_calibration(calib_file)
    
    def _load_calibration(self, calib_file: str) -> None:
        """โหลด calibration parameters จากไฟล์"""
        try:
            data = np.load(calib_file)
            self.camera_matrix = data['mtx']
            self.dist_coeffs = data['dist']
            print(f"✓ โหลด Camera Parameters จาก: {calib_file}")
            print(f"\nCamera Matrix (K):\n{self.camera_matrix}")
            print(f"\nDistortion Coefficients (D):\n{self.dist_coeffs}")
        except FileNotFoundError:
            raise FileNotFoundError(f"ไม่พบไฟล์ calibration: {calib_file}")


# =============================================================================
# FPS COUNTER
# =============================================================================

class FPSCounter:
    """นับและคำนวณ FPS แบบ Thread-safe"""
    
    def __init__(self, update_interval: float = 0.5):
        self.update_interval = update_interval
        self.lock = threading.Lock()
        self.counter = 0
        self.fps = 0.0
        self.last_update = time.time()
    
    def increment(self) -> None:
        """เพิ่ม counter (thread-safe)"""
        with self.lock:
            self.counter += 1
    
    def update(self) -> float:
        """อัพเดท FPS และ return ค่าปัจจุบัน"""
        current_time = time.time()
        elapsed = current_time - self.last_update
        
        if elapsed >= self.update_interval:
            with self.lock:
                self.fps = self.counter / elapsed if elapsed > 0 else 0
                self.counter = 0
            self.last_update = current_time
        
        return self.fps


# =============================================================================
# MAIN APPLICATION
# =============================================================================

class ArUcoTrackerApp:
    """แอปพลิเคชันหลักสำหรับ ArUco Tracking"""
    
    def __init__(self, config: Config):
        self.config = config
        self.running = False
        self.verbose = False
        
        # Initialize components
        self._init_calibration()
        self._init_camera()
        self._init_detector()
        self._init_display()
        self._init_threading()
        self._init_fps_counters()
    
    def _init_calibration(self) -> None:
        """โหลด camera calibration"""
        script_dir = os.path.dirname(os.path.abspath(__file__))
        calib_file = os.path.join(script_dir, "camera_calibration", 
                                  "camera_params_ef1635f4l_1080p60.npz")
        self.calibration = CameraCalibration(calib_file)
    
    def _init_camera(self) -> None:
        """เปิดและตั้งค่ากล้อง"""
        self.cap = cv2.VideoCapture(self.config.CAMERA_INDEX)
        if not self.cap.isOpened():
            raise RuntimeError("ไม่สามารถเปิดกล้องได้")
        
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.config.CAMERA_WIDTH)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.config.CAMERA_HEIGHT)
        self.cap.set(cv2.CAP_PROP_FPS, self.config.CAMERA_FPS)
        
        actual_w = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        actual_h = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        print(f"\n✓ กล้องเปิดแล้ว: {actual_w}x{actual_h}")
    
    def _init_detector(self) -> None:
        """สร้าง ArUco detector"""
        self.detector = ArucoDetector(self.config)
    
    def _init_display(self) -> None:
        """คำนวณขนาดการแสดงผล"""
        actual_w = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        actual_h = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        
        scale = min(
            (self.config.DISPLAY_WIDTH * self.config.DISPLAY_MARGIN) / actual_w,
            (self.config.DISPLAY_HEIGHT * self.config.DISPLAY_MARGIN) / actual_h
        )
        
        self.output_size = (int(actual_w * scale), int(actual_h * scale))
        print(f"✓ ขนาดแสดงผล: {self.output_size[0]}x{self.output_size[1]}")
    
    def _init_threading(self) -> None:
        """สร้าง queues และ threading events"""
        self.frame_queue = queue.Queue(maxsize=self.config.QUEUE_SIZE)
        self.result_queue = queue.Queue(maxsize=self.config.QUEUE_SIZE)
        self.stop_event = threading.Event()
    
    def _init_fps_counters(self) -> None:
        """สร้าง FPS counters"""
        interval = self.config.FPS_UPDATE_INTERVAL
        self.camera_fps = FPSCounter(interval)
        self.processing_fps = FPSCounter(interval)
        self.display_fps = FPSCounter(interval)
    
    # -------------------------------------------------------------------------
    # THREAD FUNCTIONS
    # -------------------------------------------------------------------------
    
    def _camera_thread(self) -> None:
        """Thread สำหรับรับภาพจากกล้อง"""
        while not self.stop_event.is_set():
            ret, frame = self.cap.read()
            if not ret:
                continue
            
            self.camera_fps.increment()
            
            # ถ้า queue เต็ม ให้ทิ้ง frame เก่า
            if self.frame_queue.full():
                try:
                    self.frame_queue.get_nowait()
                except queue.Empty:
                    pass
            
            try:
                self.frame_queue.put_nowait(frame)
            except queue.Full:
                pass
        
        print("Camera thread หยุดทำงาน")
    
    def _processing_thread(self) -> None:
        """Thread สำหรับประมวลผล ArUco detection"""
        scale = self.config.DETECTION_SCALE
        
        while not self.stop_event.is_set():
            try:
                frame = self.frame_queue.get_nowait()
            except queue.Empty:
                time.sleep(0.001)
                continue
            
            # Downsample frame สำหรับ detection
            if scale < 1.0:
                small = cv2.resize(frame, None, fx=scale, fy=scale, 
                                   interpolation=cv2.INTER_LINEAR)
                gray = cv2.cvtColor(small, cv2.COLOR_BGR2GRAY)
            else:
                gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            
            # ตรวจจับ markers
            corners, ids = self.detector.detect(gray)
            
            # ปรับ corners กลับเป็น scale เดิม
            if scale < 1.0 and corners:
                corners = [c / scale for c in corners]
            
            # สร้างผลลัพธ์
            result = {'frame': frame, 'corners': corners, 'ids': ids}
            
            # คำนวณ pose สำหรับแต่ละ marker
            if ids is not None:
                markers_data = []
                for i, marker_id in enumerate(ids):
                    pose = self.detector.estimate_pose(
                        corners[i][0],
                        self.calibration.camera_matrix,
                        self.calibration.dist_coeffs
                    )
                    if pose:
                        pose['id'] = marker_id[0]
                        markers_data.append(pose)
                result['markers_data'] = markers_data
            
            self.processing_fps.increment()
            
            # ใส่ผลลัพธ์ใน queue
            if self.result_queue.full():
                try:
                    self.result_queue.get_nowait()
                except queue.Empty:
                    pass
            
            try:
                self.result_queue.put_nowait(result)
            except queue.Full:
                pass
        
        print("Processing thread หยุดทำงาน")
    
    # -------------------------------------------------------------------------
    # RENDERING
    # -------------------------------------------------------------------------
    
    def _render_markers(self, frame: np.ndarray, result: dict) -> None:
        """วาด markers และข้อมูลลงบน frame"""
        corners = result.get('corners')
        ids = result.get('ids')
        markers_data = result.get('markers_data', [])
        
        if ids is None or not markers_data:
            return
        
        # วาดกรอบ marker
        cv2.aruco.drawDetectedMarkers(frame, corners, ids)
        
        # วาดแกนและข้อมูลแต่ละ marker
        for marker in markers_data:
            rvec, tvec = marker['rvec'], marker['tvec']
            center = marker['center']
            marker_id = marker['id']
            
            # วาดแกน XYZ
            cv2.drawFrameAxes(
                frame, 
                self.calibration.camera_matrix,
                self.calibration.dist_coeffs,
                rvec, tvec, 0.025
            )
            
            # แสดง ID และระยะทาง
            z_distance = tvec.ravel()[2]
            cv2.putText(
                frame, f"ID:{marker_id} Z:{z_distance:.2f}m",
                tuple(center), cv2.FONT_HERSHEY_SIMPLEX,
                0.5, (0, 255, 0), 2
            )
            
            # Verbose output
            if self.verbose:
                t = tvec.ravel()
                print(f"ID:{marker_id} X:{t[0]:.3f} Y:{t[1]:.3f} Z:{t[2]:.3f}m")
    
    def _render_fps(self, frame: np.ndarray) -> None:
        """แสดง FPS ที่มุมขวาบน"""
        cam = self.camera_fps.update()
        proc = self.processing_fps.update()
        disp = self.display_fps.update()
        
        fps_text = f"CAM:{cam:.0f} PROC:{proc:.0f} DISP:{disp:.0f}"
        h, w = frame.shape[:2]
        
        cv2.rectangle(frame, (w-280, 5), (w-5, 35), (0, 0, 0), -1)
        cv2.putText(frame, fps_text, (w-275, 28), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 2)
    
    # -------------------------------------------------------------------------
    # MAIN LOOP
    # -------------------------------------------------------------------------
    
    def run(self) -> None:
        """รันโปรแกรมหลัก"""
        print("\n" + "="*50)
        print("เริ่ม Multi-threading ArUco Detection")
        print("กด 'v' = verbose mode, 'q' = ออก")
        print("="*50 + "\n")
        
        # เริ่ม threads
        cam_thread = threading.Thread(target=self._camera_thread, daemon=True)
        proc_thread = threading.Thread(target=self._processing_thread, daemon=True)
        
        cam_thread.start()
        proc_thread.start()
        
        self.running = True
        last_result = None
        
        try:
            while self.running:
                # รับผลลัพธ์จาก queue
                got_new = False
                try:
                    result = self.result_queue.get_nowait()
                    last_result = result
                    got_new = True
                except queue.Empty:
                    if last_result:
                        result = last_result
                    else:
                        time.sleep(0.001)
                        continue
                
                frame = result['frame'].copy()  # Copy เพื่อไม่ให้กระทบ frame อื่น
                
                # Render
                self._render_markers(frame, result)
                self._render_fps(frame)
                
                # นับ display FPS
                if got_new:
                    self.display_fps.increment()
                
                # Resize และแสดงผล
                display_frame = cv2.resize(
                    frame, self.output_size, 
                    interpolation=cv2.INTER_NEAREST
                )
                cv2.imshow("ArUco Detection", display_frame)
                
                # จัดการ keyboard input
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q'):
                    self.running = False
                elif key == ord('v'):
                    self.verbose = not self.verbose
                    print(f"Verbose mode: {'ON' if self.verbose else 'OFF'}")
        
        finally:
            self._cleanup(cam_thread, proc_thread)
    
    def _cleanup(self, *threads) -> None:
        """ทำความสะอาดและปิดโปรแกรม"""
        print("\nกำลังปิดโปรแกรม...")
        self.stop_event.set()
        
        for t in threads:
            t.join(timeout=1.0)
        
        self.cap.release()
        cv2.destroyAllWindows()
        print("✓ ปิดโปรแกรมเรียบร้อย")


# =============================================================================
# ENTRY POINT
# =============================================================================

def main():
    """จุดเริ่มต้นโปรแกรม"""
    try:
        config = Config()
        app = ArUcoTrackerApp(config)
        app.run()
    except FileNotFoundError as e:
        print(f"❌ Error: {e}")
        print("กรุณาทำ camera calibration ก่อนใช้งาน")
    except RuntimeError as e:
        print(f"❌ Error: {e}")
    except KeyboardInterrupt:
        print("\n⚠ โปรแกรมถูกหยุดโดยผู้ใช้")


if __name__ == "__main__":
    main()
