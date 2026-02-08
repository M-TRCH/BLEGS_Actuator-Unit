import cv2
import numpy as np
import time

class GreenCircleDetector:
    def __init__(self, crop_size=720):
        # กำหนดช่วงกว้างของสีเขียวนีออนใน HSV
        # สีเป้าหมาย RGB = (165, 224, 155) แปลงเป็น HSV ≈ (98, 81, 88) (OpenCV scale: H=49, S=80, V=224)
        # กำหนดช่วงเผื่อ H, S, V เพื่อให้ตรวจจับได้แม่นยำและเผื่อความคลาดเคลื่อนของแสง
        h, s, v = 49, 80, 224  # จาก #A5E09B
        h_margin = 120   # ขยายช่วงสีเพิ่มอีก 20% (H ±120)
        s_margin = 98    # S ±98 (เพิ่มจาก 82)
        v_margin = 56    # V ±56 (เพิ่มจาก 47)
        self.lower_green = np.array([max(0, h - h_margin), max(0, s - s_margin), max(0, v - v_margin)])
        self.upper_green = np.array([min(179, h + h_margin), min(255, s + s_margin), min(255, v + v_margin)])
        
        # การตั้งค่าขนาดสำหรับการ crop
        self.crop_size = crop_size
        
        # พารามิเตอร์สำหรับ HoughCircles
        self.dp = 1.2
        self.min_dist = 200     # ระยะห่างขั้นต่ำระหว่างวงกลม
        self.param1 = 50        # พารามิเตอร์แรกสำหรับ Canny edge detection
        self.param2 = 20        # พารามิเตอร์ที่สองสำหรับ HoughCircles (เพิ่ม threshold เพื่อลด noise)
        self.min_radius = 42    
        self.max_radius = 48
        
        # สำหรับคำนวณ FPS
        self.fps_counter = 0
        self.fps_start_time = time.time()
        self.current_fps = 0
        
        # สำหรับการอัดวิดีโอ
        self.video_writer = None
        self.is_recording = False
        self.recording_start_time = None
    
    def crop_center(self, frame):
        """ตัดภาพจากตรงกลางเป็นขนาดสี่เหลี่ยมจัตุรัส"""
        height, width = frame.shape[:2]
        
        # คำนวณตำแหน่งเริ่มต้นสำหรับการ crop ตรงกลาง
        start_x = max(0, (width - self.crop_size) // 2)
        start_y = max(0, (height - self.crop_size) // 2)
        end_x = min(width, start_x + self.crop_size)
        end_y = min(height, start_y + self.crop_size)
        
        # ตัดภาพ
        cropped = frame[start_y:end_y, start_x:end_x]
        
        # ปรับขนาดให้เป็นขนาดที่ต้องการหากไม่พอดี
        if cropped.shape[0] != self.crop_size or cropped.shape[1] != self.crop_size:
            cropped = cv2.resize(cropped, (self.crop_size, self.crop_size))
            
        return cropped
    
    def set_crop_size(self, new_size):
        """เปลี่ยนขนาดการ crop"""
        if 200 <= new_size <= 1080:  # จำกัดขนาดที่สมเหตุสมผล
            self.crop_size = new_size
            print(f"เปลี่ยนขนาด crop เป็น: {self.crop_size}x{self.crop_size}")
            return True
        else:
            print(f"ขนาด crop ต้องอยู่ระหว่าง 200-1080 พิกเซล")
            return False
    
    def preprocess_frame(self, frame):
        """ปรับปรุงภาพก่อนการตรวจจับ"""
        # แปลงเป็น HSV
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        
        # สร้าง mask สำหรับสีเขียว
        mask = cv2.inRange(hsv, self.lower_green, self.upper_green)
        
        # ใช้ Gaussian Blur ลด noise
        mask = cv2.GaussianBlur(mask, (5, 5), 0)
        
        # Morphological operations เพื่อปิดช่องว่างและกำจัด noise
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        
        return mask
    
    def detect_circles(self, mask):
        """ตรวจจับวงกลมใน mask"""
        circles = cv2.HoughCircles(
            mask, 
            cv2.HOUGH_GRADIENT,
            dp=self.dp,
            minDist=self.min_dist,
            param1=self.param1,
            param2=self.param2,
            minRadius=self.min_radius,
            maxRadius=self.max_radius
        )
        
        return circles
    
    def draw_results(self, frame, circles, mask):
        """วาดผลลัพธ์บนเฟรม"""
        result_frame = frame.copy()
        
        if circles is not None:
            circles = np.round(circles[0, :]).astype("int")
            
            for (x, y, r) in circles:
                # วาดวงกลมที่ตรวจจับได้
                cv2.circle(result_frame, (x, y), r, (255, 0, 0), 3)
                cv2.circle(result_frame, (x, y), 2, (0, 0, 255), 3)
                
                # แสดงข้อมูลตำแหน่งและรัศมี
                cv2.putText(result_frame, f"R:{r} X:{x} Y:{y}", 
                           (x-50, y-r-10), cv2.FONT_HERSHEY_SIMPLEX, 
                           0.5, (255, 255, 255), 2)
        
        # แสดง FPS
        cv2.putText(result_frame, f"FPS: {self.current_fps:.1f}", 
                   (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 2)
        
        # แสดงขนาด crop
        cv2.putText(result_frame, f"Crop: {self.crop_size}x{self.crop_size}", 
                   (10, 110), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        
        # แสดงสถานะการอัดวิดีโอ
        if self.is_recording:
            recording_time = int(time.time() - self.recording_start_time) if self.recording_start_time else 0
            cv2.putText(result_frame, f"REC {recording_time:02d}s", 
                       (10, 70), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
            # วาดจุดสีแดงกระพริบ
            if int(time.time() * 2) % 2:  # กระพริบทุก 0.5 วินาที
                cv2.circle(result_frame, (200, 60), 8, (0, 0, 255), -1)
        
        return result_frame
    
    def calculate_fps(self):
        """คำนวณ FPS"""
        self.fps_counter += 1
        if self.fps_counter >= 30:
            end_time = time.time()
            self.current_fps = 30 / (end_time - self.fps_start_time)
            self.fps_counter = 0
            self.fps_start_time = time.time()
    
    def start_recording(self, width, height, fps=30):
        """เริ่มการอัดวิดีโอ"""
        if not self.is_recording:
            timestamp = int(time.time())
            filename = f'recorded_circles_{timestamp}.mp4'
            fourcc = cv2.VideoWriter_fourcc(*'mp4v')
            self.video_writer = cv2.VideoWriter(filename, fourcc, fps, (width, height))
            self.is_recording = True
            self.recording_start_time = time.time()
            print(f"เริ่มการอัดวิดีโอ: {filename}")
            return filename
        return None
    
    def stop_recording(self):
        """หยุดการอัดวิดีโอ"""
        if self.is_recording and self.video_writer:
            self.video_writer.release()
            self.video_writer = None
            self.is_recording = False
            recording_duration = int(time.time() - self.recording_start_time) if self.recording_start_time else 0
            self.recording_start_time = None
            print(f"หยุดการอัดวิดีโอแล้ว (ระยะเวลา: {recording_duration} วินาที)")
            return recording_duration
        return 0
    
    def record_frame(self, frame):
        """บันทึกเฟรมลงในวิดีโอ"""
        if self.is_recording and self.video_writer:
            self.video_writer.write(frame)

def main():
    # เปิดกล้อง
    cap = cv2.VideoCapture(0)
    
    # ตั้งค่าความละเอียดกล้องเป็น 1920x1080
    original_width, original_height = 1920, 1080
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, original_width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, original_height)
    cap.set(cv2.CAP_PROP_FPS, 30)  # ตั้งค่า FPS
    
    # ขนาดสำหรับการประมวลผล (crop)
    crop_size = 860
    
    # สร้าง detector
    detector = GreenCircleDetector(crop_size=crop_size)
    
    print("กำลังเปิดกล้อง... กด 'q' เพื่อออก, 's' เพื่อบันทึกภาพ")
    print("กด 'h' เพื่อแสดง/ซ่อน mask, 'c' เพื่อปรับตั้งค่า")
    print("กด 'r' เพื่อเริ่ม/หยุดการอัดวิดีโอ")
    print("กด '+' เพื่อเพิ่มขนาด crop, '-' เพื่อลดขนาด crop")
    print(f"Camera: {original_width}x{original_height} → Processing: {crop_size}x{crop_size} (center crop)")
    
    show_mask = False
    
    while True:
        ret, frame = cap.read()
        if not ret:
            print("ไม่สามารถอ่านข้อมูลจากกล้องได้")
            break
        
        # ปรับขนาดเฟรม จาก 1920x1080 และตัดเป็น 720x720 ตรงกลาง
        cropped_frame = detector.crop_center(frame)
        
        # ประมวลผลภาพ
        mask = detector.preprocess_frame(cropped_frame)
        circles = detector.detect_circles(mask)
        result_frame = detector.draw_results(cropped_frame, circles, mask)
        
        # คำนวณ FPS
        detector.calculate_fps()
        
        # แสดงผล
        cv2.imshow('Green Circle Detection', result_frame)
        
        # บันทึกเฟรมลงในวิดีโอ (ถ้ากำลังอัด)
        detector.record_frame(result_frame)
        
        if show_mask:
            cv2.imshow('Green Mask', mask)
        
        # จัดการ key input
        key = cv2.waitKey(1) & 0xFF
        
        if key == ord('q'):  # ออกจากโปรแกรม
            break
        elif key == ord('s'):  # บันทึกภาพ
            timestamp = int(time.time())
            cv2.imwrite(f'detected_circles_{timestamp}.jpg', result_frame)
            print(f"บันทึกภาพแล้ว: detected_circles_{timestamp}.jpg")
        elif key == ord('h'):  # แสดง/ซ่อน mask
            show_mask = not show_mask
            if not show_mask:
                cv2.destroyWindow('Green Mask')
        elif key == ord('c'):  # แสดงค่าปัจจุบัน
            print("\n--- การตั้งค่าปัจจุบัน ---")
            print(f"Original Camera Resolution: {original_width}x{original_height}")
            print(f"Crop Size: {crop_size}x{crop_size}")
            print(f"Green HSV Range: {detector.lower_green} - {detector.upper_green}")
            print(f"Circle Radius: {detector.min_radius} - {detector.max_radius}")
            print(f"Min Distance: {detector.min_dist}")
        elif key == ord('r'):  # เริ่ม/หยุดการอัดวิดีโอ
            if detector.is_recording:
                detector.stop_recording()
            else:
                detector.start_recording(detector.crop_size, detector.crop_size)
        elif key == ord('+') or key == ord('='):  # เพิ่มขนาด crop
            new_size = min(1080, detector.crop_size + 50)
            detector.set_crop_size(new_size)
        elif key == ord('-') or key == ord('_'):  # ลดขนาด crop
            new_size = max(200, detector.crop_size - 50)
            detector.set_crop_size(new_size)
    
    # หยุดการอัดวิดีโอก่อนปิดโปรแกรม
    if detector.is_recording:
        detector.stop_recording()
    
    # ปิดการใช้งาน
    cap.release()
    cv2.destroyAllWindows()
    print("ปิดกล้องแล้ว")

if __name__ == "__main__":
    main()