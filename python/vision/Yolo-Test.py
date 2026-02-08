from ultralytics import YOLO
import cv2
import time

# 1. โหลดโมเดล YOLOv8 Nano (ตัวเล็กสุด เร็วสุด)
# ครั้งแรกมันจะโหลดไฟล์จากเน็ตให้อัตโนมัติ
model = YOLO('yolov8n.pt') 

print("🚀 Starting YOLOv8 CPU Benchmark...")

# 2. เปิดกล้อง (หรือจะใส่ path รูปภาพ/วิดีโอ ก็ได้)
cap = cv2.VideoCapture(0) # 0 คือกล้องเว็บแคมเครื่อง

# ตั้งค่าความละเอียดให้เหมือนงานจริง
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

prev_time = 0

while True:
    ret, frame = cap.read()
    if not ret: break

    # 3. รันโมเดล (Inference)
    # device='cpu' เพื่อบังคับใช้ CPU (จริงๆ มัน auto อยู่แล้วถ้าไม่มี NVIDIA)
    start = time.time()
    results = model(frame, verbose=False, device='cpu')
    end = time.time()

    # 4. คำนวณ FPS
    fps = 1 / (end - start)
    
    # วาดผลลัพธ์ลงภาพ
    annotated_frame = results[0].plot()
    
    # โชว์ FPS
    cv2.putText(annotated_frame, f"CPU FPS: {fps:.2f}", (10, 30), 
                cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

    cv2.imshow("YOLOv8 CPU Test", annotated_frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()