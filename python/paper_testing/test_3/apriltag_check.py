import os

import cv2
import cv2.aruco as aruco

# 1. ระบุพาธไฟล์วิดีโอที่บันทึกมา (0, 1 หรือ 2 kg)
video_path = r'D:\THESIS\walk_test\walk_0kg.MOV'
cap = cv2.VideoCapture(video_path)

if not os.path.exists(video_path):
    raise FileNotFoundError(f"Video file not found: {video_path}")

if not cap.isOpened():
    raise RuntimeError(f"Cannot open video file: {video_path}")

# 2. ตั้งค่าตัวตรวจจับ ArUco ให้ตรงกับแท็กที่สร้างจาก Create-AR-Tag.py
ARUCO_DICT = aruco.DICT_6X6_250
dictionary = aruco.getPredefinedDictionary(ARUCO_DICT)
detector_params = aruco.DetectorParameters()
aruco_detector = aruco.ArucoDetector(dictionary, detector_params)

# เก็บประวัติว่าวิดีโอนี้เคยเจอ ID อะไรบ้าง
unique_ids_found = set()
FRAME_STEP = 2  # ประมวลผลทุก 2 เฟรม เพื่อลดภาระตอนวิดีโอความละเอียดสูง


def resize_to_fit(frame, max_width, max_height):
    height, width = frame.shape[:2]
    scale = min(max_width / width, max_height / height, 1.0)

    if scale == 1.0:
        return frame

    new_width = int(width * scale)
    new_height = int(height * scale)
    return cv2.resize(frame, (new_width, new_height))

print("กำลังเริ่มตรวจสอบวิดีโอ... (กดปุ่ม 'q' บนคีย์บอร์ดเพื่อหยุด)")

while cap.isOpened():
    ret, frame = cap.read()
    if not ret:
        print("จบไฟล์วิดีโอ")
        break

    frame_index = int(cap.get(cv2.CAP_PROP_POS_FRAMES))
    if frame_index % FRAME_STEP != 0:
        continue

    # 3. ArUco ต้องการภาพขาวดำ (Grayscale) ในการประมวลผล
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # 4. ตรวจจับ Tag
    corners, ids, _ = aruco_detector.detectMarkers(gray)

    current_frame_ids = []

    # 5. วาดกรอบและจุดศูนย์กลางลงบนเฟรม
    if ids is not None:
        aruco.drawDetectedMarkers(frame, corners, ids)

    for i, marker_id in enumerate(ids.flatten() if ids is not None else []):
        tag_id = int(marker_id)
        current_frame_ids.append(tag_id)
        unique_ids_found.add(tag_id)

        # ดึงมุมทั้ง 4 ของ Tag
        (ptA, ptB, ptC, ptD) = corners[i][0]
        ptB = (int(ptB[0]), int(ptB[1]))
        ptC = (int(ptC[0]), int(ptC[1]))
        ptD = (int(ptD[0]), int(ptD[1]))
        ptA = (int(ptA[0]), int(ptA[1]))

        # วาดเส้นขอบสีเขียว
        cv2.line(frame, ptA, ptB, (0, 255, 0), 2)
        cv2.line(frame, ptB, ptC, (0, 255, 0), 2)
        cv2.line(frame, ptC, ptD, (0, 255, 0), 2)
        cv2.line(frame, ptD, ptA, (0, 255, 0), 2)

        # วาดจุดสีแดงตรงกลาง
        cX = int((ptA[0] + ptB[0] + ptC[0] + ptD[0]) / 4)
        cY = int((ptA[1] + ptB[1] + ptC[1] + ptD[1]) / 4)
        cv2.circle(frame, (cX, cY), 5, (0, 0, 255), -1)

        # เขียนเลข ID กํากับไว้ด้านบน
        cv2.putText(frame, f"ID: {tag_id}", (ptA[0], ptA[1] - 15),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)

    # แสดงผล ID ที่เจอในเฟรมปัจจุบันที่มุมซ้ายบนของจอ
    cv2.putText(frame, f"Visible IDs: {current_frame_ids}", (20, 40),
                cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 0), 2)

    # 6. ย่อภาพเฉพาะตอนแสดงผล เพื่อรองรับวิดีโอแนวตั้ง 4K โดยไม่บิดภาพ
    if frame.shape[0] > frame.shape[1]:
        frame_resized = resize_to_fit(frame, 540, 960)
    else:
        frame_resized = resize_to_fit(frame, 960, 540)
    cv2.imshow('AprilTag Check', frame_resized)

    # กด 'q' เพื่อออก
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()

# สรุปผล
print("\n--- สรุปผลการตรวจสอบ ---")
print(f"ID ทั้งหมดที่สามารถมองเห็นได้ในวิดีโอนี้: {list(unique_ids_found)}")