import cv2

cap = cv2.VideoCapture(1)  # ใช้กล้องตัวแรก (0)

width, height = 1920, 1080

cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
cap.set(cv2.CAP_PROP_FPS, 60)  # ตั้งเฟรมเรทเป็น 60 FPS

# แสดงความละเอียดและเฟรมเรทจริงที่ได้
actual_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
actual_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
actual_fps = cap.get(cv2.CAP_PROP_FPS)

print(f"ความละเอียดจริง: {actual_width}x{actual_height}")
print(f"เฟรมเรทจริง: {actual_fps} FPS")
print("กด 'q' เพื่อออก")
print("-" * 40)

while True:
    ret, frame = cap.read()
    if not ret:
        break

    cv2.imshow('Camera', frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):  # กด q
        break

cap.release()
cv2.destroyAllWindows()