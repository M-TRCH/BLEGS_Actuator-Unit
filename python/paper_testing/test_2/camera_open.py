import cv2
import time

def main():
    # เลือก Index ของกล้อง (ปกติถ้ามีกล้องติดเครื่องจะเป็น 1 หรือ 2 ลองปรับเปลี่ยนดูครับ)
    # ใช้ cv2.CAP_DSHOW เพื่อเข้าถึงฮาร์ดแวร์โดยตรงบน Windows รองรับการปรับ Resolution สูง
    camera_index = 1 
    cap = cv2.VideoCapture(camera_index, cv2.CAP_DSHOW)
    
    if not cap.isOpened():
        print(f"❌ ไม่สามารถเปิดอุปกรณ์ที่ Index {camera_index} ได้ ลองเปลี่ยนเลข Index")
        return

    # ---------------------------------------------------------
    # บังคับตั้งค่าพารามิเตอร์สัญญาณภาพเป็น 4K Uncompressed
    # ---------------------------------------------------------
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 3840)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 2160)
    
    # บังคับรูปแบบการส่งข้อมูลเป็น NV12 (ภาพดิบ ไม่ผ่าน MJPEG ย้วยๆ)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'NV12'))

    # อ่านค่าที่ตั้งได้จริงจากฮาร์ดแวร์มาตรวจสอบ
    actual_w = cap.get(cv2.CAP_PROP_FRAME_WIDTH)
    actual_h = cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
    actual_fourcc = int(cap.get(cv2.CAP_PROP_FOURCC))
    codec_str = "".join([chr((actual_fourcc >> 8 * i) & 0xFF) for i in range(4)])

    print("-" * 50)
    print(f"📡 สัญญาณอินพุตที่ตรวจพบ: {actual_w:.0f} x {actual_h:.0f}")
    print(f"🎥 Video Codec (Format): {codec_str}")
    print("-" * 50)
    print("กดปุ่ม 'q' บนคีย์บอร์ดเพื่อปิดหน้าต่างทดสอบ")

    # ตัวแปรสำหรับคำนวณ FPS หน้างานจริง
    prev_time = 0
    
    while True:
        ret, frame = cap.read()
        if not ret:
            print("⚠️ ไม่สามารถรับเฟรมภาพจาก Capture Card ได้")
            break

        # คำนวณ Real-time FPS
        current_time = time.time()
        fps = 1 / (current_time - prev_time) if (current_time - prev_time) > 0 else 0
        prev_time = current_time

        # ย่อขนาดหน้าจอแสดงผลลงมาเฉพาะตอนเปิดหน้าต่างโชว์ (Preview) 
        # เพื่อไม่ให้หน่วงทรัพยากรเครื่อง แต่ตัวแปร frame จริงยังคงเป็น 4K คมกริบ
        display_frame = cv2.resize(frame, (1280, 720))

        # ใส่ข้อความสถานะลงบนหน้าจอ Preview
        status_text = f"Res: {actual_w:.0f}x{actual_h:.0f} | Codec: {codec_str} | FPS: {fps:.1f}"
        cv2.putText(display_frame, status_text, (20, 40), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2, cv2.LINE_AA)

        # แสดงหน้าต่างภาพ
        cv2.imshow("Lumix G9 II + Cam Link 4K Verification", display_frame)

        # รอการกดปุ่ม 'q' เพื่อออกจากลูป
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()
    print("🏁 ปิดการทำงานระบบทดสอบเรียบร้อย")

if __name__ == "__main__":
    main()