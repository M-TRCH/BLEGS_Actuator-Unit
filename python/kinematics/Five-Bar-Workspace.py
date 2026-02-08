import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Circle

# --- 1. กำหนดค่าคงที่ของหุ่นยนต์ (ตามที่เราคุยกัน) ---
L1 = 105.0  # ความยาวลิงก์ขับ (AC และ BD)
L2 = 185.0  # ความยาวลิงก์ต่อเสมือน (CF และ DF)
BASE_HALF = 42.5 # ครึ่งหนึ่งของความยาวฐาน (85.0 / 2)

# จุดหมุนของมอเตอร์
POS_A = np.array([-BASE_HALF, 0])
POS_B = np.array([BASE_HALF, 0])

# รัศมีการทำงานของแขนแต่ละข้าง
R_MAX = L1 + L2  # 290.0
R_MIN = abs(L1 - L2) # 80.0

# --- 2. สร้าง Grid สำหรับตรวจสอบ ---
# กำหนดขอบเขตและมติของ Grid
x_min, x_max = -350, 350
y_min, y_max = -350, 50 # (เน้นพื้นที่ด้านล่าง)
RESOLUTION = 500 # ยิ่งละเอียด ยิ่งช้าแต่สวยขึ้น

# สร้างพิกัด x และ y
x = np.linspace(x_min, x_max, RESOLUTION)
y = np.linspace(y_min, y_max, RESOLUTION)

# สร้าง Grid 2 มิติ (X, Y)
# X และ Y จะเป็น 2D arrays ขนาด 500x500
X, Y = np.meshgrid(x, y)

# --- 3. คำนวณ Workspace (Vectorized) ---

# คำนวณระยะห่างจากมอเตอร์ A ไปยัง *ทุกจุด* ใน Grid
dist_A = np.sqrt((X - POS_A[0])**2 + (Y - POS_A[1])**2)
# คำนวณระยะห่างจากมอเตอร์ B ไปยัง *ทุกจุด* ใน Grid
dist_B = np.sqrt((X - POS_B[0])**2 + (Y - POS_B[1])**2)

# ตรวจสอบว่าแขนซ้ายเอื้อมถึงหรือไม่ (ได้ 2D boolean array)
reach_A = (dist_A <= R_MAX) & (dist_A >= R_MIN)
# ตรวจสอบว่าแขนขวาเอื้อมถึงหรือไม่
reach_B = (dist_B <= R_MAX) & (dist_B >= R_MIN)

# Workspace คือส่วนที่ทับซ้อนกัน (Intersection)
# (True เมื่อทั้งคู่เป็น True)
# ** เพิ่มเงื่อนไข Y <= 0 เพราะกลไกทำงานด้านล่างเท่านั้น (Q3, Q4) **
Workspace = reach_A & reach_B & (Y <= 0)

# --- 3.5 คำนวณจุดที่เสี่ยงเกิด Singularities ---
# กำหนด Tolerance สำหรับการตรวจจับ Singularity
SINGULAR_TOL = 15.0  # mm (ระยะห่างจากเงื่อนไข singular)

# 1. Singularity ประเภท "แขนยืดเต็มที่" (Extended Configuration)
extended_A = (dist_A >= R_MAX - SINGULAR_TOL) & (dist_A <= R_MAX)
extended_B = (dist_B >= R_MAX - SINGULAR_TOL) & (dist_B <= R_MAX)
singular_extended = (extended_A | extended_B) & Workspace

# 2. Singularity ประเภท "แขนพับเข้า" (Folded Configuration)
folded_A = (dist_A >= R_MIN) & (dist_A <= R_MIN + SINGULAR_TOL)
folded_B = (dist_B >= R_MIN) & (dist_B <= R_MIN + SINGULAR_TOL)
singular_folded = (folded_A | folded_B) & Workspace

# 3. Singularity ประเภท "อยู่บนเส้นระหว่างมอเตอร์" (Collinear Configuration)
# ตรวจจับจุดที่ Y ใกล้ 0 และ X อยู่ระหว่างมอเตอร์
collinear = (np.abs(Y) <= SINGULAR_TOL) & (X >= POS_A[0]) & (X <= POS_B[0])
singular_collinear = collinear & Workspace

# รวม Singularities ทั้งหมด
singular_zones = singular_extended | singular_folded | singular_collinear

# --- 4. พล็อตผลลัพธ์ด้วย Matplotlib ---
print("กำลังพล็อต Workspace...")
plt.figure(figsize=(10, 10))
ax = plt.gca()

# พล็อตพื้นที่ Workspace ที่คำนวณได้
# (ใช้ contourf เพื่อวาดพื้นที่ที่เป็น True)
plt.contourf(X, Y, Workspace, levels=[0.5, 1.5], colors=['#88c999'], alpha=0.7)

# พล็อต Singularity Zones
plt.contourf(X, Y, singular_extended, levels=[0.5, 1.5], colors=['#ff9999'], alpha=0.6)
plt.contourf(X, Y, singular_folded, levels=[0.5, 1.5], colors=['#ffcc99'], alpha=0.6)
plt.contourf(X, Y, singular_collinear, levels=[0.5, 1.5], colors=['#ff99ff'], alpha=0.6)

# วาดวงกลมแสดงขอบเขตของแขนแต่ละข้าง (เพื่อความเข้าใจ)
circle_A_outer = Circle(tuple(POS_A), R_MAX, color='blue', fill=False, ls=':', label='Left Arm Range')
circle_A_inner = Circle(tuple(POS_A), R_MIN, color='blue', fill=False, ls=':')
circle_B_outer = Circle(tuple(POS_B), R_MAX, color='red', fill=False, ls=':', label='Right Arm Range')
circle_B_inner = Circle(tuple(POS_B), R_MIN, color='red', fill=False, ls=':')

ax.add_patch(circle_A_outer)
ax.add_patch(circle_A_inner)
ax.add_patch(circle_B_outer)
ax.add_patch(circle_B_inner)

# พล็อตตำแหน่งมอเตอร์
plt.plot(POS_A[0], POS_A[1], 'bo', markersize=8, label=f'Motor A {POS_A}')
plt.plot(POS_B[0], POS_B[1], 'ro', markersize=8, label=f'Motor B {POS_B}')

# --- ตกแต่งกราฟ ---
plt.title('Robot Leg Workspace (5-Bar Linkage)')
plt.xlabel('X Position (mm)')
plt.ylabel('Y Position (mm)')
plt.grid(True, linestyle='--', alpha=0.5)
plt.legend()
# !! สำคัญมาก: ทำให้แกน X และ Y มีสเกลเท่ากัน
plt.axis('equal') 
plt.xlim(x_min, x_max)
plt.ylim(y_min, y_max)

plt.show()

print("พล็อตเสร็จสิ้น")