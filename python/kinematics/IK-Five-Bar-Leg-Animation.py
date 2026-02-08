import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

# --- 1. กำหนดพารามิเตอร์คงที่ของหุ่นยนต์ (หน่วย mm) ---
# พิกัดมอเตอร์
P_A = np.array([-42.5, 0.0])  # จุด A (มอเตอร์ 1, ซ้าย)
P_B = np.array([42.5, 0.0])   # จุด B (มอเตอร์ 2, ขวา)

# ความยาวลิงก์
L_AC = 105.0  # L1
L_BD = 105.0  # L2
L_CE = 145.0  # L3
L_DE = 145.0  # L4
L_EF = 40.0   # Offset

# อัตราส่วน Offset
OFFSET_RATIO_E = 37.0 / 29.0
OFFSET_RATIO_D = 8.0 / 29.0

# ตัวแปรเก่าเพื่อความเข้ากันได้
M1_X = P_A[0]
M1_Y = P_A[1]
M2_X = P_B[0]
M2_Y = P_B[1]
L1 = L_AC
L2 = L_BD
L3 = L_CE
L4 = L_DE

# --- 2. ฟังก์ชัน Forward Kinematics (FK) ---
def calculate_fk(thetas):
    """
    คำนวณ Forward Kinematics
    Input: thetas = [theta_A, theta_B] (radians)
    Output: P_F = [x_f, y_f] (mm)
    """
    theta_A = thetas[0]
    theta_B = thetas[1]
    
    # คำนวณพิกัดข้อเข่า P_C และ P_D
    P_C = P_A + np.array([L_AC * np.cos(theta_A), L_AC * np.sin(theta_A)])
    P_D = P_B + np.array([L_BD * np.cos(theta_B), L_BD * np.sin(theta_B)])

    # คำนวณพิกัด P_E (จุดตัดวงกลม)
    V_CD = P_D - P_C
    d = np.linalg.norm(V_CD)
    
    if d > (L_CE + L_DE) or d < abs(L_CE - L_DE) or d == 0:
        return np.array([np.nan, np.nan])

    a = (L_CE**2 - L_DE**2 + d**2) / (2 * d)
    h_squared = L_CE**2 - a**2
    if h_squared < 0:
        return np.array([np.nan, np.nan])
        
    h = np.sqrt(h_squared)
    
    v_d = V_CD / d
    v_perp = np.array([-v_d[1], v_d[0]])

    # เลือกท่าเข่าชี้ลง
    P_E = P_C + a * v_d - h * v_perp

    # คำนวณพิกัดปลายเท้า P_F
    P_F = (OFFSET_RATIO_E * P_E) - (OFFSET_RATIO_D * P_D)
    
    return P_F

# --- 3. ฟังก์ชันช่วย: หาจุดตัดของวงกลม 2 วง ---
def solve_circle_intersection(center1, radius1, center2, radius2, choose_lower=True):
    """
    หาจุดตัดของวงกลม 2 วง
    
    Input:
        center1 = [x1, y1] - ศูนย์กลางวงกลมที่ 1
        radius1 - รัศมีวงกลมที่ 1
        center2 = [x2, y2] - ศูนย์กลางวงกลมที่ 2
        radius2 - รัศมีวงกลมที่ 2
        choose_lower = True/False - เลือกจุดที่มี y น้อยกว่า (True) หรือมากกว่า (False)
    
    Output:
        [x, y] - จุดตัง หรือ [nan, nan] ถ้าไม่มีจุดตัด
    """
    
    V_12 = center2 - center1
    d = np.linalg.norm(V_12)
    
    # ตรวจสอบว่าวงกลมตัดกันหรือไม่
    if d > (radius1 + radius2) or d < abs(radius1 - radius2) or d == 0:
        return np.array([np.nan, np.nan])
    
    # คำนวณพิกัดจุดตัด
    a = (radius1**2 - radius2**2 + d**2) / (2 * d)
    h_squared = radius1**2 - a**2
    
    if h_squared < 0:
        return np.array([np.nan, np.nan])
    
    h = np.sqrt(h_squared)
    
    v_d = V_12 / d  # เวกเตอร์หนึ่งหน่วย
    v_perp = np.array([-v_d[1], v_d[0]])  # เวกเตอร์ตั้งฉาก
    
    # จุดตัด 2 จุด
    P_intersection_1 = center1 + a * v_d + h * v_perp
    P_intersection_2 = center1 + a * v_d - h * v_perp
    
    # เลือกจุดที่มี y น้อยกว่า (ท่าเข่าลง) หรือมากกว่า (ท่าเข่าขึ้น)
    if choose_lower:
        if P_intersection_2[1] < P_intersection_1[1]:
            return P_intersection_2
        else:
            return P_intersection_1
    else:
        if P_intersection_1[1] > P_intersection_2[1]:
            return P_intersection_1
        else:
            return P_intersection_2

# --- 4. ฟังก์ชัน IK แบบ Analytical ---
def calculate_ik_analytical(P_F_target, elbow_C_down=True, elbow_D_down=True):
    """
    คำนวณ Inverse Kinematics (IK) แบบ Analytical
    
    Input: 
        P_F_target = [x_f, y_f] (mm) - พิกัดเป้าหมายของปลายเท้า
        elbow_C_down = True/False - เลือกท่าข้อเข่า C (ซ้าย) "ลง" หรือ "ขึ้น"
        elbow_D_down = True/False - เลือกท่าข้อเข่า D (ขวา) "ลง" หรือ "ขึ้น"
    
    Output: 
        [theta_A, theta_B] (radians) หรือ [nan, nan] ถ้าไปไม่ถึง
    """
    
    (x_f, y_f) = P_F_target
    (x_a, y_a) = P_A
    (x_b, y_b) = P_B
    
    # คำนวณระยะจาก P_F ถึง P_D
    R_FD = 145.0 * 37.0 / 29.0  # ≈ 185.0 mm
    R_DB = L_BD  # = 105.0 mm
    
    # หา P_D จากการตัดกันของวงกลม 2 วง (สลับทิศ: elbow_D_down=True → เลือกข้างบน)
    P_D = solve_circle_intersection(P_F_target, R_FD, P_B, R_DB, not elbow_D_down)
    
    if np.isnan(P_D).any():
        return np.array([np.nan, np.nan])
    
    # หา P_E จากสมการ
    P_E = (29.0 * P_F_target + 8.0 * P_D) / 37.0
    
    # หา P_C จากการตัดกันของวงกลม 2 วง
    P_C = solve_circle_intersection(P_A, L_AC, P_E, L_CE, elbow_C_down)
    
    if np.isnan(P_C).any():
        return np.array([np.nan, np.nan])
    
    # คำนวณมุม theta_A และ theta_B
    V_AC = P_C - P_A
    V_BD = P_D - P_B
    
    theta_A = np.arctan2(V_AC[1], V_AC[0])
    theta_B = np.arctan2(V_BD[1], V_BD[0])
    
    return np.array([theta_A, theta_B])

# --- 2. สร้าง Walking Trajectory (วิถีการเดิน) ---
def generate_walking_trajectory(num_steps=60, lift_height=50, step_forward=50):
    """
    สร้างวิถีการเดินแบบ Elliptical Path (วงรี) - หมุนทวนเข็มนาฬิกา
    เป็นวิถีที่นิยมใช้ในหุ่นยนต์สี่ขาเพราะให้การเคลื่อนที่ที่นุ่มนวลและต่อเนื่อง
    อ้างอิงจากท่า Home (0, -200)
    
    num_steps: จำนวน frame ในแอนิเมชัน
    lift_height: แกนสั้นของวงรี (แกนตั้ง Y) - ค่าเริ่มต้น 50mm
    step_forward: แกนยาวของวงรี (แกนนอน X) - ค่าเริ่มต้น 50mm
    
    สูตร Ellipse: x = a*cos(t), y = b*sin(t)
    โดย a = step_forward (แกนยาว), b = lift_height (แกนสั้น)
    """
    trajectory = []
    home_y = -200  # ท่ายืนอ้างอิง (จุดศูนย์กลางของวงรี)
    
    # กำหนดพารามิเตอร์วงรี
    a = step_forward  # แกนยาว (แนวนอน)
    b = lift_height   # แกนสั้น (แนวตั้ง)
    
    for i in range(num_steps):
        # t เป็นพารามิเตอร์มุม (0 → 2π) สำหรับวงรี
        # เริ่มที่ t=0 จะอยู่ที่ตำแหน่ง (a, 0) คือด้านขวาของวงรี
        t = 2 * np.pi * i / num_steps
        
        # สูตรวงรีมาตรฐาน (หมุนทวนเข็มนาฬิกา)
        px = a * np.cos(t)         # แกน X: เคลื่อนที่ +a → 0 → -a → 0 → +a
        py = home_y + b * np.sin(t)  # แกน Y: อ้างอิงจาก home_y, เคลื่อนที่ขึ้น-ลง
        
        trajectory.append((px, py))
    
    return trajectory

# --- 5. ทดสอบ IK ที่ท่ายืน (Home Position) ---
print("--- 🤖 ทดสอบ Analytical IK ที่ท่ายืน P=(0, -200) ---")
target_Px, target_Py = 0, -200

# ใช้ Configuration 1: Down-Down (ท่ามาตรฐาน)
solution_rad = calculate_ik_analytical(np.array([target_Px, target_Py]), 
                                       elbow_C_down=True, elbow_D_down=True)

if not np.isnan(solution_rad).any():
    print(f"เป้าหมาย: ({target_Px}, {target_Py}) mm")
    print(f"  Theta A (M1): {np.rad2deg(solution_rad[0]):.2f} องศา")
    print(f"  Theta B (M2): {np.rad2deg(solution_rad[1]):.2f} องศา")
    
    # ตรวจสอบความแม่นยำ
    verification = calculate_fk(solution_rad)
    error = np.linalg.norm(verification - np.array([target_Px, target_Py]))
    print(f"  ความแม่นยำ: Error = {error:.6f} mm")
    print(f"  Configuration: Down-Down (ข้อเข่าชี้ลงทั้งสองข้าง)")

# --- 6. สร้าง Walking Trajectory ---
print("\n--- 🚶 กำลังสร้างวิถีการเดิน (Elliptical Path)... ---")
walking_path = generate_walking_trajectory(num_steps=60, lift_height=30, step_forward=60)
print(f"สร้างวิถีการเดินเสร็จสิ้น: {len(walking_path)} frames")
print(f"  - รูปแบบ: Elliptical Path (วงรี)")
print(f"  - แกนยาว (X): ±60 mm (แนวนอน)")
print(f"  - แกนสั้น (Y): ±30 mm (แนวตั้ง, จาก home -200mm)")
print(f"  - ช่วงความสูง: -230 mm ถึง -170 mm")
print(f"  - อัตราส่วน: 2:1 (แกนยาว:แกนสั้น)")

# --- 7. สร้างข้อมูลสำหรับพล็อตกราฟ Workspace ---
print("\n--- 📈 กำลังสร้างกราฟ Workspace... ---")
x_range = np.linspace(-250, 250, 50)  # ลดความละเอียดเพื่อความเร็ว
y_range = np.linspace(-300, 50, 50)
reachable_x = []
reachable_y = []

for px in x_range:
    for py in y_range:
        result = calculate_ik_analytical(np.array([px, py]), elbow_C_down=True, elbow_D_down=True)
        if not np.isnan(result).any():
            reachable_x.append(px)
            reachable_y.append(py)

# --- 8. สร้างแอนิเมชันด้วย Matplotlib (⭐ อัปเกรดใหม่ - ใช้ Numerical IK) ---
print("\n--- 🎬 กำลังสร้างแอนิเมชันการเดิน... ---")

fig, ax = plt.subplots(figsize=(12, 12))

# พล็อต Workspace (พื้นหลัง)
ax.scatter(reachable_x, reachable_y, s=2, alpha=0.15, color='lightblue', label='Reachable Workspace')
ax.plot(M1_X, M1_Y, 'ro', markersize=12, label=f'Motor 1 ({M1_X}, {M1_Y})', zorder=5)
ax.plot(M2_X, M2_Y, 'bo', markersize=12, label=f'Motor 2 ({M2_X}, {M2_Y})', zorder=5)

# วาดวิถีการเดินทั้งหมด (เส้นประสีทอง)
path_x = [p[0] for p in walking_path]
path_y = [p[1] for p in walking_path]
ax.plot(path_x, path_y, color='gold', alpha=0.7, linewidth=3, linestyle=':', label='Elliptical Walking Path')

# วาดท่า Home Position
ax.plot(0, -200, 'mo', markersize=10, label='Home (0, -200)', zorder=5)
ax.axhline(-200, color='magenta', linewidth=1, linestyle=':', alpha=0.5, label='Home Level')

# สร้างออบเจ็กต์สำหรับแอนิเมชัน (ใช้สีสันที่แตกต่างชัดเจน)
link1, = ax.plot([], [], color='#FF1744', linewidth=5, label='L₁ (AC) - Left Motor Link', zorder=4)  # แดงสด
link2, = ax.plot([], [], color='#2196F3', linewidth=5, label='L₂ (BD) - Right Motor Link', zorder=4)  # น้ำเงินสด
link3, = ax.plot([], [], color='#FF6F00', linestyle='--', linewidth=4, label='L₃ (CE) - Left Coupler', zorder=3)  # ส้มเข้ม
link4, = ax.plot([], [], color='#00BCD4', linestyle='--', linewidth=4, label='L₄ (DE) - Right Coupler', zorder=3)  # ฟ้าเข้ม
link5, = ax.plot([], [], color='#4CAF50', linestyle='-', linewidth=3.5, label='L₅ (EF) - End-Effector Offset', zorder=4)  # เขียวสด
knee1, = ax.plot([], [], 'o', color='#FF1744', markersize=12, markeredgecolor='black', markeredgewidth=2, label='Joint C (Left Knee)', zorder=5)
knee2, = ax.plot([], [], 'o', color='#2196F3', markersize=12, markeredgecolor='black', markeredgewidth=2, label='Joint D (Right Knee)', zorder=5)
joint_e, = ax.plot([], [], 's', color='#9C27B0', markersize=10, markeredgecolor='black', markeredgewidth=2, label='Joint E (Coupler Point)', zorder=5)  # สี่เหลี่ยมม่วง
foot, = ax.plot([], [], '*', color='#4CAF50', markersize=25, markeredgecolor='black', markeredgewidth=1.5, label='Point F (End-Effector)', zorder=6)

# Text สำหรับแสดงข้อมูล
info_text = ax.text(0.02, 0.98, '', transform=ax.transAxes, 
                    fontsize=10, verticalalignment='top', family='monospace',
                    bbox=dict(boxstyle='round,pad=0.8', facecolor='lightyellow', 
                             edgecolor='black', linewidth=2, alpha=0.9))

# ตั้งค่ากราฟ
ax.set_title('Five-Bar Parallel Linkage Robot - Elliptical Gait Pattern\nAnalytical IK Method (Circle-Circle Intersection)', 
             fontsize=15, weight='bold', pad=15)
ax.set_xlabel('Horizontal Position - X (mm)', fontsize=11, weight='bold')
ax.set_ylabel('Vertical Position - Y (mm)', fontsize=11, weight='bold')
ax.grid(True, linestyle='--', alpha=0.3)
ax.axhline(0, color='black', linewidth=1.0)
ax.axvline(0, color='black', linewidth=1.0)
ax.legend(loc='lower left', fontsize=8, framealpha=0.95, 
         edgecolor='black', fancybox=True, shadow=True, ncol=2)
ax.axis('equal')
ax.set_xlim(-250, 250)
ax.set_ylim(-350, 100)

# ฟังก์ชัน init สำหรับ animation
def init():
    link1.set_data([], [])
    link2.set_data([], [])
    link3.set_data([], [])
    link4.set_data([], [])
    link5.set_data([], [])
    knee1.set_data([], [])
    knee2.set_data([], [])
    joint_e.set_data([], [])
    foot.set_data([], [])
    info_text.set_text('')
    return link1, link2, link3, link4, link5, knee1, knee2, joint_e, foot, info_text

# ตัวแปร global สำหรับเก็บมุมก่อนหน้า (ช่วยตรวจสอบ configuration ที่ถูกต้อง)
prev_solution = None

# ฟังก์ชัน animate สำหรับอัพเดตแต่ละ frame
def animate(frame):
    global prev_solution
    
    # ดึงตำแหน่งเป้าหมายจากวิถีการเดิน
    px, py = walking_path[frame]
    
    # ลองทุก configuration และเลือกที่ใกล้เคียงมุมก่อนหน้าที่สุด
    configs = [
        (True, True),   # Down-Down
        (True, False),  # Down-Up
        (False, True),  # Up-Down
        (False, False)  # Up-Up
    ]
    
    best_solution = None
    best_distance = float('inf')
    
    for elbow_C, elbow_D in configs:
        solution = calculate_ik_analytical(np.array([px, py]), elbow_C_down=elbow_C, elbow_D_down=elbow_D)
        
        if not np.isnan(solution).any():
            # ถ้าไม่มี prev_solution ให้เลือก Down-Down (config แรก)
            if prev_solution is None:
                if elbow_C and elbow_D:  # Down-Down
                    best_solution = solution
                    break
            else:
                # คำนวณระยะห่างของมุม (angular distance)
                angle_diff = np.abs(solution - prev_solution)
                # ปรับให้อยู่ในช่วง [0, π]
                angle_diff = np.minimum(angle_diff, 2*np.pi - angle_diff)
                distance = np.sum(angle_diff)
                
                if distance < best_distance:
                    best_distance = distance
                    best_solution = solution
    
    if best_solution is not None:
        th1, th2 = best_solution
        prev_solution = best_solution  # เก็บมุมปัจจุบันสำหรับ frame ถัดไป
        
        # คำนวณพิกัดข้อเข่า C และ D
        K1_x = P_A[0] + L_AC * np.cos(th1)
        K1_y = P_A[1] + L_AC * np.sin(th1)
        K2_x = P_B[0] + L_BD * np.cos(th2)
        K2_y = P_B[1] + L_BD * np.sin(th2)
        
        # คำนวณจุด E สำหรับการแสดงผล
        P_C = np.array([K1_x, K1_y])
        P_D = np.array([K2_x, K2_y])
        V_CD = P_D - P_C
        d = np.linalg.norm(V_CD)
        
        if d > 0:
            a = (L_CE**2 - L_DE**2 + d**2) / (2 * d)
            h_squared = L_CE**2 - a**2
            if h_squared >= 0:
                h = np.sqrt(h_squared)
                v_d = V_CD / d
                v_perp = np.array([-v_d[1], v_d[0]])
                P_E = P_C + a * v_d - h * v_perp
                
                # อัพเดตลิงก์ทั้งหมด
                link1.set_data([M1_X, K1_x], [M1_Y, K1_y])  # AC
                link2.set_data([M2_X, K2_x], [M2_Y, K2_y])  # BD
                link3.set_data([K1_x, P_E[0]], [K1_y, P_E[1]])  # CE
                link4.set_data([K2_x, P_E[0]], [K2_y, P_E[1]])  # DE
                link5.set_data([P_E[0], px], [P_E[1], py])  # EF (Offset)
                knee1.set_data([K1_x], [K1_y])  # Joint C
                knee2.set_data([K2_x], [K2_y])  # Joint D
                joint_e.set_data([P_E[0]], [P_E[1]])  # Joint E
                foot.set_data([px], [py])  # Foot F
                
                # อัพเดตข้อมูลแสดง
                info_text.set_text(
                    f'═══ Animation Info ═══\n'
                    f'Frame: {frame+1}/{len(walking_path)}\n'
                    f'Progress: {(frame+1)/len(walking_path)*100:.1f}%\n\n'
                    f'═══ Foot Position (F) ═══\n'
                    f'X: {px:+7.1f} mm\n'
                    f'Y: {py:+7.1f} mm\n\n'
                    f'═══ Joint E Position ═══\n'
                    f'X: {P_E[0]:+7.1f} mm\n'
                    f'Y: {P_E[1]:+7.1f} mm\n\n'
                    f'═══ Motor Angles ═══\n'
                    f'θA (Left):  {np.rad2deg(th1):+6.1f}°\n'
                    f'θB (Right): {np.rad2deg(th2):+6.1f}°'
                )
    
    return link1, link2, link3, link4, link5, knee1, knee2, joint_e, foot, info_text

# สร้างแอนิเมชัน
anim = FuncAnimation(fig, animate, init_func=init, 
                     frames=len(walking_path), 
                     interval=50,  # 50ms ต่อ frame (20 FPS)
                     blit=True, 
                     repeat=True)

print("--- ✅ แอนิเมชันพร้อมแล้ว! กำลังแสดงผล... ---")
plt.show()

print("--- สร้างแอนิเมชันเสร็จสิ้น ---")