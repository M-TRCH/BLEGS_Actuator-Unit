import numpy as np

# --- 1. 🤖 กำหนดพารามิเตอร์คงที่ของหุ่นยนต์ (หน่วย mm) ---
# (อ้างอิงจากเอกสาร fk.pdf ที่เราสร้างขึ้น)

# พิกัดมอเตอร์
P_A = np.array([-42.5, 0.0])  # จุด A (มอเตอร์ 1, ซ้าย)
P_B = np.array([42.5, 0.0])   # จุด B (มอเตอร์ 2, ขวา)

# ความยาวลิงก์
L_AC = 105.0  # L1
L_BD = 105.0  # L2
L_CE = 145.0  # L3
L_DE = 145.0  # L4
L_EF = 40.0   # Offset

# อัตราส่วน Offset (จาก PDF)
OFFSET_RATIO_E = 37.0 / 29.0  # (1 + 8/29)
OFFSET_RATIO_D = 8.0 / 29.0


# --- 2. ⚙️ ฟังก์ชัน Forward Kinematics (สำหรับการตรวจสอบ) ---
def calculate_fk(thetas):
    """
    คำนวณ Forward Kinematics (FK)
    Input: thetas = [theta_A, theta_B] (radians)
    Output: P_F = [x_f, y_f] (mm)
    """
    theta_A = thetas[0]
    theta_B = thetas[1]
    
    P_C = P_A + np.array([L_AC * np.cos(theta_A), L_AC * np.sin(theta_A)])
    P_D = P_B + np.array([L_BD * np.cos(theta_B), L_BD * np.sin(theta_B)])

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
    P_E = P_C + a * v_d - h * v_perp

    P_F = (OFFSET_RATIO_E * P_E) - (OFFSET_RATIO_D * P_D)
    
    return P_F


# --- 3. 🎯 ฟังก์ชัน Inverse Kinematics แบบ Analytical ---
def calculate_ik_analytical(P_F_target, elbow_C_down=True, elbow_D_down=True):
    """
    คำนวณ Inverse Kinematics (IK) แบบ Analytical
    
    Input: 
        P_F_target = [x_f, y_f] (mm) - พิกัดเป้าหมายของปลายเท้า
        elbow_C_down = True/False - เลือกท่าข้อเข่า C (ซ้าย) "ลง" หรือ "ขึ้น"
        elbow_D_down = True/False - เลือกท่าข้อเข่า D (ขวา) "ลง" หรือ "ขึ้น"
    
    Output: 
        [theta_A, theta_B] (radians) หรือ [nan, nan] ถ้าไปไม่ถึง
    
    วิธีการ:
    1. จาก P_F = (37/29)*P_E - (8/29)*P_D
       => หา P_E และ P_D ที่เป็นไปได้
    
    2. ใช้ Constraint:
       - |P_E - P_C| = L_CE = 145 mm
       - |P_E - P_D| = L_DE = 145 mm
       - |P_C - P_A| = L_AC = 105 mm
       - |P_D - P_B| = L_BD = 105 mm
    
    3. แก้สมการหา P_D และ P_E
    4. คำนวณมุม theta_A และ theta_B จาก geometry
    
    Configurations:
    - (True, True):   C ลง, D ลง - ท่ามาตรฐาน
    - (True, False):  C ลง, D ขึ้น
    - (False, True):  C ขึ้น, D ลง
    - (False, False): C ขึ้น, D ขึ้น
    """
    
    (x_f, y_f) = P_F_target
    (x_a, y_a) = P_A
    (x_b, y_b) = P_B
    
    # --- ขั้นตอนที่ 1: สร้างสมการจาก P_F = (37/29)*P_E - (8/29)*P_D ---
    # P_F_target = (37/29)*P_E - (8/29)*P_D
    # => 29*P_F_target = 37*P_E - 8*P_D
    # => 37*P_E = 29*P_F_target + 8*P_D
    # => P_E = (29*P_F_target + 8*P_D) / 37
    
    # เราจะหา P_D ก่อน โดยใช้ข้อจำกัด:
    # 1) |P_D - P_B| = L_BD = 105
    # 2) |P_E - P_D| = L_DE = 145
    # 3) P_E = (29*P_F_target + 8*P_D) / 37
    
    # แทนค่า (3) ใน (2):
    # |(29*P_F_target + 8*P_D)/37 - P_D| = 145
    # |(29*P_F_target + 8*P_D - 37*P_D)/37| = 145
    # |(29*P_F_target - 29*P_D)/37| = 145
    # |29*(P_F_target - P_D)|/37 = 145
    # |P_F_target - P_D| = 145 * 37 / 29 = 185.0 mm
    
    R_FD = 145.0 * 37.0 / 29.0  # ระยะจาก P_F ถึง P_D (≈ 185.0 mm)
    R_DB = L_BD  # ระยะจาก P_D ถึง P_B (= 105.0 mm)
    
    # --- ขั้นตอนที่ 2: หา P_D จากการตัดกันของวงกลม 2 วง ---
    # วงกลมที่ 1: ศูนย์กลางที่ P_F_target, รัศมี R_FD
    # วงกลมที่ 2: ศูนย์กลางที่ P_B, รัศมี R_DB
    
    P_D = solve_circle_intersection(P_F_target, R_FD, P_B, R_DB, elbow_D_down)
    
    if np.isnan(P_D).any():
        print("⚠️  เป้าหมายอยู่นอก Workspace (ไปไม่ถึง P_D)")
        return np.array([np.nan, np.nan])
    
    # --- ขั้นตอนที่ 3: หา P_E จากสมการ ---
    P_E = (29.0 * P_F_target + 8.0 * P_D) / 37.0
    
    # --- ขั้นตอนที่ 4: หา P_C จากการตัดกันของวงกลม 2 วง ---
    # วงกลมที่ 1: ศูนย์กลางที่ P_A, รัศมี L_AC
    # วงกลมที่ 2: ศูนย์กลางที่ P_E, รัศมี L_CE
    
    P_C = solve_circle_intersection(P_A, L_AC, P_E, L_CE, elbow_C_down)
    
    if np.isnan(P_C).any():
        print("⚠️  เป้าหมายอยู่นอก Workspace (ไปไม่ถึง P_C)")
        return np.array([np.nan, np.nan])
    
    # --- ขั้นตอนที่ 5: คำนวณมุม theta_A และ theta_B ---
    # theta_A = atan2(P_C - P_A)
    # theta_B = atan2(P_D - P_B)
    
    V_AC = P_C - P_A
    V_BD = P_D - P_B
    
    theta_A = np.arctan2(V_AC[1], V_AC[0])
    theta_B = np.arctan2(V_BD[1], V_BD[0])
    
    return np.array([theta_A, theta_B])


# --- 4. 🔧 ฟังก์ชันช่วย: หาจุดตัดของวงกลม 2 วง ---
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
        [x, y] - จุดตัด หรือ [nan, nan] ถ้าไม่มีจุดตัด
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
    
    # จุดตัด 2 จุด:
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


# --- 5. 🚀 ทดสอบ IK Analytical ทั้ง 4 Configurations ---
if __name__ == "__main__":
    print("=" * 70)
    print("🤖 Inverse Kinematics (Analytical Method) - All 4 Configurations")
    print("=" * 70)
    
    # กำหนดเป้าหมาย
    P_F_TARGET = np.array([0.0, -200.0])
    
    print(f"\n🎯 เป้าหมาย (Target Pose): {P_F_TARGET} mm")
    print("\n" + "=" * 70)
    
    # กำหนด 4 Configurations ที่เป็นไปได้
    configurations = [
        (True, True,   "Config 1: Elbow C ลง, D ลง (Down-Down) 🦵"),
        (True, False,  "Config 2: Elbow C ลง, D ขึ้น (Down-Up)   🦵"),
        (False, True,  "Config 3: Elbow C ขึ้น, D ลง (Up-Down)   🦵"),
        (False, False, "Config 4: Elbow C ขึ้น, D ขึ้น (Up-Up)   🦵")
    ]
    
    valid_solutions = []
    
    for i, (elbow_C, elbow_D, description) in enumerate(configurations, 1):
        print(f"\n{description}")
        print("-" * 70)
        
        # คำนวณ IK
        solution_rad = calculate_ik_analytical(P_F_TARGET, elbow_C_down=elbow_C, elbow_D_down=elbow_D)
        
        if np.isnan(solution_rad).any():
            print("❌ ไม่สามารถหาคำตอบได้ (เป้าหมายอยู่นอก Workspace)")
            continue
        
        solution_deg = np.rad2deg(solution_rad)
        solution_deg_normalized = ((solution_deg + 180) % 360) - 180
        
        # ตรวจสอบด้วย FK
        verification_pose = calculate_fk(solution_rad)
        error = verification_pose - P_F_TARGET
        error_norm = np.linalg.norm(error)
        
        # แสดงผล
        print(f"✅ พบคำตอบ!")
        print(f"   Theta A (M1): {solution_deg[0]:8.4f}° → {solution_deg_normalized[0]:8.4f}°")
        print(f"   Theta B (M2): {solution_deg[1]:8.4f}° → {solution_deg_normalized[1]:8.4f}°")
        print(f"\n   📍 ตรวจสอบด้วย FK:")
        print(f"      พิกัดที่ได้: [{verification_pose[0]:10.6f}, {verification_pose[1]:10.6f}] mm")
        print(f"      เป้าหมาย:    [{P_F_TARGET[0]:10.1f}, {P_F_TARGET[1]:10.1f}] mm")
        print(f"      |Error|:      {error_norm:.6e} mm", end="")
        
        # ตรวจสอบว่าเป็น Valid Solution หรือไม่ (error < 0.01 mm)
        if error_norm < 0.01:
            print(" ✅ (Valid)")
            valid_solutions.append((i, description, solution_deg_normalized, error_norm))
        else:
            print(" ⚠️  (Invalid - error สูง)")
    
    # สรุปผล
    print("\n" + "=" * 70)
    print("📊 สรุปผลลัพธ์")
    print("=" * 70)
    print(f"\n✅ พบคำตอบที่ถูกต้อง (Valid): {len(valid_solutions)} จาก 4 Configurations")
    
    if valid_solutions:
        print("\n🎯 คำตอบที่ถูกต้องทั้งหมด:")
        for config_num, desc, angles, err in valid_solutions:
            print(f"\n   {desc}")
            print(f"      θA = {angles[0]:8.4f}°, θB = {angles[1]:8.4f}° (Error: {err:.2e} mm)")
    
    print("\n" + "=" * 70)
    print("✨ เสร็จสิ้น!")
    print("=" * 70)
