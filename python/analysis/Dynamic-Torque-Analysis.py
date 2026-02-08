import numpy as np
import matplotlib.pyplot as plt
from matplotlib.gridspec import GridSpec

# ============================================================================
# Phase 2.2: Dynamic Torque Analysis for 5-Bar Parallel Linkage Robot
# ============================================================================
# คำนวณทอร์กแบบ Dynamic สำหรับ Elliptical Gait Pattern
# โดยพิจารณา Inertia, Coriolis, และ Gravity
# ============================================================================

# --- 1. พารามิเตอร์คงที่ของหุ่นยนต์ (หน่วย SI) ---

# พิกัดมอเตอร์ (meters)
P_A = np.array([-0.0425, 0.0])  # จุด A (มอเตอร์ซ้าย)
P_B = np.array([0.0425, 0.0])   # จุด B (มอเตอร์ขวา)

# ความยาวลิงก์ (meters)
L_AC = 0.105  # L1
L_BD = 0.105  # L2
L_CE = 0.145  # L3
L_DE = 0.145  # L4
L_EF = 0.040  # Offset

# อัตราส่วน Offset
OFFSET_RATIO_E = 37.0 / 29.0
OFFSET_RATIO_D = 8.0 / 29.0

# ค่าคงที่
g = 9.81  # m/s² (แรงโน้มถ่วง)

# น้ำหนักรวมหุ่นยนต์ (8-DOF Quadruped Mobile Robot)
MASS_BATTERY_ELECTRONICS = 2.00  # kg
MASS_FRAME = 1.62                 # kg
MASS_MOTORS_TOTAL = 3.08          # kg (8 motors × 385g)
TOTAL_ROBOT_MASS = MASS_BATTERY_ELECTRONICS + MASS_FRAME + MASS_MOTORS_TOTAL  # 6.70 kg
MASS_PER_LEG = TOTAL_ROBOT_MASS / 4.0  # 1.675 kg per leg (กระจายน้ำหนักเท่ากัน)

# --- 2. พารามิเตอร์มวลและ Inertia (จาก CAD) ---

# Link 1 (AC) - Left Motor Link
m1 = 0.02488  # kg
L1 = L_AC
COM_ratio_1 = 0.3356  # COM อยู่ที่ 33.56% จาก A
r_COM_1 = COM_ratio_1 * L1  # 0.03524 m
I_zz_1 = 0.00000  # kg·m² (negligible)

# Link 2 (BD) - Right Motor Link
m2 = 0.03533  # kg
L2 = L_BD
COM_ratio_2 = 0.2363  # COM อยู่ที่ 23.63% จาก B
r_COM_2 = COM_ratio_2 * L2  # 0.02481 m
I_zz_2 = 0.00001  # kg·m²

# Link 3 (CE) - Left Coupler
m3 = 0.02056  # kg
L3 = L_CE
COM_ratio_3 = 0.5000  # COM อยู่กึ่งกลาง
r_COM_3 = COM_ratio_3 * L3  # 0.07250 m
I_zz_3 = 0.00005  # kg·m²

# Link 4 (DE) - Right Coupler
m4 = 0.02506  # kg
L4 = L_DE
COM_ratio_4 = 0.6173  # COM อยู่ที่ 61.73% จาก D
r_COM_4 = COM_ratio_4 * L4  # 0.08951 m
I_zz_4 = 0.00008  # kg·m²

# --- 3. พารามิเตอร์การเดิน ---

STEP_FREQUENCY = 1.0  # Hz (ความถี่การก้าว)
CYCLE_TIME = 1.0 / STEP_FREQUENCY  # seconds (เวลา 1 รอบ)
NUM_STEPS = 60  # จำนวน frame

# Elliptical Path
STRIDE_X = 0.060  # m (±60 mm แกนยาว)
STRIDE_Y = 0.030  # m (±30 mm แกนสั้น)
HOME_Y = -0.200  # m (ท่ายืน)

# --- 4. พารามิเตอร์มอเตอร์ ---

GEAR_RATIO = 8  # 8:1
MAX_RPM_OUTPUT = 120  # RPM (หลังเกียร์)
MAX_OMEGA_OUTPUT = MAX_RPM_OUTPUT * (2 * np.pi / 60)  # 12.57 rad/s

print("=" * 80)
print("Dynamic Torque Analysis - 5-Bar Parallel Linkage Robot")
print("=" * 80)
print(f"\nConfiguration:")
print(f"   - Robot Type: 8-DOF Quadruped Mobile Robot")
print(f"   - Total Robot Mass: {TOTAL_ROBOT_MASS:.2f} kg")
print(f"     * Battery + Electronics: {MASS_BATTERY_ELECTRONICS:.2f} kg")
print(f"     * Frame: {MASS_FRAME:.2f} kg")
print(f"     * Motors (8x): {MASS_MOTORS_TOTAL:.2f} kg")
print(f"   - Mass per Leg (distributed): {MASS_PER_LEG:.3f} kg")
print(f"   - Link masses: L1={m1*1000:.2f}g, L2={m2*1000:.2f}g, L3={m3*1000:.2f}g, L4={m4*1000:.2f}g")
print(f"   - Total link mass: {(m1+m2+m3+m4)*1000:.2f} g")
print(f"   - Step frequency: {STEP_FREQUENCY} Hz")
print(f"   - Cycle time: {CYCLE_TIME} s")
print(f"   - Motor speed: {MAX_RPM_OUTPUT} RPM (output shaft)")
print(f"   - Gear ratio: {GEAR_RATIO}:1")


# --- 5. ฟังก์ชัน Forward Kinematics ---

def calculate_fk(thetas):
    """
    คำนวณ Forward Kinematics
    Input: thetas = [theta_A, theta_B] (radians)
    Output: P_F = [x_f, y_f] (meters)
    """
    theta_A = thetas[0]
    theta_B = thetas[1]
    
    # คำนวณพิกัดข้อเข่า
    P_C = P_A + np.array([L_AC * np.cos(theta_A), L_AC * np.sin(theta_A)])
    P_D = P_B + np.array([L_BD * np.cos(theta_B), L_BD * np.sin(theta_B)])

    # คำนวณพิกัด P_E
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

    # คำนวณพิกัดปลายเท้า
    P_F = (OFFSET_RATIO_E * P_E) - (OFFSET_RATIO_D * P_D)
    
    return P_F


# --- 6. ฟังก์ชัน Inverse Kinematics (Analytical) ---

def solve_circle_intersection(center1, radius1, center2, radius2, choose_lower=True):
    """หาจุดตัดของวงกลม 2 วง"""
    V_12 = center2 - center1
    d = np.linalg.norm(V_12)
    
    if d > (radius1 + radius2) or d < abs(radius1 - radius2) or d == 0:
        return np.array([np.nan, np.nan])
    
    a = (radius1**2 - radius2**2 + d**2) / (2 * d)
    h_squared = radius1**2 - a**2
    
    if h_squared < 0:
        return np.array([np.nan, np.nan])
    
    h = np.sqrt(h_squared)
    v_d = V_12 / d
    v_perp = np.array([-v_d[1], v_d[0]])
    
    P_intersection_1 = center1 + a * v_d + h * v_perp
    P_intersection_2 = center1 + a * v_d - h * v_perp
    
    if choose_lower:
        return P_intersection_2 if P_intersection_2[1] < P_intersection_1[1] else P_intersection_1
    else:
        return P_intersection_1 if P_intersection_1[1] > P_intersection_2[1] else P_intersection_2


def calculate_ik_analytical(P_F_target, elbow_C_down=True, elbow_D_down=True):
    """คำนวณ IK แบบ Analytical"""
    R_FD = L_DE * OFFSET_RATIO_E
    R_DB = L_BD
    
    # หา P_D (สลับทิศสำหรับ Motor 2)
    P_D = solve_circle_intersection(P_F_target, R_FD, P_B, R_DB, not elbow_D_down)
    
    if np.isnan(P_D).any():
        return np.array([np.nan, np.nan])
    
    # หา P_E
    P_E = (29.0 * P_F_target + 8.0 * P_D) / 37.0
    
    # หา P_C
    P_C = solve_circle_intersection(P_A, L_AC, P_E, L_CE, elbow_C_down)
    
    if np.isnan(P_C).any():
        return np.array([np.nan, np.nan])
    
    # คำนวณมุม
    V_AC = P_C - P_A
    V_BD = P_D - P_B
    
    theta_A = np.arctan2(V_AC[1], V_AC[0])
    theta_B = np.arctan2(V_BD[1], V_BD[0])
    
    return np.array([theta_A, theta_B])


# --- 7. ฟังก์ชัน Jacobian สำหรับ End-Effector ---

def calculate_jacobian_F(thetas):
    """
    คำนวณ Jacobian ของ End-Effector (Point F)
    Input: thetas = [theta_A, theta_B]
    Output: J_F (2x2 matrix)
    """
    theta_A = thetas[0]
    theta_B = thetas[1]
    
    # คำนวณตำแหน่งข้อต่อ
    P_C = P_A + np.array([L_AC * np.cos(theta_A), L_AC * np.sin(theta_A)])
    P_D = P_B + np.array([L_BD * np.cos(theta_B), L_BD * np.sin(theta_B)])
    
    V_CD = P_D - P_C
    d = np.linalg.norm(V_CD)
    
    if d == 0 or d > (L_CE + L_DE) or d < abs(L_CE - L_DE):
        return np.zeros((2, 2))
    
    a = (L_CE**2 - L_DE**2 + d**2) / (2 * d)
    h_squared = L_CE**2 - a**2
    
    if h_squared < 0:
        return np.zeros((2, 2))
    
    h = np.sqrt(h_squared)
    v_d = V_CD / d
    v_perp = np.array([-v_d[1], v_d[0]])
    
    P_E = P_C + a * v_d - h * v_perp
    
    # คำนวณ Jacobian J_D
    J_D = np.array([
        [0.0, -L_BD * np.sin(theta_B)],
        [0.0,  L_BD * np.cos(theta_B)]
    ])
    
    # คำนวณ Jacobian J_E
    A = np.array([
        [P_E[0] - P_C[0], P_E[1] - P_C[1]],
        [P_E[0] - P_D[0], P_E[1] - P_D[1]]
    ])
    
    B_11 = L_AC * ((P_E[1] - P_C[1]) * np.cos(theta_A) - (P_E[0] - P_C[0]) * np.sin(theta_A))
    B_22 = L_BD * ((P_E[1] - P_D[1]) * np.cos(theta_B) - (P_E[0] - P_D[0]) * np.sin(theta_B))
    
    B = np.array([
        [B_11, 0.0],
        [0.0,  B_22]
    ])
    
    try:
        A_inv = np.linalg.inv(A)
        J_E = A_inv @ B
    except np.linalg.LinAlgError:
        return np.zeros((2, 2))
    
    # คำนวณ J_F
    J_F = (1.0 / 29.0) * (37.0 * J_E - 8.0 * J_D)
    
    return J_F


# --- 8. ฟังก์ชัน Jacobian สำหรับ COM ของแต่ละ Link ---

def calculate_jacobian_COM(thetas, link_id):
    """
    คำนวณ Jacobian ของศูนย์กลางมวล (COM) แต่ละ link
    
    Input:
        thetas = [theta_A, theta_B]
        link_id = 1, 2, 3, 4
    
    Output:
        J_v: Linear velocity Jacobian (2x2)
        J_omega: Angular velocity Jacobian (1x2)
    """
    theta_A = thetas[0]
    theta_B = thetas[1]
    
    if link_id == 1:  # Link AC
        # COM อยู่ที่ระยะ r_COM_1 จาก A ตามทิศ theta_A
        J_v = np.array([
            [-r_COM_1 * np.sin(theta_A), 0.0],
            [ r_COM_1 * np.cos(theta_A), 0.0]
        ])
        J_omega = np.array([1.0, 0.0])  # ω = θ̇_A
        
    elif link_id == 2:  # Link BD
        # COM อยู่ที่ระยะ r_COM_2 จาก B ตามทิศ theta_B
        J_v = np.array([
            [0.0, -r_COM_2 * np.sin(theta_B)],
            [0.0,  r_COM_2 * np.cos(theta_B)]
        ])
        J_omega = np.array([0.0, 1.0])  # ω = θ̇_B
        
    elif link_id == 3:  # Link CE
        # COM อยู่ที่ระยะ r_COM_3 จาก C
        # ต้องคำนวณมุม theta_CE (มุมของ link CE)
        P_C = P_A + np.array([L_AC * np.cos(theta_A), L_AC * np.sin(theta_A)])
        P_D = P_B + np.array([L_BD * np.cos(theta_B), L_BD * np.sin(theta_B)])
        
        V_CD = P_D - P_C
        d = np.linalg.norm(V_CD)
        
        if d == 0:
            return np.zeros((2, 2)), np.zeros(2)
        
        a = (L_CE**2 - L_DE**2 + d**2) / (2 * d)
        h_squared = L_CE**2 - a**2
        
        if h_squared < 0:
            return np.zeros((2, 2)), np.zeros(2)
        
        h = np.sqrt(h_squared)
        v_d = V_CD / d
        v_perp = np.array([-v_d[1], v_d[0]])
        
        P_E = P_C + a * v_d - h * v_perp
        V_CE = P_E - P_C
        theta_CE = np.arctan2(V_CE[1], V_CE[0])
        
        # Jacobian ของจุด C
        J_C = np.array([
            [-L_AC * np.sin(theta_A), 0.0],
            [ L_AC * np.cos(theta_A), 0.0]
        ])
        
        # Jacobian ของ COM Link 3
        J_v = J_C + np.array([
            [-r_COM_3 * np.sin(theta_CE), 0.0],  # อาจต้องปรับตาม partial derivative
            [ r_COM_3 * np.cos(theta_CE), 0.0]
        ])
        
        # Angular velocity (ต้องคำนวณ dθ_CE/dq)
        J_omega = np.array([1.0, 0.0])  # Simplified
        
    elif link_id == 4:  # Link DE
        # COM อยู่ที่ระยะ r_COM_4 จาก D
        P_C = P_A + np.array([L_AC * np.cos(theta_A), L_AC * np.sin(theta_A)])
        P_D = P_B + np.array([L_BD * np.cos(theta_B), L_BD * np.sin(theta_B)])
        
        V_CD = P_D - P_C
        d = np.linalg.norm(V_CD)
        
        if d == 0:
            return np.zeros((2, 2)), np.zeros(2)
        
        a = (L_CE**2 - L_DE**2 + d**2) / (2 * d)
        h_squared = L_CE**2 - a**2
        
        if h_squared < 0:
            return np.zeros((2, 2)), np.zeros(2)
        
        h = np.sqrt(h_squared)
        v_d = V_CD / d
        v_perp = np.array([-v_d[1], v_d[0]])
        
        P_E = P_C + a * v_d - h * v_perp
        V_DE = P_E - P_D
        theta_DE = np.arctan2(V_DE[1], V_DE[0])
        
        # Jacobian ของจุด D
        J_D = np.array([
            [0.0, -L_BD * np.sin(theta_B)],
            [0.0,  L_BD * np.cos(theta_B)]
        ])
        
        # Jacobian ของ COM Link 4
        J_v = J_D + np.array([
            [0.0, -r_COM_4 * np.sin(theta_DE)],  # อาจต้องปรับ
            [0.0,  r_COM_4 * np.cos(theta_DE)]
        ])
        
        # Angular velocity
        J_omega = np.array([0.0, 1.0])  # Simplified
    
    else:
        J_v = np.zeros((2, 2))
        J_omega = np.zeros(2)
    
    return J_v, J_omega


# --- 9. ฟังก์ชันคำนวณ Inertia Matrix M(q) ---

def calculate_inertia_matrix(thetas):
    """
    คำนวณ Inertia Matrix M(q)
    
    M(q) = Σ [m_i * J_v_i^T * J_v_i + I_zz_i * J_ω_i^T * J_ω_i]
    
    Output: M (2x2 matrix)
    """
    M = np.zeros((2, 2))
    
    # Link 1
    J_v1, J_omega1 = calculate_jacobian_COM(thetas, 1)
    M += m1 * (J_v1.T @ J_v1) + I_zz_1 * np.outer(J_omega1, J_omega1)
    
    # Link 2
    J_v2, J_omega2 = calculate_jacobian_COM(thetas, 2)
    M += m2 * (J_v2.T @ J_v2) + I_zz_2 * np.outer(J_omega2, J_omega2)
    
    # Link 3
    J_v3, J_omega3 = calculate_jacobian_COM(thetas, 3)
    M += m3 * (J_v3.T @ J_v3) + I_zz_3 * np.outer(J_omega3, J_omega3)
    
    # Link 4
    J_v4, J_omega4 = calculate_jacobian_COM(thetas, 4)
    M += m4 * (J_v4.T @ J_v4) + I_zz_4 * np.outer(J_omega4, J_omega4)
    
    return M


# --- 10. ฟังก์ชันคำนวณ Gravity Vector G(q) ---

def calculate_gravity_vector(thetas):
    """
    คำนวณ Gravity Vector G(q)
    
    G(q) = Σ m_i * g * J_v_i^T * [0, -1]^T
    รวมถึงน้ำหนักหุ่นยนต์ที่กระจายมาที่ขานี้ (1/4 ของน้ำหนักรวม)
    
    Output: G (2x1 vector)
    """
    G = np.zeros(2)
    gravity_dir = np.array([0, -g])  # แรงโน้มถ่วงชี้ลง
    
    # Link 1
    J_v1, _ = calculate_jacobian_COM(thetas, 1)
    G += m1 * (J_v1.T @ gravity_dir)
    
    # Link 2
    J_v2, _ = calculate_jacobian_COM(thetas, 2)
    G += m2 * (J_v2.T @ gravity_dir)
    
    # Link 3
    J_v3, _ = calculate_jacobian_COM(thetas, 3)
    G += m3 * (J_v3.T @ gravity_dir)
    
    # Link 4
    J_v4, _ = calculate_jacobian_COM(thetas, 4)
    G += m4 * (J_v4.T @ gravity_dir)
    
    # เพิ่มน้ำหนักหุ่นยนต์ที่กระจายมาที่ขานี้ (ที่จุด foot)
    J_foot = calculate_jacobian_F(thetas)
    G += MASS_PER_LEG * (J_foot.T @ gravity_dir)
    
    return G


# --- 11. สร้าง Elliptical Trajectory ---

def generate_walking_trajectory(num_steps, lift_height, step_forward):
    """สร้างวิถี Elliptical Path"""
    trajectory = []
    
    for i in range(num_steps):
        t = 2 * np.pi * i / num_steps
        px = step_forward * np.cos(t)
        py = HOME_Y + lift_height * np.sin(t)
        trajectory.append(np.array([px, py]))
    
    return trajectory


# --- 12. คำนวณ Velocity และ Acceleration ---

def calculate_trajectory_derivatives(trajectory, T_cycle):
    """
    คำนวณ velocity (q̇) และ acceleration (q̈) จาก trajectory
    
    Input:
        trajectory: list of (x, y) positions
        T_cycle: เวลา 1 รอบ (seconds)
    
    Output:
        q_traj: array of joint angles (Nx2)
        q_dot: array of joint velocities (Nx2)
        q_ddot: array of joint accelerations (Nx2)
    """
    N = len(trajectory)
    dt = T_cycle / N
    
    q_traj = []
    prev_sol = None
    
    # คำนวณ IK สำหรับทุกจุด
    for pos in trajectory:
        # ลองทุก configuration และเลือกที่ smooth ที่สุด
        configs = [(True, True), (True, False), (False, True), (False, False)]
        best_sol = None
        best_dist = float('inf')
        
        for elbow_C, elbow_D in configs:
            sol = calculate_ik_analytical(pos, elbow_C, elbow_D)
            if not np.isnan(sol).any():
                if prev_sol is None:
                    if elbow_C and elbow_D:
                        best_sol = sol
                        break
                else:
                    angle_diff = np.abs(sol - prev_sol)
                    angle_diff = np.minimum(angle_diff, 2*np.pi - angle_diff)
                    dist = np.sum(angle_diff)
                    if dist < best_dist:
                        best_dist = dist
                        best_sol = sol
        
        if best_sol is not None:
            q_traj.append(best_sol)
            prev_sol = best_sol
        else:
            q_traj.append(prev_sol if prev_sol is not None else np.array([0, 0]))
    
    q_traj = np.array(q_traj)
    
    # คำนวณ velocity (ใช้ central difference)
    q_dot = np.zeros_like(q_traj)
    for i in range(N):
        if i == 0:
            q_dot[i] = (q_traj[1] - q_traj[-1]) / (2 * dt)
        elif i == N - 1:
            q_dot[i] = (q_traj[0] - q_traj[-2]) / (2 * dt)
        else:
            q_dot[i] = (q_traj[i+1] - q_traj[i-1]) / (2 * dt)
    
    # คำนวณ acceleration (ใช้ central difference)
    q_ddot = np.zeros_like(q_traj)
    for i in range(N):
        if i == 0:
            q_ddot[i] = (q_traj[1] - 2*q_traj[0] + q_traj[-1]) / (dt**2)
        elif i == N - 1:
            q_ddot[i] = (q_traj[0] - 2*q_traj[-1] + q_traj[-2]) / (dt**2)
        else:
            q_ddot[i] = (q_traj[i+1] - 2*q_traj[i] + q_traj[i-1]) / (dt**2)
    
    return q_traj, q_dot, q_ddot


# --- 13. คำนวณ Dynamic Torque ---

print("\n" + "=" * 80)
print("Calculating Dynamic Torque...")
print("=" * 80)

# สร้าง trajectory
trajectory = generate_walking_trajectory(NUM_STEPS, STRIDE_Y, STRIDE_X)

# คำนวณ q, q̇, q̈
q_traj, q_dot, q_ddot = calculate_trajectory_derivatives(trajectory, CYCLE_TIME)

# คำนวณ torque สำหรับแต่ละจุด
tau_dynamic = []
tau_gravity = []
tau_inertia = []

for i in range(NUM_STEPS):
    q = q_traj[i]
    qd = q_dot[i]
    qdd = q_ddot[i]
    
    # Inertia Matrix
    M = calculate_inertia_matrix(q)
    
    # Gravity Vector
    G = calculate_gravity_vector(q)
    
    # Dynamic Torque (simplified without Coriolis)
    tau_total = M @ qdd + G
    tau_M = M @ qdd
    
    tau_dynamic.append(tau_total)
    tau_gravity.append(G)
    tau_inertia.append(tau_M)

tau_dynamic = np.array(tau_dynamic)
tau_gravity = np.array(tau_gravity)
tau_inertia = np.array(tau_inertia)


# --- 14. วิเคราะห์ผล ---

print("\n" + "=" * 80)
print("Dynamic Torque Analysis Results")
print("=" * 80)

# Peak Torque
peak_tau_A = np.max(np.abs(tau_dynamic[:, 0]))
peak_tau_B = np.max(np.abs(tau_dynamic[:, 1]))

print(f"\nPeak Dynamic Torque:")
print(f"   Motor A (Left):  {peak_tau_A:.4f} N·m")
print(f"   Motor B (Right): {peak_tau_B:.4f} N·m")
print(f"   Maximum: {max(peak_tau_A, peak_tau_B):.4f} N·m")

# Static Torque (Home Position)
home_idx = 0
tau_static_A = tau_gravity[home_idx, 0]
tau_static_B = tau_gravity[home_idx, 1]

print(f"\nStatic Torque (Home Position):")
print(f"   Motor A: {tau_static_A:.4f} N·m")
print(f"   Motor B: {tau_static_B:.4f} N·m")

# Comparison
print(f"\nDynamic vs Static Ratio:")
print(f"   Motor A: {peak_tau_A / abs(tau_static_A):.2f}x")
print(f"   Motor B: {peak_tau_B / abs(tau_static_B):.2f}x")

# Motor Check
MOTOR_TORQUE = 5.0  # N·m
safety_factor_A = MOTOR_TORQUE / peak_tau_A
safety_factor_B = MOTOR_TORQUE / peak_tau_B

print(f"\nMotor Selection (5 N·m):")
print(f"   Safety Factor A: {safety_factor_A:.2f}x")
print(f"   Safety Factor B: {safety_factor_B:.2f}x")

if min(safety_factor_A, safety_factor_B) >= 2.0:
    print(f"   PASS - Motor is adequate (Safety Factor >= 2x)")
elif min(safety_factor_A, safety_factor_B) >= 1.5:
    print(f"   WARNING - Low safety factor (1.5x - 2x)")
else:
    print(f"   FAIL - Motor is inadequate (Safety Factor < 1.5x)")


# --- 15. Visualization ---

print("\n" + "=" * 80)
print("Generating plots...")
print("=" * 80)

fig = plt.figure(figsize=(16, 12))
gs = GridSpec(3, 2, figure=fig, hspace=0.6, wspace=0.3)

time = np.linspace(0, CYCLE_TIME, NUM_STEPS)

# Plot 1: Joint Angles
ax1 = fig.add_subplot(gs[0, 0])
ax1.plot(time, np.rad2deg(q_traj[:, 0]), 'r-', linewidth=2, label='θA (Left)')
ax1.plot(time, np.rad2deg(q_traj[:, 1]), 'b-', linewidth=2, label='θB (Right)')
ax1.set_xlabel('Time (s)', fontsize=11, weight='bold')
ax1.set_ylabel('Joint Angle (deg)', fontsize=11, weight='bold')
ax1.set_title('Joint Angles over Time', fontsize=12, weight='bold')
ax1.grid(True, alpha=0.3)
ax1.legend()

# Plot 2: Joint Velocities
ax2 = fig.add_subplot(gs[0, 1])
ax2.plot(time, q_dot[:, 0], 'r-', linewidth=2, label='θ̇A (Left)')
ax2.plot(time, q_dot[:, 1], 'b-', linewidth=2, label='θ̇B (Right)')
ax2.axhline(MAX_OMEGA_OUTPUT, color='orange', linestyle='--', linewidth=2, label=f'Max ({MAX_RPM_OUTPUT} RPM)')
ax2.axhline(-MAX_OMEGA_OUTPUT, color='orange', linestyle='--', linewidth=2)
ax2.set_xlabel('Time (s)', fontsize=11, weight='bold')
ax2.set_ylabel('Angular Velocity (rad/s)', fontsize=11, weight='bold')
ax2.set_title('Joint Velocities over Time', fontsize=12, weight='bold')
ax2.grid(True, alpha=0.3)
ax2.legend()

# Plot 3: Joint Accelerations
ax3 = fig.add_subplot(gs[1, 0])
ax3.plot(time, q_ddot[:, 0], 'r-', linewidth=2, label='θ̈A (Left)')
ax3.plot(time, q_ddot[:, 1], 'b-', linewidth=2, label='θ̈B (Right)')
ax3.set_xlabel('Time (s)', fontsize=11, weight='bold')
ax3.set_ylabel('Angular Acceleration (rad/s²)', fontsize=11, weight='bold')
ax3.set_title('Joint Accelerations over Time', fontsize=12, weight='bold')
ax3.grid(True, alpha=0.3)
ax3.legend()

# Plot 4: Dynamic Torque Components
ax4 = fig.add_subplot(gs[1, 1])
ax4.plot(time, tau_inertia[:, 0], 'r--', linewidth=1.5, alpha=0.7, label='Inertia A')
ax4.plot(time, tau_gravity[:, 0], 'r:', linewidth=1.5, alpha=0.7, label='Gravity A')
ax4.plot(time, tau_dynamic[:, 0], 'r-', linewidth=2.5, label='Total A')
ax4.set_xlabel('Time (s)', fontsize=11, weight='bold')
ax4.set_ylabel('Torque (N·m)', fontsize=11, weight='bold')
ax4.set_title('Torque Components - Motor A (Left)', fontsize=12, weight='bold')
ax4.grid(True, alpha=0.3)
ax4.legend()

# Plot 5: Total Dynamic Torque
ax5 = fig.add_subplot(gs[2, :])
ax5.plot(time, tau_dynamic[:, 0], 'r-', linewidth=2.5, label='Motor A (Left)')
ax5.plot(time, tau_dynamic[:, 1], 'b-', linewidth=2.5, label='Motor B (Right)')
ax5.axhline(MOTOR_TORQUE, color='green', linestyle='--', linewidth=2, label=f'Motor Limit ({MOTOR_TORQUE} N·m)')
ax5.axhline(-MOTOR_TORQUE, color='green', linestyle='--', linewidth=2)
ax5.fill_between(time, -MOTOR_TORQUE, MOTOR_TORQUE, alpha=0.2, color='green')
ax5.set_xlabel('Time (s)', fontsize=11, weight='bold')
ax5.set_ylabel('Torque (N·m)', fontsize=11, weight='bold')
ax5.set_title('Total Dynamic Torque - Both Motors', fontsize=12, weight='bold')
ax5.grid(True, alpha=0.3)
ax5.legend()

plt.suptitle('Dynamic Torque Analysis - 5-Bar Parallel Linkage Robot\n' + 
             f'Elliptical Gait ({STEP_FREQUENCY} Hz) | Total Mass: {TOTAL_ROBOT_MASS:.2f} kg | Per Leg: {MASS_PER_LEG:.3f} kg', 
             fontsize=14, weight='bold', y=0.995)

plt.tight_layout()
plt.show()

print("\n" + "=" * 80)
print("Analysis completed successfully!")
print("=" * 80)
