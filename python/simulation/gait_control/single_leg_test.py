import pybullet as p
import pybullet_data
import time
import math
import os

# 1. เชื่อมต่อกับ Physics Server
physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.81)
planeId = p.loadURDF("plane.urdf")

# 2. โหลด URDF quadruped (แต่จะใช้เฉพาะขา FR)
script_dir = os.path.dirname(os.path.abspath(__file__))
project_root = os.path.abspath(os.path.join(script_dir, "..", "..", ".."))
urdf_path = os.path.join(project_root, "python", "models", "urdf", "quadruped", "my_robot.urdf")

# แขวนทั้งตัวไว้กับที่ (จะควบคุมเฉพาะขา FR)
startPos = [0, 0, 0.5]
startOrientation = p.getQuaternionFromEuler([0, 0, 0])
robotId = p.loadURDF(urdf_path, startPos, startOrientation, useFixedBase=True)

# ========================================
# เปิดใช้งานการควบคุมกล้องด้วยเมาส์
# ========================================
# ปิด GUI sidebar ก่อน (อาจบล็อคการควบคุมเมาส์)
p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
# เปิดการควบคุมด้วยเมาส์และคีย์บอร์ด
p.configureDebugVisualizer(p.COV_ENABLE_MOUSE_PICKING, 0)  # ปิด mouse picking (ให้ใช้เมาส์กับกล้อง)
p.configureDebugVisualizer(p.COV_ENABLE_KEYBOARD_SHORTCUTS, 1)
p.configureDebugVisualizer(p.COV_ENABLE_SHADOWS, 1)

# ตั้งมุมมองเริ่มต้น (ด้านข้าง)
p.resetDebugVisualizerCamera(
    cameraDistance=0.8,
    cameraYaw=90,              # มองจากด้านข้าง
    cameraPitch=-20,
    cameraTargetPosition=[0.2, -0.15, 0.3]
)

print("\n🖱️ การควบคุมกล้อง:")
print("  - ซูม: Scroll Wheel")
print("  - หมุน: คีย์ 1-4 (มุมมองต่างๆ)")
print("    [1] Front View  [2] Side View  [3] Top View  [4] Isometric")
print("  - ปรับแต่ง: คีย์ U/I/J/K/O/L (Yaw/Pitch/Distance)")
print("  - เลื่อนซ้าย-ขวา: คีย์ A/D")
print("  - เลื่อนหน้า-หลัง: คีย์ W/S")
print("  - เลื่อนขึ้น-ลง: คีย์ Q/E")
print("  - รีเซ็ต: คีย์ R\n")

# ตัวแปรสำหรับควบคุมกล้อง
camera_distance = 0.8
camera_yaw = 90
camera_pitch = -20
camera_target = [0.2, -0.15, 0.3]

def update_camera():
    """อัปเดตมุมมองกล้อง"""
    p.resetDebugVisualizerCamera(camera_distance, camera_yaw, camera_pitch, camera_target)

# 3. รวบรวม Joints และ Links
joint_name_to_id = {}
link_name_to_id = {}

num_joints = p.getNumJoints(robotId)
print(f"--- Reading {num_joints} Joints from Quadruped URDF (testing FR leg only) ---")

for i in range(num_joints):
    joint_info = p.getJointInfo(robotId, i)
    joint_name = joint_info[1].decode('utf-8').replace(u'\xa0', u' ').strip()
    link_name = joint_info[12].decode('utf-8').replace(u'\xa0', u' ').strip()
    
    print(f"  Joint Index {i}: '{joint_name}' → Link: '{link_name}' (Type: {joint_info[2]})")
    
    joint_name_to_id[joint_name] = i
    if link_name:
        link_name_to_id[link_name] = i

# รวบรวม controllable joints (REVOLUTE)
controllable_joints_ids = []
for i in range(num_joints):
    joint_info = p.getJointInfo(robotId, i)
    joint_type = joint_info[2]
    if joint_type == p.JOINT_REVOLUTE:
        controllable_joints_ids.append(i)

# สร้าง mapping จาก joint ID ไปยัง index ใน IK result
joint_id_to_ik_index = {joint_id: idx for idx, joint_id in enumerate(controllable_joints_ids)}

print(f"\nControllable Joints: {controllable_joints_ids}")
print(f"Joint to IK Index Mapping: {joint_id_to_ik_index}\n")

# 4. กำหนด Foot Link และ Moveable Joints (เฉพาะขา FR)
foot_link_id = link_name_to_id['FR_foot_link']
moveable_joint_ids = [joint_name_to_id['FR_thigh_joint'], joint_name_to_id['FR_shank_joint']]

print(f"Foot Link ID (FR): {foot_link_id}")
print(f"Moveable Joint IDs (FR): {moveable_joint_ids}\n")

# 5. ตั้งค่าพารามิเตอร์การเคลื่อนไหว
# Elliptical Gait (วงรีแบบท่าเดินธรรมชาติ)
STEP_LENGTH = 0.10        # ความยาวก้าวในแนวหน้า-หลัง (100mm)
LIFT_HEIGHT = 0.06        # ความสูงยกขา (60mm)
HOME_POSITION = [0.18, -0.15, -0.20]  # ตำแหน่งยืนปกติของขา FR (x, y, z relative to base)
CYCLE_DURATION = 2.0      # ระยะเวลา 1 รอบการเดิน (วินาที)

sim_time = 0.0
time_step = 1./240.
JOINT_DAMPING = 0.5

# 6. Main Simulation Loop
print("--- เริ่มการจำลอง: Single Leg (FR) Elliptical Gait ---")
try:
    while True:
        # ตรวจสอบคีย์บอร์ด (ควบคุมกล้อง)
        keys = p.getKeyboardEvents()
        
        # มุมมอง Preset
        if ord('1') in keys and keys[ord('1')] & p.KEY_WAS_TRIGGERED:
            camera_distance, camera_yaw, camera_pitch = 0.8, 0, -15
            update_camera()
            print("📷 Front View")
        elif ord('2') in keys and keys[ord('2')] & p.KEY_WAS_TRIGGERED:
            camera_distance, camera_yaw, camera_pitch = 0.8, 90, -20
            update_camera()
            print("📷 Side View")
        elif ord('3') in keys and keys[ord('3')] & p.KEY_WAS_TRIGGERED:
            camera_distance, camera_yaw, camera_pitch = 1.2, 0, -89
            update_camera()
            print("📷 Top View")
        elif ord('4') in keys and keys[ord('4')] & p.KEY_WAS_TRIGGERED:
            camera_distance, camera_yaw, camera_pitch = 1.0, 45, -30
            update_camera()
            print("📷 Isometric View")
        
        # ปรับแต่งกล้องแบบละเอียด
        if ord('u') in keys and keys[ord('u')] & p.KEY_IS_DOWN:
            camera_yaw -= 2
            update_camera()
        if ord('i') in keys and keys[ord('i')] & p.KEY_IS_DOWN:
            camera_yaw += 2
            update_camera()
        if ord('j') in keys and keys[ord('j')] & p.KEY_IS_DOWN:
            camera_pitch = max(-89, camera_pitch - 1)
            update_camera()
        if ord('k') in keys and keys[ord('k')] & p.KEY_IS_DOWN:
            camera_pitch = min(89, camera_pitch + 1)
            update_camera()
        if ord('o') in keys and keys[ord('o')] & p.KEY_IS_DOWN:
            camera_distance = max(0.3, camera_distance - 0.02)
            update_camera()
        if ord('l') in keys and keys[ord('l')] & p.KEY_IS_DOWN:
            camera_distance = min(3.0, camera_distance + 0.02)
            update_camera()
        
        # เลื่อนกล้องซ้าย-ขวา (A/D)
        if ord('a') in keys and keys[ord('a')] & p.KEY_IS_DOWN:
            camera_target[0] -= 0.01
            update_camera()
        if ord('d') in keys and keys[ord('d')] & p.KEY_IS_DOWN:
            camera_target[0] += 0.01
            update_camera()
        
        # เลื่อนกล้องหน้า-หลัง (W/S)
        if ord('w') in keys and keys[ord('w')] & p.KEY_IS_DOWN:
            camera_target[1] += 0.01
            update_camera()
        if ord('s') in keys and keys[ord('s')] & p.KEY_IS_DOWN:
            camera_target[1] -= 0.01
            update_camera()
        
        # เลื่อนกล้องขึ้น-ลง (Q/E)
        if ord('q') in keys and keys[ord('q')] & p.KEY_IS_DOWN:
            camera_target[2] -= 0.01
            update_camera()
        if ord('e') in keys and keys[ord('e')] & p.KEY_IS_DOWN:
            camera_target[2] += 0.01
            update_camera()
        
        # รีเซ็ตกล้อง
        if ord('r') in keys and keys[ord('r')] & p.KEY_WAS_TRIGGERED:
            camera_distance, camera_yaw, camera_pitch = 0.8, 90, -20
            camera_target = [0.2, -0.15, 0.3]
            update_camera()
            print("📷 Reset Camera")
        
        sim_time += time_step
        
        # คำนวณพิกัดเป้าหมายแบบ Ellipse (วงรี) - ท่าเดินธรรมชาติ
        # Phase: 0.0 - 0.5 = Swing (ยกขาแกว่งไปข้างหน้า)
        # Phase: 0.5 - 1.0 = Stance (ขาอยู่พื้นเลื่อนกลับ)
        phase = (sim_time % CYCLE_DURATION) / CYCLE_DURATION  # 0.0 ถึง 1.0
        
        if phase < 0.5:  # Swing Phase (ยกขา)
            swing_progress = phase * 2.0  # 0.0 ถึง 1.0
            # เดินหน้า + ยกขาเป็นโค้ง
            x_offset = STEP_LENGTH * (swing_progress - 0.5)  # -0.05 ถึง +0.05
            z_offset = LIFT_HEIGHT * math.sin(swing_progress * math.pi)  # 0 → peak → 0
        else:  # Stance Phase (ขาพื้น)
            stance_progress = (phase - 0.5) * 2.0  # 0.0 ถึง 1.0
            # ถอยหลังเพื่อเตรียมก้าวถัดไป (ขาไม่ยก)
            x_offset = STEP_LENGTH * (0.5 - stance_progress)  # +0.05 ถึง -0.05
            z_offset = 0.0  # ขาอยู่พื้น
        
        target_pos_REL = [
            HOME_POSITION[0] + x_offset,
            HOME_POSITION[1],
            HOME_POSITION[2] + z_offset
        ]
        
        # แปลงเป็น World Coordinates
        basePos, baseOrn = p.getBasePositionAndOrientation(robotId)
        world_target_pos, _ = p.multiplyTransforms(basePos, baseOrn, target_pos_REL, [0, 0, 0, 1])
        
        # คำนวณ IK
        joint_angles_all = p.calculateInverseKinematics(
            robotId, foot_link_id, world_target_pos,
            jointDamping=[JOINT_DAMPING] * num_joints,
            maxNumIterations=50
        )
        
        # ดึงมุมของ thigh และ shank
        target_angles = [joint_angles_all[joint_id_to_ik_index[j]] for j in moveable_joint_ids]
        
        # ควบคุม Joint Motors
        p.setJointMotorControlArray(
            robotId, moveable_joint_ids, p.POSITION_CONTROL,
            targetPositions=target_angles,
            forces=[10] * len(moveable_joint_ids),
            positionGains=[0.5] * len(moveable_joint_ids),
            velocityGains=[0.7] * len(moveable_joint_ids)
        )
        
        # วาดเส้นแสดงตำแหน่งเป้าหมาย (Debug)
        color = [1, 0, 0] if phase < 0.5 else [0, 1, 0]  # แดง=Swing, เขียว=Stance
        p.addUserDebugLine(world_target_pos, [world_target_pos[0], world_target_pos[1], world_target_pos[2] + 0.05], 
                          color, lifeTime=time_step)
        
        # Step Simulation
        p.stepSimulation()
        time.sleep(time_step)

except KeyboardInterrupt:
    print("\n--- หยุดการจำลอง ---")
except Exception as e:
    print(f"\n--- เกิดข้อผิดพลาด: {e} ---")
finally:
    if p.isConnected():
        p.disconnect()
