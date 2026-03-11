import os
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

# =========================================================
# 1. ตั้งค่า Path
# =========================================================
df_vid_new = pd.read_csv('linkage_20260310_222906.csv')
df_sim = pd.read_csv('sim_walk600_20260302_204611.csv')

# =========================================================
# 2. กรองข้อมูล (Filtering) ด้วยช่วงเวลาที่กำหนด
# =========================================================
# เลือกเฉพาะวินาทีที่ 3.0 ถึง 6.0 ของวิดีโอ
start_time = 3.0
end_time = 6.0
df_vid_filtered = df_vid_new[(df_vid_new['timestamp_s'] >= start_time) & (df_vid_new['timestamp_s'] <= end_time)].copy()

# ดึงข้อมูลจาก PyBullet (เฉพาะขา FL เฟสเดิน)
df_sim_fl = df_sim[(df_sim['leg'] == 'FL') & (df_sim['phase'] == 'WALKING')].copy()

# =========================================================
# 3. จัดเตรียมพิกัด
# =========================================================
real_x = df_vid_filtered['E_x_mm']
real_y = df_vid_filtered['E_y_mm']

sim_x_center = df_sim_fl['x_actual_mm'].mean()
sim_x = df_sim_fl['x_actual_mm'] - sim_x_center
sim_z = df_sim_fl['z_actual_mm']

sim_x_set_center = df_sim_fl['x_setpoint_mm'].mean()
sim_x_set = df_sim_fl['x_setpoint_mm'] - sim_x_set_center
sim_z_set = df_sim_fl['z_setpoint_mm']

# คำนวณความสูงในการยกเท้า
real_lift = real_y.max() - real_y.min()
sim_lift = sim_z.max() - sim_z.min()

# =========================================================
# 4. พล็อตกราฟเปรียบเทียบ
# =========================================================
plt.figure(figsize=(10, 6))

plt.plot(sim_x_set, sim_z_set, color='black', linestyle='--', linewidth=2, label='Target Trajectory')
plt.plot(sim_x, sim_z, color='limegreen', linestyle='-', linewidth=2.5, alpha=0.8, label='Simulated Actual (PyBullet)')
plt.plot(real_x, real_y, color='red', linestyle='-', linewidth=2.5, alpha=0.8, label=f'Real Robot Actual')

plt.title('Foot Trajectory Comparison (Walking Phase)', fontsize=16, fontweight='bold')
plt.xlabel('Forward Progression X (mm)', fontsize=14)
plt.ylabel('Foot Height Y/Z (mm)', fontsize=14)

plt.gca().set_aspect('equal', adjustable='box')
# Set Y-axis scale to [-220, -190] as requested
plt.ylim(-220, -190)
plt.legend(loc='upper right', fontsize=12)
plt.grid(True, linestyle=':', alpha=0.7)

plt.tight_layout()
plt.savefig('foot_trajectory_new_data.png', dpi=300)

# =========================================================
# 5. Numerical summary
# =========================================================
print("\n===== Foot Trajectory Comparison Summary =====")
print(f"Video window         : {start_time:.1f} s – {end_time:.1f} s  ({len(df_vid_filtered)} frames)")
print()
print("  Real robot (AR-tag measured):")
print(f"    X range : {real_x.min():.1f} – {real_x.max():.1f} mm  (span = {real_x.max()-real_x.min():.1f} mm)")
print(f"    Y range : {real_y.min():.1f} – {real_y.max():.1f} mm  (lift = {real_lift:.1f} mm)")
print()
print("  Simulation actual (PyBullet – FL walking):")
print(f"    X range : {sim_x.min():.1f} – {sim_x.max():.1f} mm  (span = {sim_x.max()-sim_x.min():.1f} mm)")
print(f"    Z range : {sim_z.min():.1f} – {sim_z.max():.1f} mm  (lift = {sim_lift:.1f} mm)")
print()
print("  Simulation setpoint (ideal cycloid):")
print(f"    X range : {sim_x_set.min():.1f} – {sim_x_set.max():.1f} mm  (span = {(sim_x_set.max()-sim_x_set.min()):.1f} mm)")
print(f"    Z range : {sim_z_set.min():.1f} – {sim_z_set.max():.1f} mm  (lift = {(sim_z_set.max()-sim_z_set.min()):.1f} mm)")
print()
print(f"  Lift diff (real vs sim) : {real_lift - sim_lift:+.1f} mm")
print()

# =========================================================
# 6. RMSE vs Target (interpolated onto common X grid)
#    Strategy: use target's X range as the reference grid.
#    Sort each trajectory by X, interpolate Y/Z onto that grid,
#    compute RMSE only over the overlapping X span.
# =========================================================
# Sort all trajectories by X
set_sort  = np.argsort(sim_x_set.values)
target_x  = sim_x_set.values[set_sort]
target_z  = sim_z_set.values[set_sort]

sim_sort  = np.argsort(sim_x.values)
sim_xs    = sim_x.values[sim_sort]
sim_zs    = sim_z.values[sim_sort]

real_sort = np.argsort(real_x.values)
real_xs   = real_x.values[real_sort]
real_ys   = real_y.values[real_sort]

# Common X grid = overlap of all three
x_lo = max(target_x.min(), sim_xs.min(), real_xs.min())
x_hi = min(target_x.max(), sim_xs.max(), real_xs.max())
n_grid = 500
x_grid = np.linspace(x_lo, x_hi, n_grid)

# Interpolate each onto x_grid
target_interp = np.interp(x_grid, target_x, target_z)
sim_interp    = np.interp(x_grid, sim_xs,   sim_zs)
real_interp   = np.interp(x_grid, real_xs,  real_ys)

rmse_sim  = np.sqrt(np.mean((sim_interp  - target_interp) ** 2))
rmse_real = np.sqrt(np.mean((real_interp - target_interp) ** 2))

print("  RMSE vs Target (over overlapping X span):")
print(f"    Sim  actual  RMSE : {rmse_sim:.2f} mm")
print(f"    Real actual  RMSE : {rmse_real:.2f} mm")
print(f"    Diff (real-sim)   : {rmse_real - rmse_sim:+.2f} mm")
print(f"    X overlap used    : {x_lo:.1f} to {x_hi:.1f} mm ({x_hi-x_lo:.1f} mm wide)")
print()

# ── Lift-peak RMSE (Y/Z max only) ──────────────────────────
# RMSE of a single value = |error|
target_z_max    = sim_z_set.max()
sim_z_max       = sim_z.max()
real_y_max      = real_y.max()

rmse_sim_lift  = abs(sim_z_max  - target_z_max)
rmse_real_lift = abs(real_y_max - target_z_max)

print("  Lift-peak RMSE vs Target (Y/Z max only):")
print(f"    Target  Z max     : {target_z_max:.2f} mm")
print(f"    Sim     Z max     : {sim_z_max:.2f} mm  →  error = {sim_z_max - target_z_max:+.2f} mm  (RMSE = {rmse_sim_lift:.2f} mm)")
print(f"    Real    Y max     : {real_y_max:.2f} mm  →  error = {real_y_max - target_z_max:+.2f} mm  (RMSE = {rmse_real_lift:.2f} mm)")
print(f"    Diff (real-sim)   : {rmse_real_lift - rmse_sim_lift:+.2f} mm")
print("="*46)

plt.show()