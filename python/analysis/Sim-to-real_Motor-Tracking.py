import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
from scipy.interpolate import interp1d

# 1. โหลดข้อมูล Log ของจริงและจาก PyBullet
df_real = pd.read_csv('real_walk600_20260308_132624.csv')
df_sim = pd.read_csv('sim_walk600_20260302_204611.csv')

# 2. ตั้งค่าระยะเวลาที่จะแสดง (2 วินาทีเพื่อให้เห็นคลื่น 2-3 ลูกชัดเจน)
time_window = 2.0

legs = ['FL', 'FR', 'RL', 'RR']
joints = ['theta1', 'theta2']

# 3. สร้างตารางพล็อต 4 แถว (ขา) 2 คอลัมน์ (มอเตอร์)
fig, axs = plt.subplots(4, 2, figsize=(16, 16), sharex=True)
fig.suptitle('Motor Tracking Comparison: Real vs Sim (Walking Phase)', fontsize=20, fontweight='bold', y=0.98)

for i, leg in enumerate(legs):
    for j, joint in enumerate(joints):
        ax = axs[i, j]
        
        # ดึงข้อมูลเฉพาะเฟสเดินของขานั้นๆ
        real_data = df_real[(df_real['leg'] == leg) & (df_real['phase'] == 'WALK')].copy()
        sim_data = df_sim[(df_sim['leg'] == leg) & (df_sim['phase'] == 'WALKING')].copy()
        
        # ปรับเวลาให้เริ่มที่ 0 วินาทีพร้อมกัน
        real_data['time_norm'] = real_data['time'] - real_data['time'].min()
        sim_data['time_norm'] = sim_data['time_s'] - sim_data['time_s'].min()
        
        # ครอปข้อมูลตามหน้าต่างเวลา (Zoom window)
        real_zoom = real_data[real_data['time_norm'] <= time_window]
        sim_zoom = sim_data[sim_data['time_norm'] <= time_window].copy()
        
        col_setpoint = f'{joint}_setpoint_deg'
        col_actual = f'{joint}_actual_deg'
        
        # 4. พระเอกของงาน: คำนวณหา Offset เพื่อจัดศูนย์ (Alignment) แบบอัตโนมัติ
        mean_sim = sim_zoom[col_setpoint].mean()
        mean_real = real_zoom[col_actual].mean()
        offset = mean_real - mean_sim
        
        # ใส่ Offset ชดเชยลงในข้อมูลของ Sim
        sim_zoom[f'{col_setpoint}_aligned'] = sim_zoom[col_setpoint] + offset
        sim_zoom[f'{col_actual}_aligned'] = sim_zoom[col_actual] + offset
        
        # 5. พล็อตกราฟทั้ง 3 เส้น
        ax.plot(sim_zoom['time_norm'], sim_zoom[f'{col_setpoint}_aligned'], 
                color='black', linestyle='--', linewidth=2, label='Target Setpoint (Ideal)')
        ax.plot(sim_zoom['time_norm'], sim_zoom[f'{col_actual}_aligned'], 
                color='limegreen', linestyle='-', linewidth=2.5, alpha=0.8, label='Sim Actual (PyBullet)')
        ax.plot(real_zoom['time_norm'], real_zoom[col_actual], 
                color='red', linestyle='-', linewidth=2.5, alpha=0.8, label='Real Actual (Hardware)')
        
        # 6. จัดรูปแบบกราฟย่อย
        ax.set_title(f'Leg: {leg} | Motor: {joint.capitalize()} (Calibrated Offset: {offset:.1f}°)', fontsize=14)
        ax.set_ylabel('Angle (°)', fontsize=12)
        ax.grid(True, linestyle=':', alpha=0.7)
        
        # โชว์ Legend ที่กราฟแรกกราฟเดียว จะได้ดูสะอาดตา
        if i == 0 and j == 0:
            ax.legend(loc='upper right', fontsize=12)
            
        # ใส่ Label แกน X เฉพาะแถวล่างสุด
        if i == 3:
            ax.set_xlabel('Time in WALKING Phase (Seconds)', fontsize=14)

# 7. บันทึกรูปและแสดงผล
plt.tight_layout(rect=[0, 0, 1, 0.96])
plt.savefig('all_motors_aligned_comparison.png', dpi=300) # เซฟความละเอียดสูงสำหรับพรีเซนต์
plt.show()

# ============================================================
# 8. คำนวณและแสดงผล Angle RMSE & Frequency ของทุกมอเตอร์
# ============================================================

def estimate_dominant_frequency(time_arr, angle_arr):
    """ประมาณความถี่หลักของสัญญาณด้วย FFT"""
    signal = angle_arr - angle_arr.mean()
    dt = np.mean(np.diff(time_arr))
    if dt <= 0 or len(signal) < 4:
        return np.nan
    fs = 1.0 / dt
    n = len(signal)
    freqs = np.fft.rfftfreq(n, d=1.0 / fs)
    fft_mag = np.abs(np.fft.rfft(signal))
    # ข้ามค่า DC (index 0)
    dominant_idx = np.argmax(fft_mag[1:]) + 1
    return freqs[dominant_idx]

results = {}

for leg in legs:
    for joint in joints:
        motor_name = f"{leg}_{joint}"

        real_data = df_real[(df_real['leg'] == leg) & (df_real['phase'] == 'WALK')].copy()
        sim_data  = df_sim[(df_sim['leg'] == leg)  & (df_sim['phase'] == 'WALKING')].copy()

        real_data['time_norm'] = real_data['time']    - real_data['time'].min()
        sim_data['time_norm']  = sim_data['time_s']   - sim_data['time_s'].min()

        real_zoom = real_data[real_data['time_norm'] <= time_window].copy()
        sim_zoom  = sim_data[sim_data['time_norm']   <= time_window].copy()

        col_sp  = f'{joint}_setpoint_deg'
        col_act = f'{joint}_actual_deg'

        # ใช้ Offset เดิมเพื่อให้ได้ฐานเดียวกับกราฟ
        offset = real_zoom[col_act].mean() - sim_zoom[col_sp].mean()
        sim_zoom[f'{col_sp}_aligned']  = sim_zoom[col_sp]  + offset
        sim_zoom[f'{col_act}_aligned'] = sim_zoom[col_act] + offset

        # --- Angle RMSE ---
        sim_angle_rmse = np.sqrt(
            np.mean((sim_zoom[f'{col_act}_aligned'] - sim_zoom[f'{col_sp}_aligned']) ** 2)
        )

        # Interpolate target (aligned) onto real timestamps
        interp_target = interp1d(
            sim_zoom['time_norm'], sim_zoom[f'{col_sp}_aligned'],
            bounds_error=False, fill_value='extrapolate'
        )
        target_at_real = interp_target(real_zoom['time_norm'].values)
        real_angle_rmse = np.sqrt(np.mean((real_zoom[col_act].values - target_at_real) ** 2))

        # --- Dominant Frequency ---
        target_freq = estimate_dominant_frequency(
            sim_zoom['time_norm'].values, sim_zoom[f'{col_sp}_aligned'].values
        )
        sim_freq = estimate_dominant_frequency(
            sim_zoom['time_norm'].values, sim_zoom[f'{col_act}_aligned'].values
        )
        real_freq = estimate_dominant_frequency(
            real_zoom['time_norm'].values, real_zoom[col_act].values
        )

        results[motor_name] = dict(
            sim_angle_rmse  = sim_angle_rmse,
            real_angle_rmse = real_angle_rmse,
            target_freq     = target_freq,
            sim_freq        = sim_freq,
            real_freq       = real_freq,
            sim_freq_err    = abs(sim_freq  - target_freq),
            real_freq_err   = abs(real_freq - target_freq),
        )

# ---- แสดงผลรายมอเตอร์ ----
SEP = "=" * 80
sep = "-" * 80

print(f"\n{SEP}")
print("  MOTOR TRACKING ANALYSIS  —  Angle RMSE & Frequency (8 Motors)")
print(SEP)

for motor_name, r in results.items():
    print(f"\n  Motor : {motor_name}")
    print(f"  {sep}")
    print(f"  Angle RMSE      │  Sim  vs Target : {r['sim_angle_rmse']:>7.3f} °")
    print(f"                  │  Real vs Target : {r['real_angle_rmse']:>7.3f} °")
    print(f"  Frequency       │  Target         : {r['target_freq']:>7.3f} Hz")
    print(f"                  │  Sim            : {r['sim_freq']:>7.3f} Hz  (err {r['sim_freq_err']:+.3f} Hz)")
    print(f"                  │  Real           : {r['real_freq']:>7.3f} Hz  (err {r['real_freq_err']:+.3f} Hz)")

# ---- ตารางสรุป ----
print(f"\n{SEP}")
print("  SUMMARY TABLE")
print(SEP)
hdr = f"{'Motor':<14} {'SimRMSE(°)':>12} {'RealRMSE(°)':>13} {'Tgt Hz':>9} {'Sim Hz':>9} {'Real Hz':>9} {'SimΔHz':>9} {'RealΔHz':>9}"
print(hdr)
print(sep)
for motor_name, r in results.items():
    print(
        f"{motor_name:<14}"
        f"{r['sim_angle_rmse']:>12.3f}"
        f"{r['real_angle_rmse']:>13.3f}"
        f"{r['target_freq']:>9.3f}"
        f"{r['sim_freq']:>9.3f}"
        f"{r['real_freq']:>9.3f}"
        f"{r['sim_freq_err']:>9.3f}"
        f"{r['real_freq_err']:>9.3f}"
    )
print(SEP)