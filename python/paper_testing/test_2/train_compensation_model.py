import pandas as pd
import os
_DIR = os.path.dirname(os.path.abspath(__file__))
import numpy as np
import matplotlib.pyplot as plt
from sklearn.preprocessing import PolynomialFeatures
from sklearn.linear_model import LinearRegression
from sklearn.metrics import root_mean_squared_error, r2_score

def main():
    print("🚀 เริ่มต้นการเทรนแบบจำลองชดเชยความคลาดเคลื่อน (Kinematic Compensation Model)")
    print("-" * 60)
    
    # 1. โหลดข้อมูล Dataset ของเรา
    try:
        df = pd.read_csv(os.path.join(_DIR, 'analysis_results.csv'))
        print(f"✅ โหลดข้อมูลสำเร็จ: จำนวน {len(df)} จุดทดสอบ")
    except FileNotFoundError:
        print("❌ ไม่พบไฟล์ analysis_results.csv กรุณาตรวจสอบพาร์ท")
        return

    # 2. กำหนดตัวแปรต้น (Features) และตัวแปรตาม (Targets)
    X = df[['act_thetaA_deg', 'act_thetaB_deg']].values
    y_x = df['error_x_mm'].values
    y_y = df['error_y_mm'].values

    # 3. สร้างฟีเจอร์พหุนาม (Polynomial Features Degree 2)
    # ลำดับฟีเจอร์ที่จะได้: 1, thetaA, thetaB, thetaA^2, thetaA*thetaB, thetaB^2
    poly = PolynomialFeatures(degree=2, include_bias=False)
    X_poly = poly.fit_transform(X)

    # 4. เทรนโมเดล (Linear Regression ซ้อนบน Polynomial Features)
    model_x = LinearRegression()
    model_x.fit(X_poly, y_x)
    pred_x = model_x.predict(X_poly)

    model_y = LinearRegression()
    model_y.fit(X_poly, y_y)
    pred_y = model_y.predict(X_poly)

    # 5. ประเมินและแสดงผลสมรรถนะโมเดล
    rmse_x_before = root_mean_squared_error(y_x, [0] * len(y_x))
    rmse_x_after  = root_mean_squared_error(y_x, pred_x)
    r2_x          = r2_score(y_x, pred_x)
    rmse_y_before = root_mean_squared_error(y_y, [0] * len(y_y))
    rmse_y_after  = root_mean_squared_error(y_y, pred_y)
    r2_y          = r2_score(y_y, pred_y)

    print("\n📊 ผลการประเมินความแม่นยำ (Training Metrics):")
    print(f"[แกน X] RMSE ก่อนชดเชย: {rmse_x_before:.3f} mm  ->  หลังชดเชย: {rmse_x_after:.3f} mm (R2 = {r2_x:.3f})")
    print(f"[แกน Y] RMSE ก่อนชดเชย: {rmse_y_before:.3f} mm  ->  หลังชดเชย: {rmse_y_after:.3f} mm (R2 = {r2_y:.3f})")

    # 6. ดึงค่าสัมประสิทธิ์เพื่อนำไปใช้กับ STM32 (C/C++)
    C0_x = model_x.intercept_
    C_x = model_x.coef_
    
    C0_y = model_y.intercept_
    C_y = model_y.coef_

    # 7. สร้างและพิมพ์โค้ด C/C++ ออกมาทางหน้าจอ
    generate_cpp_code(C0_x, C_x, C0_y, C_y)

    # 8. สร้างกราฟ 3D Surface Map สำหรับเปเปอร์
    plot_3d_surface(X, y_x, y_y, poly, model_x, model_y)

    # 9. บันทึกผลลัพธ์ทั้งหมดลงไฟล์
    metrics = {
        'rmse_x_before': rmse_x_before, 'rmse_x_after': rmse_x_after, 'r2_x': r2_x,
        'rmse_y_before': rmse_y_before, 'rmse_y_after': rmse_y_after, 'r2_y': r2_y,
    }
    save_results(metrics, C0_x, C_x, C0_y, C_y)


def save_results(metrics: dict, C0_x, C_x, C0_y, C_y) -> None:
    """บันทึกผลลัพธ์การเทรนลงไฟล์ JSON และ text summary"""
    import json

    # บันทึก coefficients + metrics → JSON
    model_data = {
        'polynomial_degree': 2,
        'feature_names': ['thetaA', 'thetaB', 'thetaA^2', 'thetaA*thetaB', 'thetaB^2'],
        'model_x': {'intercept': float(C0_x), 'coef': [float(c) for c in C_x]},
        'model_y': {'intercept': float(C0_y), 'coef': [float(c) for c in C_y]},
        'metrics': {k: float(v) for k, v in metrics.items()},
    }
    json_path = os.path.join(_DIR, 'compensation_model.json')
    with open(json_path, 'w', encoding='utf-8') as f:
        json.dump(model_data, f, indent=2)
    print(f"💾 บันทึกโมเดล (JSON)  : {json_path}")

    # บันทึก training summary → text
    txt_path = os.path.join(_DIR, 'training_summary.txt')
    with open(txt_path, 'w', encoding='utf-8') as f:
        f.write("Kinematic Compensation Model — Training Summary\n")
        f.write("=" * 52 + "\n\n")
        f.write(f"[X]  RMSE before : {metrics['rmse_x_before']:.4f} mm\n")
        f.write(f"     RMSE after  : {metrics['rmse_x_after']:.4f} mm\n")
        f.write(f"     R²          : {metrics['r2_x']:.4f}\n\n")
        f.write(f"[Y]  RMSE before : {metrics['rmse_y_before']:.4f} mm\n")
        f.write(f"     RMSE after  : {metrics['rmse_y_after']:.4f} mm\n")
        f.write(f"     R²          : {metrics['r2_y']:.4f}\n\n")
        f.write("Coefficients (X): intercept={:.6f}  coef={}\n".format(
            float(C0_x), [f"{c:.6f}" for c in C_x]))
        f.write("Coefficients (Y): intercept={:.6f}  coef={}\n".format(
            float(C0_y), [f"{c:.6f}" for c in C_y]))
    print(f"💾 บันทึกสรุปผล (TXT) : {txt_path}")


def generate_cpp_code(C0_x, C_x, C0_y, C_y):
    """ฟังก์ชันสำหรับสร้างโค้ดภาษา C/C++ และบันทึกเป็นไฟล์ .h"""
    cpp_code = f"""
\n{"="*60}
// โค้ด C/C++ สำหรับนำไปฝังในฟังก์ชัน Inverse Kinematics ของ STM32
// คัดลอกส่วนนี้ไปใช้งานได้เลยครับ
{"="*60}

// ฟังก์ชันชดเชยความคลาดเคลื่อน (Data-driven Kinematic Compensation)
void apply_kinematic_compensation(float thetaA_deg, float thetaB_deg, float* target_x_mm, float* target_y_mm) {{
    
    // คำนวณตัวแปรพหุนาม
    float tA = thetaA_deg;
    float tB = thetaB_deg;
    float tA2 = tA * tA;
    float tB2 = tB * tB;
    float tAB = tA * tB;

    // คำนวณ Error แกน X (mm) จากสมการที่เทรนมาได้
    float err_x = {C0_x: .6f}f 
                + ({C_x[0]: .6f}f * tA) 
                + ({C_x[1]: .6f}f * tB) 
                + ({C_x[2]: .6f}f * tA2) 
                + ({C_x[3]: .6f}f * tAB) 
                + ({C_x[4]: .6f}f * tB2);

    // คำนวณ Error แกน Y (mm) จากสมการที่เทรนมาได้
    float err_y = {C0_y: .6f}f 
                + ({C_y[0]: .6f}f * tA) 
                + ({C_y[1]: .6f}f * tB) 
                + ({C_y[2]: .6f}f * tA2) 
                + ({C_y[3]: .6f}f * tAB) 
                + ({C_y[4]: .6f}f * tB2);

    // ปรับแก้พิกัดเป้าหมาย (นำเป้าหมายอุดมคติ มาลบด้วย Error ที่คาดการณ์)
    *target_x_mm = *target_x_mm - err_x;
    *target_y_mm = *target_y_mm - err_y;
}}
"""
    print(cpp_code)

    h_path = os.path.join(_DIR, 'compensation_model.h')
    with open(h_path, 'w', encoding='utf-8') as f:
        f.write(cpp_code)
    print(f"💾 บันทึกโค้ด C/C++ (H): {h_path}")


def plot_3d_surface(X, y_x, y_y, poly, model_x, model_y):
    """ฟังก์ชันสำหรับสร้างกราฟ 3D Surface และเซฟเป็นไฟล์รูปภาพ"""
    print(f"\n🎨 กำลังสร้างรูปภาพ 3D Surface Map...")
    fig = plt.figure(figsize=(16, 7))

    # สร้างกริดสมมติเพื่อวาดพื้นผิว
    tA_range = np.linspace(X[:,0].min() - 5, X[:,0].max() + 5, 30)
    tB_range = np.linspace(X[:,1].min() - 5, X[:,1].max() + 5, 30)
    TA, TB = np.meshgrid(tA_range, tB_range)
    
    mesh_X = np.c_[TA.ravel(), TB.ravel()]
    mesh_X_poly = poly.transform(mesh_X)

    pred_mesh_x = model_x.predict(mesh_X_poly).reshape(TA.shape)
    pred_mesh_y = model_y.predict(mesh_X_poly).reshape(TA.shape)

    # วาดแกน X
    ax1 = fig.add_subplot(121, projection='3d')
    ax1.scatter(X[:,0], X[:,1], y_x, color='red', label='Measured Error', s=30, alpha=0.8)
    surf1 = ax1.plot_surface(TA, TB, pred_mesh_x, cmap='viridis', alpha=0.6)
    ax1.set_title('Error X Surface Map')
    ax1.set_xlabel('Theta A (deg)')
    ax1.set_ylabel('Theta B (deg)')
    ax1.set_zlabel('Error X (mm)')
    ax1.view_init(elev=20, azim=45) # ปรับมุมมองกล้อง
    fig.colorbar(surf1, ax=ax1, shrink=0.5, aspect=10)

    # วาดแกน Y
    ax2 = fig.add_subplot(122, projection='3d')
    ax2.scatter(X[:,0], X[:,1], y_y, color='red', label='Measured Error', s=30, alpha=0.8)
    surf2 = ax2.plot_surface(TA, TB, pred_mesh_y, cmap='plasma', alpha=0.6)
    ax2.set_title('Error Y Surface Map')
    ax2.set_xlabel('Theta A (deg)')
    ax2.set_ylabel('Theta B (deg)')
    ax2.set_zlabel('Error Y (mm)')
    ax2.view_init(elev=20, azim=45) # ปรับมุมมองกล้อง
    fig.colorbar(surf2, ax=ax2, shrink=0.5, aspect=10)

    plt.tight_layout()
    filename = os.path.join(_DIR, 'error_surface_maps.png')
    plt.savefig(filename, dpi=300, bbox_inches='tight')
    print(f"💾 บันทึกรูปภาพสำเร็จ: {filename} (พร้อมนำไปใส่ในเปเปอร์แล้ว!)")


if __name__ == "__main__":
    main()