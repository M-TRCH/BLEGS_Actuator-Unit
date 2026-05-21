import pandas as pd
import os
import json
import numpy as np
import matplotlib.pyplot as plt
from sklearn.preprocessing import PolynomialFeatures
from sklearn.linear_model import LinearRegression
from sklearn.metrics import root_mean_squared_error, r2_score

_DIR = os.path.dirname(os.path.abspath(__file__))

def main():
    print("🚀 เริ่มต้นการเทรนแบบจำลองชดเชยความคลาดเคลื่อน (Kinematic Compensation Model) v3.0 (Degree 4)")
    print("-" * 60)
    
    POLY_DEGREE = 4
    MODEL_NAME  = f"compensation_model_poly{POLY_DEGREE}"
    
    # 1. โหลดข้อมูล Dataset จากระบบ Auto-Calibration (4K Video)
    csv_path = os.path.join(_DIR, 'output', 'data', 'grid_log.csv')
    try:
        df = pd.read_csv(csv_path)
        print(f"✅ โหลดข้อมูลสำเร็จ: จำนวน {len(df)} จุดทดสอบ จากไฟล์ grid_log.csv")
    except FileNotFoundError:
        print(f"❌ ไม่พบไฟล์ {csv_path} กรุณาตรวจสอบพาร์ท")
        return

    # 2. กำหนดตัวแปรต้น (Features) และตัวแปรตาม (Targets)
    X = df[['cmd_thetaA_deg', 'cmd_thetaB_deg']].values
    y_x = df['video_err_x_mm'].values
    y_y = df['video_err_y_mm'].values

    # 3. สร้างฟีเจอร์พหุนาม (Polynomial Features Degree 4)
    # สมการจะมีทั้งหมด 14 พจน์ (ไม่รวม Bias) 
    poly = PolynomialFeatures(degree=POLY_DEGREE, include_bias=False)
    X_poly = poly.fit_transform(X)

    # 4. เทรนโมเดล (Training)
    model_x = LinearRegression()
    model_x.fit(X_poly, y_x)
    
    model_y = LinearRegression()
    model_y.fit(X_poly, y_y)

    # 5. ประเมินผลความแม่นยำ (Evaluation)
    pred_x = model_x.predict(X_poly)
    pred_y = model_y.predict(X_poly)
    
    rmse_x = root_mean_squared_error(y_x, pred_x)
    rmse_y = root_mean_squared_error(y_y, pred_y)
    r2_x = r2_score(y_x, pred_x)
    r2_y = r2_score(y_y, pred_y)

    print(f"\n📊 ผลการประเมินความแม่นยำ (Model Evaluation) [Polynomial Degree {POLY_DEGREE}]:")
    print(f"แกน X -> RMSE: {rmse_x:.4f} mm | R-Squared (ความแนบเนียน): {r2_x:.4f}")
    print(f"แกน Y -> RMSE: {rmse_y:.4f} mm | R-Squared (ความแนบเนียน): {r2_y:.4f}")

    # 6. บันทึกโมเดลสำหรับนำไปใช้จริง (Export JSON)
    # ใช้ฟังก์ชันดึงชื่อตัวแปรอัตโนมัติ เพื่อรองรับ 14 พจน์ของ Degree 4
    feature_names = poly.get_feature_names_out(['thetaA', 'thetaB']).tolist()
    
    model_data = {
        "model_name": MODEL_NAME,
        "polynomial_degree": POLY_DEGREE,
        "feature_names": feature_names,
        "model_x": {
            "intercept": model_x.intercept_,
            "coef": model_x.coef_.tolist()
        },
        "model_y": {
            "intercept": model_y.intercept_,
            "coef": model_y.coef_.tolist()
        }
    }

    # สร้างโฟลเดอร์ output/models ถ้ายังไม่มี เพื่อป้องกัน FileNotFoundError
    output_dir = os.path.join(_DIR, 'output', 'models')
    os.makedirs(output_dir, exist_ok=True)
    
    json_path = os.path.join(output_dir, f'{MODEL_NAME}.json')
    with open(json_path, 'w', encoding='utf-8') as f:
        json.dump(model_data, f, indent=4)
    print(f"\n💾 บันทึกค่าสัมประสิทธิ์ลงไฟล์สำเร็จ: {json_path}")

    # 7. วาดกราฟ 3D Surface Map
    fig = plt.figure(figsize=(14, 6))

    tA_range = np.linspace(X[:,0].min(), X[:,0].max(), 30)
    tB_range = np.linspace(X[:,1].min(), X[:,1].max(), 30)
    TA, TB = np.meshgrid(tA_range, tB_range)
    
    mesh_X = np.c_[TA.ravel(), TB.ravel()]
    mesh_X_poly = poly.transform(mesh_X)

    pred_mesh_x = model_x.predict(mesh_X_poly).reshape(TA.shape)
    pred_mesh_y = model_y.predict(mesh_X_poly).reshape(TA.shape)

    ax1 = fig.add_subplot(121, projection='3d')
    ax1.scatter(X[:,0], X[:,1], y_x, color='red', label='Measured Video Error', s=30, alpha=0.8, edgecolors='k')
    surf1 = ax1.plot_surface(TA, TB, pred_mesh_x, cmap='viridis', alpha=0.7, edgecolor='none')
    ax1.set_title(f'Error X Surface Map (Poly deg={POLY_DEGREE}, RMSE: {rmse_x:.3f} mm)', fontweight='bold')
    ax1.set_xlabel('Command Theta A (deg)')
    ax1.set_ylabel('Command Theta B (deg)')
    ax1.set_zlabel('Error X (mm)')
    ax1.view_init(elev=20, azim=-45)
    fig.colorbar(surf1, ax=ax1, shrink=0.5, aspect=10, pad=0.1)

    ax2 = fig.add_subplot(122, projection='3d')
    ax2.scatter(X[:,0], X[:,1], y_y, color='red', label='Measured Video Error', s=30, alpha=0.8, edgecolors='k')
    surf2 = ax2.plot_surface(TA, TB, pred_mesh_y, cmap='plasma', alpha=0.7, edgecolor='none')
    ax2.set_title(f'Error Y Surface Map (Poly deg={POLY_DEGREE}, RMSE: {rmse_y:.3f} mm)', fontweight='bold')
    ax2.set_xlabel('Command Theta A (deg)')
    ax2.set_ylabel('Command Theta B (deg)')
    ax2.set_zlabel('Error Y (mm)')
    ax2.view_init(elev=20, azim=-45)
    fig.colorbar(surf2, ax=ax2, shrink=0.5, aspect=10, pad=0.1)

    plt.tight_layout()

    plots_dir  = os.path.join(_DIR, 'output', 'plots')
    os.makedirs(plots_dir, exist_ok=True)
    plot_path = os.path.join(plots_dir, f'{MODEL_NAME}_surface_map.png')
    plt.savefig(plot_path, dpi=150, bbox_inches='tight')
    print(f"📈 บันทึกกราฟ: {plot_path}")

    plt.show()

if __name__ == "__main__":
    main()