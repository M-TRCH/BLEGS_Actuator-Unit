import pandas as pd
import os
import json
import numpy as np
import matplotlib.pyplot as plt
import joblib
from sklearn.preprocessing import PolynomialFeatures, StandardScaler
from sklearn.linear_model import LinearRegression
from sklearn.svm import SVR
from sklearn.ensemble import RandomForestRegressor
from sklearn.neural_network import MLPRegressor
from sklearn.pipeline import Pipeline
from sklearn.metrics import root_mean_squared_error, r2_score

_DIR       = os.path.dirname(os.path.abspath(__file__))
MODELS_DIR = os.path.join(_DIR, 'output', 'models')
PLOTS_DIR  = os.path.join(_DIR, 'output', 'plots')
DATA_DIR   = os.path.join(_DIR, 'output', 'data')


# ── Helpers ───────────────────────────────────────────────────────────────────

def _train_and_eval(pipe_x, pipe_y, X, y_x, y_y):
    """Fit both pipelines and return (rmse_x, rmse_y, r2_x, r2_y)."""
    pipe_x.fit(X, y_x)
    pipe_y.fit(X, y_y)
    pred_x = pipe_x.predict(X)
    pred_y = pipe_y.predict(X)
    return (
        root_mean_squared_error(y_x, pred_x),
        root_mean_squared_error(y_y, pred_y),
        r2_score(y_x, pred_x),
        r2_score(y_y, pred_y),
    )


def _save_pkl(pipe_x, pipe_y, model_name, metrics):
    """Save model pair + metrics as .pkl (joblib compress=3)."""
    path = os.path.join(MODELS_DIR, f'{model_name}.pkl')
    joblib.dump(
        {"model_name": model_name, "model_x": pipe_x,
         "model_y": pipe_y, "metrics": metrics},
        path, compress=3,
    )
    return path


def _save_poly_json(pipe_x, pipe_y, model_name, poly_degree):
    """Export polynomial regression coefficients as human-readable JSON."""
    poly = pipe_x.named_steps['poly']
    lr_x = pipe_x.named_steps['lr']
    lr_y = pipe_y.named_steps['lr']
    data = {
        "model_name": model_name,
        "polynomial_degree": poly_degree,
        "feature_names": poly.get_feature_names_out(['thetaA', 'thetaB']).tolist(),
        "model_x": {"intercept": lr_x.intercept_, "coef": lr_x.coef_.tolist()},
        "model_y": {"intercept": lr_y.intercept_, "coef": lr_y.coef_.tolist()},
    }
    path = os.path.join(MODELS_DIR, f'{model_name}.json')
    with open(path, 'w', encoding='utf-8') as f:
        json.dump(data, f, indent=4)
    return path


def _save_surface_map(X, y_x, y_y, pipe_x, pipe_y, model_name, label, rmse_x, rmse_y):
    """Save 3-D surface map (X-error and Y-error side by side) to plots dir."""
    tA = np.linspace(X[:, 0].min(), X[:, 0].max(), 30)
    tB = np.linspace(X[:, 1].min(), X[:, 1].max(), 30)
    TA, TB = np.meshgrid(tA, tB)
    mesh = np.c_[TA.ravel(), TB.ravel()]
    zx = pipe_x.predict(mesh).reshape(TA.shape)
    zy = pipe_y.predict(mesh).reshape(TA.shape)

    fig = plt.figure(figsize=(14, 6))

    ax1 = fig.add_subplot(121, projection='3d')
    ax1.scatter(X[:, 0], X[:, 1], y_x, c='red', s=20, alpha=0.6,
                edgecolors='k', linewidths=0.3, label='Measured')
    sf1 = ax1.plot_surface(TA, TB, zx, cmap='viridis', alpha=0.7, edgecolor='none')
    ax1.set_title(f'Error X  [{label}]\nRMSE = {rmse_x:.3f} mm', fontweight='bold')
    ax1.set_xlabel('Cmd θA (deg)'); ax1.set_ylabel('Cmd θB (deg)'); ax1.set_zlabel('Err X (mm)')
    ax1.view_init(elev=20, azim=-45)
    fig.colorbar(sf1, ax=ax1, shrink=0.5, aspect=10, pad=0.1)

    ax2 = fig.add_subplot(122, projection='3d')
    ax2.scatter(X[:, 0], X[:, 1], y_y, c='red', s=20, alpha=0.6,
                edgecolors='k', linewidths=0.3, label='Measured')
    sf2 = ax2.plot_surface(TA, TB, zy, cmap='plasma', alpha=0.7, edgecolor='none')
    ax2.set_title(f'Error Y  [{label}]\nRMSE = {rmse_y:.3f} mm', fontweight='bold')
    ax2.set_xlabel('Cmd θA (deg)'); ax2.set_ylabel('Cmd θB (deg)'); ax2.set_zlabel('Err Y (mm)')
    ax2.view_init(elev=20, azim=-45)
    fig.colorbar(sf2, ax=ax2, shrink=0.5, aspect=10, pad=0.1)

    plt.tight_layout()
    path = os.path.join(PLOTS_DIR, f'{model_name}_surface_map.png')
    plt.savefig(path, dpi=150, bbox_inches='tight')
    plt.close()
    return path


def _save_comparison_chart(results):
    """Save bar chart comparing training RMSE of all models."""
    labels = [r['label'] for r in results]
    rmse_x = [r['rmse_x'] for r in results]
    rmse_y = [r['rmse_y'] for r in results]
    x = np.arange(len(labels))
    w = 0.35

    fig, ax = plt.subplots(figsize=(12, 5))
    b1 = ax.bar(x - w / 2, rmse_x, w, label='RMSE X', color='steelblue')
    b2 = ax.bar(x + w / 2, rmse_y, w, label='RMSE Y', color='tomato')
    ax.set_ylabel('RMSE (mm)')
    ax.set_title('Model Comparison – Training RMSE', fontweight='bold')
    ax.set_xticks(x)
    ax.set_xticklabels(labels, rotation=20, ha='right')
    ax.legend()
    ax.bar_label(b1, fmt='%.3f', padding=3, fontsize=8)
    ax.bar_label(b2, fmt='%.3f', padding=3, fontsize=8)
    ax.grid(axis='y', alpha=0.3)
    plt.tight_layout()
    path = os.path.join(PLOTS_DIR, 'model_comparison_rmse.png')
    plt.savefig(path, dpi=150, bbox_inches='tight')
    plt.close()
    return path


# ── Main ──────────────────────────────────────────────────────────────────────

def main():
    print("🚀 Multi-Model Compensation Training  v4.0")
    print("=" * 70)

    # Load dataset
    csv_path = os.path.join(DATA_DIR, 'grid_log.csv')
    try:
        df = pd.read_csv(csv_path)
        print(f"✅ โหลดข้อมูลสำเร็จ: {len(df)} จุด จาก grid_log.csv")
    except FileNotFoundError:
        print(f"❌ ไม่พบไฟล์: {csv_path}")
        return

    X   = df[['cmd_thetaA_deg', 'cmd_thetaB_deg']].values
    y_x = df['video_err_x_mm'].values
    y_y = df['video_err_y_mm'].values

    os.makedirs(MODELS_DIR, exist_ok=True)
    os.makedirs(PLOTS_DIR,  exist_ok=True)

    # ── Model configurations ──────────────────────────────────────────────────
    # Tuple: (model_name, display_label, pipe_x, pipe_y, is_poly, poly_degree)
    model_configs = [
        (
            "model_poly4",
            "Poly deg=4",
            Pipeline([('poly', PolynomialFeatures(degree=4, include_bias=False)),
                      ('lr',   LinearRegression())]),
            Pipeline([('poly', PolynomialFeatures(degree=4, include_bias=False)),
                      ('lr',   LinearRegression())]),
            True, 4,
        ),
        (
            "model_poly3",
            "Poly deg=3",
            Pipeline([('poly', PolynomialFeatures(degree=3, include_bias=False)),
                      ('lr',   LinearRegression())]),
            Pipeline([('poly', PolynomialFeatures(degree=3, include_bias=False)),
                      ('lr',   LinearRegression())]),
            True, 3,
        ),
        (
            "model_svr_rbf",
            "SVR (RBF)",
            Pipeline([('scaler', StandardScaler()),
                      ('svr',    SVR(kernel='rbf', C=100.0, gamma='scale', epsilon=0.05))]),
            Pipeline([('scaler', StandardScaler()),
                      ('svr',    SVR(kernel='rbf', C=100.0, gamma='scale', epsilon=0.05))]),
            False, None,
        ),
        (
            "model_random_forest",
            "Random Forest",
            Pipeline([('rf', RandomForestRegressor(n_estimators=200, max_depth=None,
                                                   random_state=42, n_jobs=-1))]),
            Pipeline([('rf', RandomForestRegressor(n_estimators=200, max_depth=None,
                                                   random_state=42, n_jobs=-1))]),
            False, None,
        ),
        (
            "model_mlp",
            "MLP (ANN)",
            Pipeline([('scaler', StandardScaler()),
                      ('mlp',    MLPRegressor(hidden_layer_sizes=(128, 64, 32),
                                              activation='relu', solver='adam',
                                              max_iter=3000, random_state=42,
                                              early_stopping=True, n_iter_no_change=30))]),
            Pipeline([('scaler', StandardScaler()),
                      ('mlp',    MLPRegressor(hidden_layer_sizes=(128, 64, 32),
                                              activation='relu', solver='adam',
                                              max_iter=3000, random_state=42,
                                              early_stopping=True, n_iter_no_change=30))]),
            False, None,
        ),
    ]

    results = []

    for model_name, label, pipe_x, pipe_y, is_poly, poly_deg in model_configs:
        print(f"\n{'─'*60}")
        print(f"🔧 เทรน: {label}  →  {model_name}")
        rmse_x, rmse_y, r2_x, r2_y = _train_and_eval(pipe_x, pipe_y, X, y_x, y_y)
        print(f"   X  RMSE={rmse_x:.4f} mm   R²={r2_x:.4f}")
        print(f"   Y  RMSE={rmse_y:.4f} mm   R²={r2_y:.4f}")

        metrics = dict(rmse_x=rmse_x, rmse_y=rmse_y, r2_x=r2_x, r2_y=r2_y)

        pkl_path = _save_pkl(pipe_x, pipe_y, model_name, metrics)
        print(f"   💾 PKL  → {pkl_path}")

        if is_poly:
            json_path = _save_poly_json(pipe_x, pipe_y, model_name, poly_deg)
            print(f"   💾 JSON → {json_path}")

        plot_path = _save_surface_map(X, y_x, y_y, pipe_x, pipe_y,
                                      model_name, label, rmse_x, rmse_y)
        print(f"   📈 Plot → {plot_path}")

        results.append(dict(label=label, model_name=model_name,
                            rmse_x=rmse_x, rmse_y=rmse_y, r2_x=r2_x, r2_y=r2_y))

    # Comparison chart
    print(f"\n{'─'*60}")
    cmp_path = _save_comparison_chart(results)
    print(f"📊 กราฟเปรียบเทียบ → {cmp_path}")

    # Summary table
    print(f"\n{'='*70}")
    print("📋 สรุปผลการเทรนทุกโมเดล")
    print(f"{'Model':<30} {'RMSE X':>9} {'R² X':>8} {'RMSE Y':>9} {'R² Y':>8}")
    print(f"{'─'*30} {'─'*9} {'─'*8} {'─'*9} {'─'*8}")
    for r in results:
        print(f"{r['label']:<30} {r['rmse_x']:>9.4f} {r['r2_x']:>8.4f} "
              f"{r['rmse_y']:>9.4f} {r['r2_y']:>8.4f}")

    # Save training_summary.txt
    summary_path = os.path.join(MODELS_DIR, 'training_summary.txt')
    with open(summary_path, 'w', encoding='utf-8') as f:
        f.write("Multi-Model Compensation Training Summary\n")
        f.write("=" * 60 + "\n\n")
        f.write(f"{'Model':<30} {'RMSE X':>9} {'R² X':>8} {'RMSE Y':>9} {'R² Y':>8}\n")
        f.write(f"{'─'*30} {'─'*9} {'─'*8} {'─'*9} {'─'*8}\n")
        for r in results:
            f.write(f"{r['label']:<30} {r['rmse_x']:>9.4f} {r['r2_x']:>8.4f} "
                    f"{r['rmse_y']:>9.4f} {r['r2_y']:>8.4f}\n")
    print(f"\n📋 บันทึกสรุป → {summary_path}")
    print(f"\n✅ เสร็จสิ้น! บันทึกโมเดล {len(results)} แบบ ที่ {MODELS_DIR}")


if __name__ == "__main__":
    main()