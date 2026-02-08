import matplotlib.pyplot as plt
import numpy as np

# รายชื่อหุ่นยนต์
robots = [
    "BigDog", "HyQ", "MiniHyQ", "HyQ2Max", "HyQReal",
    "Minitaur", "ANYmal", "MIT Mini Cheetah",
    "Stanford Doggo", "Spot", "Solo8", "Unitree Go2"
]

# ประเภทเซนเซอร์
sensors = [
    "Encoders", "IMU", "Force/Torque", "Hydraulic Pressure",
    "Motor Current", "Vision/Camera", "LiDAR", "External Mocap"
]

# Matrix: 1.0 = ใช้งานแน่นอน, 0.0 = ไม่มีเลย, 0.5 = optional
sensor_matrix = np.array([
    [1, 1, 0.5, 1, 0, 0.5, 0, 0],    # BigDog
    [1, 1, 0.5, 1, 0, 0.5, 0.5, 0],  # HyQ
    [1, 1, 0.5, 1, 0, 0.5, 0, 0.5],  # MiniHyQ
    [1, 1, 1,   1, 0, 0.5, 0.5, 0],  # HyQ2Max
    [1, 1, 1,   1, 0, 0.5, 1,   0],  # HyQReal
    [1, 1, 0,   0, 1, 0.5, 0, 0.5],  # Minitaur
    [1, 1, 1,   0, 1, 1,   1,   0],  # ANYmal
    [1, 1, 0,   0, 1, 0.5, 0, 0.5],  # MIT Mini Cheetah
    [1, 1, 0,   0, 1, 0.5, 0, 0.5],  # Stanford Doggo
    [1, 1, 0.5, 0, 1, 1,   0.5, 0],  # Spot
    [1, 1, 0,   0, 1, 0.5, 0, 0.5],  # Solo8
    [1, 1, 0.5, 0, 1, 1,   1,   0],  # Unitree Go2
])

# พล็อต heatmap
fig, ax = plt.subplots(figsize=(10, 6))
im = ax.imshow(sensor_matrix, cmap="YlGnBu", aspect="auto", vmin=0, vmax=1)

# กำหนด tick
ax.set_xticks(np.arange(len(sensors)))
ax.set_yticks(np.arange(len(robots)))
ax.set_xticklabels(sensors, rotation=45, ha="right")
ax.set_yticklabels(robots)

# แสดงค่าบนตาราง
for i in range(len(robots)):
    for j in range(len(sensors)):
        val = sensor_matrix[i, j]
        if val == 1:
            text = "✓"
        elif val == 0.5:
            text = "opt"
        else:
            text = ""
        ax.text(j, i, text, ha="center", va="center", color="black", fontsize=8)

plt.title("Sensor Usage Across Quadruped Robots (2005–2025)")
plt.colorbar(im, ax=ax, fraction=0.025, pad=0.04, label="Presence (1=yes, 0=no, 0.5=opt)")
plt.tight_layout()
plt.show()
