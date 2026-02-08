import matplotlib.pyplot as plt

# ✅ ข้อมูลหุ่นยนต์ (Weight ใช้ค่าที่ตรวจสอบแล้ว)
robots = [
    {"name": "BigDog", "weight": 109, "dof": 16, "line": "Research"},
    {"name": "HyQ", "weight": 85, "dof": 12, "line": "Research"},        # ~80–90 kg
    {"name": "MiniHyQ", "weight": 35, "dof": 12, "line": "Research"},    # 35 kg with onboard pump
    {"name": "HyQ2Max", "weight": 80, "dof": 12, "line": "Research"},
    {"name": "HyQReal", "weight": 130, "dof": 12, "line": "Research"},   # ~130–140 kg
    {"name": "Spot", "weight": 33.8, "dof": 12, "line": "Commercial"},
    {"name": "ANYmal", "weight": 30, "dof": 12, "line": "Commercial"},
    {"name": "Unitree Go2", "weight": 15, "dof": 12, "line": "Commercial"},
    {"name": "MIT Mini Cheetah", "weight": 9, "dof": 12, "line": "Research"},
    {"name": "Stanford Doggo", "weight": 5, "dof": 8, "line": "Research"},
    {"name": "Solo8", "weight": 2.0, "dof": 8, "line": "Research"},
    {"name": "Minitaur", "weight": 5.5, "dof": 8, "line": "Research"},
]

# สีแยกตามสายงาน
color_map = {"Research": "blue", "Commercial": "green"}

# Plot scatter
plt.figure(figsize=(10, 6))
for r in robots:
    plt.scatter(r["dof"], r["weight"],
                color=color_map[r["line"]],
                s=100,
                label=r["line"] if r["name"] == "BigDog" else "")
    plt.text(r["dof"] + 0.15, r["weight"], r["name"], fontsize=9, va="center")

# Label & title
plt.xlabel("Degrees of Freedom (DOF)")
plt.ylabel("Weight (kg)")
plt.title("DOF vs Weight of Quadruped Robots (2005–2025)", fontsize=14, weight="bold")

# ใส่ legend (กำหนดสีใหม่)
from matplotlib.lines import Line2D
custom_lines = [
    Line2D([0], [0], marker='o', color='w', markerfacecolor='blue', markersize=10, label='Research'),
    Line2D([0], [0], marker='o', color='w', markerfacecolor='green', markersize=10, label='Commercial')
]
plt.legend(handles=custom_lines, loc="upper right")

# Show graph
plt.grid(True, linestyle="--", alpha=0.6)
plt.show()
