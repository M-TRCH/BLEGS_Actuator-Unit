import matplotlib.pyplot as plt

# ข้อมูลหุ่นยนต์ทั้ง 12 ตัว
robots = [
    {"name": "BigDog", "weight": 110, "actuation": "Hydraulic", "line": "Research"},
    {"name": "HyQ", "weight": 80, "actuation": "Hydraulic", "line": "Research"},
    {"name": "MiniHyQ", "weight": 30, "actuation": "Hydraulic", "line": "Research"},
    {"name": "HyQ2Max", "weight": 80, "actuation": "Hydraulic", "line": "Research"},
    {"name": "HyQReal", "weight": 130, "actuation": "Hydraulic", "line": "Research"},
    {"name": "Spot", "weight": 33, "actuation": "Electric+Gear", "line": "Commercial"},
    {"name": "ANYmal", "weight": 32, "actuation": "Electric+SEA", "line": "Commercial"},
    {"name": "Unitree Go2", "weight": 15, "actuation": "Electric+Gear", "line": "Commercial"},
    {"name": "MIT Mini Cheetah", "weight": 9, "actuation": "Quasi-Direct", "line": "Research"},
    {"name": "Stanford Doggo", "weight": 5, "actuation": "Quasi-Direct", "line": "Research"},
    {"name": "Solo8", "weight": 2.2, "actuation": "Direct Drive", "line": "Research"},
    {"name": "Minitaur", "weight": 6, "actuation": "Direct Drive", "line": "Research"},
]

# Mapping ประเภท actuator → ตำแหน่งแกน X
actuation_map = {
    "Hydraulic": 0,
    "Electric+Gear": 1,
    "Electric+SEA": 1.2,
    "Quasi-Direct": 2,
    "Direct Drive": 2.5
}

# สีแยกตามสายงาน
color_map = {"Research": "blue", "Commercial": "green"}

# เริ่ม plot
plt.figure(figsize=(10, 6))

for r in robots:
    x = actuation_map[r["actuation"]]
    y = r["weight"]
    plt.scatter(x, y, color=color_map[r["line"]], s=100, label=r["line"] if r["name"] == "BigDog" else "")
    plt.text(x + 0.05, y, r["name"], fontsize=9, va="center")

# ปรับแกน
plt.xticks(
    [0, 1, 1.2, 2, 2.5],
    ["Hydraulic", "Elec+Gear", "Elec+SEA", "Quasi-Direct", "Direct"]
)
plt.ylabel("Weight (kg)")
plt.title("Hardware Architecture Landscape of Quadruped Robots (2005–2025)", fontsize=14, weight="bold")

# ใส่ legend (กำหนดสีใหม่)
from matplotlib.lines import Line2D
custom_lines = [
    Line2D([0], [0], marker='o', color='w', markerfacecolor='blue', markersize=10, label='Research'),
    Line2D([0], [0], marker='o', color='w', markerfacecolor='green', markersize=10, label='Commercial')
]
plt.legend(handles=custom_lines, loc="upper right")

# แสดงผล
plt.show()
