import matplotlib.pyplot as plt
import pandas as pd

# -----------------------
# Data (ตัวอย่างจากตารางที่ทำไว้)
# -----------------------
data = [
    {"Robot":"BigDog","Mass_kg":110.0,"T_W_proxy":3.636,"Group":"Hydraulic"},
    {"Robot":"HyQ","Mass_kg":85.0,"T_W_proxy":1.412,"Group":"Hydraulic"},
    {"Robot":"MiniHyQ","Mass_kg":30.0,"T_W_proxy":2.000,"Group":"Hydraulic"},
    {"Robot":"HyQ2Max","Mass_kg":80.0,"T_W_proxy":3.125,"Group":"Hydraulic"},
    {"Robot":"HyQReal","Mass_kg":130.0,"T_W_proxy":1.538,"Group":"Hydraulic"},
    {"Robot":"Minitaur","Mass_kg":6.0,"T_W_proxy":0.283,"Group":"Research"},
    {"Robot":"MIT Mini Cheetah","Mass_kg":9.0,"T_W_proxy":2.222,"Group":"Research"},
    {"Robot":"Stanford Doggo","Mass_kg":12.0,"T_W_proxy":0.667,"Group":"Research"},
    {"Robot":"Solo8","Mass_kg":2.5,"T_W_proxy":1.080,"Group":"Research"},
    {"Robot":"ANYmal","Mass_kg":30.0,"T_W_proxy":1.333,"Group":"Commercial"},
    {"Robot":"Spot","Mass_kg":33.8,"T_W_proxy":1.871,"Group":"Commercial"},
    {"Robot":"Unitree Go2","Mass_kg":15.0,"T_W_proxy":3.000,"Group":"Commercial"},
]

df = pd.DataFrame(data)

# -----------------------
# Assign colors by group
# -----------------------
colors = {
    "Hydraulic": "steelblue",
    "Research": "seagreen",
    "Commercial": "darkorange"
}

# -----------------------
# Plot scatter chart
# -----------------------
plt.figure(figsize=(8,6))

for group, subset in df.groupby("Group"):
    plt.scatter(subset["Mass_kg"], subset["T_W_proxy"],
                s=100, c=colors[str(group)], label=str(group), alpha=0.8)

    # Label each point
    for _, row in subset.iterrows():
        plt.text(row["Mass_kg"]+1, row["T_W_proxy"], row["Robot"], fontsize=7, va="center")

plt.xlabel("Weight (kg)")
plt.ylabel("T/W (proxy) [Nm/kg]")
plt.title("Torque-to-Weight Ratio (Proxy) vs Weight")
plt.legend(title="Group")
plt.grid(True, linestyle="--", alpha=0.6)
plt.tight_layout()
plt.show()
