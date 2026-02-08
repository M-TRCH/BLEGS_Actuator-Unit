import matplotlib.pyplot as plt
import pandas as pd

# -----------------------
# Data (ตัวอย่างจากตารางที่ทำไว้)
# -----------------------
data = [
    {"Robot":"BigDog","T_W_proxy":3.636,"Group":"Hydraulic"},
    {"Robot":"HyQ","T_W_proxy":1.412,"Group":"Hydraulic"},
    {"Robot":"MiniHyQ","T_W_proxy":2.000,"Group":"Hydraulic"},
    {"Robot":"HyQ2Max","T_W_proxy":3.125,"Group":"Hydraulic"},
    {"Robot":"HyQReal","T_W_proxy":1.538,"Group":"Hydraulic"},
    {"Robot":"Minitaur","T_W_proxy":0.283,"Group":"Research"},
    {"Robot":"MIT Mini Cheetah","T_W_proxy":2.222,"Group":"Research"},
    {"Robot":"Stanford Doggo","T_W_proxy":0.667,"Group":"Research"},
    {"Robot":"Solo8","T_W_proxy":1.080,"Group":"Research"},
    {"Robot":"ANYmal","T_W_proxy":1.333,"Group":"Commercial"},
    {"Robot":"Spot","T_W_proxy":1.871,"Group":"Commercial"},
    {"Robot":"Unitree Go2","T_W_proxy":3.000,"Group":"Commercial"},
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

df_sorted = df.sort_values("T_W_proxy", ascending=False)

# -----------------------
# Plot bar chart
# -----------------------
plt.figure(figsize=(10,6))
bars = plt.bar(df_sorted["Robot"], df_sorted["T_W_proxy"],
               color=[colors[g] for g in df_sorted["Group"]])

# Add values on top
for bar, value in zip(bars, df_sorted["T_W_proxy"]):
    plt.text(bar.get_x() + bar.get_width()/2, value + 0.05,
             f"{value:.2f}", ha="center", va="bottom", fontsize=8)

# Labels and title
plt.ylabel("T/W (proxy) [Nm/kg]")
plt.title("Torque-to-Weight Ratio (Proxy) Comparison")
plt.xticks(rotation=45, ha="right")

# Create custom legend
import matplotlib.patches as mpatches
legend_handles = [mpatches.Patch(color=col, label=grp) for grp, col in colors.items()]
plt.legend(handles=legend_handles, title="Group")

plt.tight_layout()
plt.show()
