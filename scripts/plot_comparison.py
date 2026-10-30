"""Plot synthetic comparison CSVs; requires matplotlib. Usage: python plot_comparison.py DIR"""
import csv
import sys
from pathlib import Path
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

folder = Path(sys.argv[1])
fig, axes = plt.subplots(2, 2, figsize=(11, 6.5), sharex=True, constrained_layout=True)
colors = {"pid": "#c26a17", "lqr": "#167b63", "mpc": "#375ac5"}
for col, scenario in enumerate(("curves", "delay")):
    for name, color in colors.items():
        with (folder / f"{name}_{scenario}_25.csv").open() as f:
            rows = list(csv.DictReader(f))
        t = [float(r["time_s"]) for r in rows]
        axes[0, col].plot(t, [float(r["cte_m"]) for r in rows], color=color, label=name.upper(), linewidth=1.6)
        axes[1, col].plot(t, [float(r["steering"]) for r in rows], color=color, linewidth=1.3)
    axes[0, col].set_title("Curves, no actuator delay" if col == 0 else "Same curves, 100 ms actuator delay")
    axes[0, col].legend(ncol=3, loc="upper right", fontsize=8)
    axes[1, col].set_xlabel("Time (s)")
    for row in range(2):
        axes[row, col].axhline(0, color="#777", linewidth=.6)
        axes[row, col].grid(alpha=.2)
axes[0, 0].set_ylabel("Cross-track error (m)")
axes[1, 0].set_ylabel("Commanded steering [-1, 1]")
fig.suptitle("PID vs LQR vs MPC at 25 mph\nSynthetic bicycle plant · starter tuning · not Unity simulator results", fontsize=13)
fig.savefig(folder / "comparison.png", dpi=160)
