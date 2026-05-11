import os

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd


csv_files = [
    ("0.00s Delay", "xarm_log_0.00.csv"),
    ("0.05s Delay", "xarm_log_0.05.csv"),
    ("0.10s Delay", "xarm_log_0.10.csv"),
    ("0.15s Delay", "xarm_log_0.15.csv"),
    # ("0.20s Delay", "xarm_log_0.20.csv"),
]

data = {}
for label, fname in csv_files:
    df = pd.read_csv(fname)
    for col in df.columns:
        df[col] = pd.to_numeric(df[col], errors="coerce")
    df = df.dropna()
    df["plot_time"] = df["global_time"] - df["global_time"].min()
    data[label] = df

colors = ["blue", "green", "orange", "red"]
velocity_limits = {1: 30, 2: 25, 3: 30, 4: 30, 5: 30, 6: 30}


for joint_num in range(1, 7):
    fig, axs = plt.subplots(3, 1, figsize=(14, 10))

    # Plot 1: Position
    for (label, df), color in zip(data.items(), colors):
        t = df["plot_time"].to_numpy(dtype=float)
        q = df[f"q{joint_num}"].to_numpy(dtype=float)
        axs[0].plot(t, q, linewidth=2, label=label, color=color)
    axs[0].set_ylabel(f"Position q{joint_num} (deg)", fontsize=12)
    axs[0].set_title(f"Joint {joint_num} Position vs Relative Time (All Delays)", fontsize=14, fontweight="bold")
    axs[0].grid(True, alpha=0.3)
    axs[0].legend(loc="best", fontsize=10)

    # Plot 2: Velocity
    for (label, df), color in zip(data.items(), colors):
        t = df["plot_time"].to_numpy(dtype=float)
        dq = df[f"dq{joint_num}"].to_numpy(dtype=float)
        axs[1].plot(t, dq, linewidth=2, label=label, color=color)

    limit = velocity_limits[joint_num]
    axs[1].axhline(limit, linestyle="--", color="black", linewidth=1.5, label="Velocity limit")
    axs[1].axhline(-limit, linestyle="--", color="black", linewidth=1.5)

    axs[1].set_ylabel(f"Velocity dq{joint_num} (deg/s)", fontsize=12)
    axs[1].set_title(f"Joint {joint_num} Velocity vs Relative Time (All Delays)", fontsize=14, fontweight="bold")
    axs[1].grid(True, alpha=0.3)
    axs[1].legend(loc="best", fontsize=10)

    # Plot 3: Torque
    for (label, df), color in zip(data.items(), colors):
        t = df["plot_time"].to_numpy(dtype=float)
        tau = df[f"tau{joint_num}"].to_numpy(dtype=float)
        axs[2].plot(t, tau, linewidth=2, label=label, color=color)

    axs[2].set_ylabel(f"Torque tau{joint_num} (Nm)", fontsize=12)
    axs[2].set_title(f"Joint {joint_num} Torque vs Relative Time (All Delays)", fontsize=14, fontweight="bold")
    axs[2].grid(True, alpha=0.3)
    axs[2].legend(loc="best", fontsize=10)

    axs[2].set_xlabel("Relative Time (seconds)", fontsize=12)

    plt.tight_layout()
    os.makedirs("figures", exist_ok=True)
    filename = f"figures/joint{joint_num}.pdf"
    plt.savefig(filename, dpi=300, bbox_inches="tight")
    print(f"Saved {filename}")
    plt.close(fig)