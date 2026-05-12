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


def compute_waypoints(df):
    """Return waypoint times and joint positions at move_time resets.

    A reset is detected when `move_time` decreases compared to the previous sample.
    Returns (times, positions_dict) where `times` is an array of relative times
    and `positions_dict` maps `q1`..`q6` to arrays of values at those times.
    """
    mt = df["move_time"].to_numpy(dtype=float)
    t = df["plot_time"].to_numpy(dtype=float)
    n = mt.size
    if n == 0:
        return np.array([]), {f"q{i}": np.array([]) for i in range(1, 7)}
    if n == 1:
        idxs = np.array([0], dtype=int)
    else:
        # reset where current move_time is less than previous move_time
        reset_mask = np.concatenate(([False], mt[1:] < mt[:-1]))
        reset_idxs = np.where(reset_mask)[0].tolist()
        # include initial state as waypoint 0
        if 0 not in reset_idxs:
            reset_idxs.insert(0, 0)
        # include last sample as final waypoint (there may be no subsequent reset)
        if (n - 1) not in reset_idxs:
            reset_idxs.append(n - 1)
        idxs = np.array(reset_idxs, dtype=int)

    times = t[idxs]
    positions = {}
    for i in range(1, 7):
        positions[f"q{i}"] = df[f"q{i}"].to_numpy(dtype=float)[idxs]
    return times, positions

colors = ["black", "green", "orange", "red"]
velocity_limits = {1: 30, 2: 25, 3: 30, 4: 30, 5: 30, 6: 30}

delay_suffixes = {
    "0.00s Delay": "0.00",
    "0.05s Delay": "0.05",
    "0.10s Delay": "0.10",
    "0.15s Delay": "0.15",
}

# map each delay label to a color (preserve order from csv_files)
labels = [lbl for lbl, _ in csv_files]
color_map = {
    "0.00s Delay": "blue",
    "0.05s Delay": "green",
    "0.10s Delay": "yellow",
    "0.15s Delay": "red",
}

def is_theoretical_label(label: str) -> bool:
    return label.lower().startswith("theor") or label.lower().startswith("theoretical")

def get_style_for_label(label: str):
    if is_theoretical_label(label):
        return ":"  # dotted for theoretical
    return "-"  # solid for actual trajectories


for joint_num in range(1, 7):
    fig, axs = plt.subplots(3, 1, figsize=(14, 10))

    # Plot 1: Position
    for label, df in data.items():
        t = df["plot_time"].to_numpy(dtype=float)
        q = df[f"q{joint_num}"].to_numpy(dtype=float)
        color = color_map.get(label, "black")
        ls = get_style_for_label(label)
        axs[0].plot(t, q, linewidth=2, label=label, color=color, linestyle=ls)
    axs[0].set_ylabel(f"Position q{joint_num} (deg)", fontsize=12)
    axs[0].set_title(f"Joint {joint_num} Position vs Relative Time (All Delays)", fontsize=14, fontweight="bold")
    axs[0].grid(True, alpha=0.3)
    axs[0].legend(loc="best", fontsize=10)

    # Plot 2: Velocity
    for label, df in data.items():
        t = df["plot_time"].to_numpy(dtype=float)
        dq = df[f"dq{joint_num}"].to_numpy(dtype=float)
        color = color_map.get(label, "black")
        ls = get_style_for_label(label)
        axs[1].plot(t, dq, linewidth=2, label=label, color=color, linestyle=ls)

    limit = velocity_limits[joint_num]
    axs[1].axhline(limit, linestyle="--", color="black", linewidth=1.5, label="Velocity limit")
    axs[1].axhline(-limit, linestyle="--", color="black", linewidth=1.5)

    axs[1].set_ylabel(f"Velocity dq{joint_num} (deg/s)", fontsize=12)
    axs[1].set_title(f"Joint {joint_num} Velocity vs Relative Time (All Delays)", fontsize=14, fontweight="bold")
    axs[1].grid(True, alpha=0.3)
    axs[1].legend(loc="best", fontsize=10)

    # Plot 3: Torque
    for label, df in data.items():
        t = df["plot_time"].to_numpy(dtype=float)
        tau = df[f"tau{joint_num}"].to_numpy(dtype=float)
        color = color_map.get(label, "black")
        ls = get_style_for_label(label)
        axs[2].plot(t, tau, linewidth=2, label=label, color=color, linestyle=ls)

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


for delay_label, df in data.items():
    delay_suffix = delay_suffixes[delay_label]

    for joint_num in range(1, 7):
        fig, axs = plt.subplots(3, 1, figsize=(14, 10))

        t = df["plot_time"].to_numpy(dtype=float)

        # Plot 1: Position
        q = df[f"q{joint_num}"].to_numpy(dtype=float)
        color = color_map.get(delay_label, colors[0])
        ls = get_style_for_label(delay_label)
        axs[0].plot(t, q, linewidth=2, color=color, label=delay_label, linestyle=ls)
        # Overlay waypoint markers and connect them when move_time resets
        wp_t, wp_pos = compute_waypoints(df)
        if wp_t.size > 0:
            q_wp = wp_pos[f"q{joint_num}"]
            axs[0].plot(
                wp_t,
                q_wp,
                linestyle="--",
                marker="o",
                markersize=6,
                color="black",
                linewidth=1.5,
                alpha=0.9,
                label="Expected Trajectory",
            )
        axs[0].set_ylabel(f"Position q{joint_num} (deg)", fontsize=12)
        axs[0].set_title(f"Joint {joint_num} Position vs Relative Time ({delay_label})", fontsize=14, fontweight="bold")
        axs[0].grid(True, alpha=0.3)
        axs[0].legend(loc="best", fontsize=10)

        # Plot 2: Velocity
        dq = df[f"dq{joint_num}"].to_numpy(dtype=float)
        ls = get_style_for_label(delay_label)
        axs[1].plot(t, dq, linewidth=2, color=color, label=delay_label, linestyle=ls)

        limit = velocity_limits[joint_num]
        axs[1].axhline(limit, linestyle="--", color="black", linewidth=1.5, label="Velocity limit")
        axs[1].axhline(-limit, linestyle="--", color="black", linewidth=1.5)

        axs[1].set_ylabel(f"Velocity dq{joint_num} (deg/s)", fontsize=12)
        axs[1].set_title(f"Joint {joint_num} Velocity vs Relative Time ({delay_label})", fontsize=14, fontweight="bold")
        axs[1].grid(True, alpha=0.3)
        axs[1].legend(loc="best", fontsize=10)

        # Plot 3: Torque
        tau = df[f"tau{joint_num}"].to_numpy(dtype=float)
        axs[2].plot(t, tau, linewidth=2, color=color, label=delay_label, linestyle=ls)

        axs[2].set_ylabel(f"Torque tau{joint_num} (Nm)", fontsize=12)
        axs[2].set_title(f"Joint {joint_num} Torque vs Relative Time ({delay_label})", fontsize=14, fontweight="bold")
        axs[2].grid(True, alpha=0.3)
        axs[2].legend(loc="best", fontsize=10)

        axs[2].set_xlabel("Relative Time (seconds)", fontsize=12)

        plt.tight_layout()
        os.makedirs("figures", exist_ok=True)
        filename = f"figures/joint{joint_num}_{delay_suffix}.pdf"
        plt.savefig(filename, dpi=300, bbox_inches="tight")
        print(f"Saved {filename}")
        plt.close(fig)