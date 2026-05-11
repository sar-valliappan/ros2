import csv
import time
import threading
from collections import deque
from xarm.wrapper import XArmAPI

# ── Config ────────────────────────────────────────────────────────────────────
INPUT_DELAY_S = 0.05
LOG_FILE      = "xarm_log_0.05.csv"

KP    = [4.0, 6.0, 4.0, 3.0, 2.5, 2.0]
V_MAX = [30.0, 25.0, 30.0, 30.0, 30.0, 30.0]

ANGLE_TOL    = 1.0
SETTLE_COUNT = 6
TIMEOUT      = 15.0
DT           = 0.02

# ── Global experiment clock ───────────────────────────────────────────────────
GLOBAL_START_TIME = time.time()

# ── ARM init ──────────────────────────────────────────────────────────────────
arm = XArmAPI('192.168.1.209')

time.sleep(0.5)

arm.set_mode(0)
arm.set_state(0)
arm.reset(wait=True)

stop_monitor = threading.Event()

# ── CSV logger ────────────────────────────────────────────────────────────────
_csv_lock   = threading.Lock()
_csv_fh     = None
_csv_writer = None

CSV_HEADER = [
    "global_time",
    "move_time",

    # End effector pose
    "ee_x", "ee_y", "ee_z",
    "ee_roll", "ee_pitch", "ee_yaw",

    # Joint angles
    "q1", "q2", "q3", "q4", "q5", "q6",

    # Joint velocities
    "dq1", "dq2", "dq3", "dq4", "dq5", "dq6",

    # Joint torques
    "tau1", "tau2", "tau3", "tau4", "tau5", "tau6",
]


def _open_log():
    global _csv_fh, _csv_writer

    _csv_fh = open(LOG_FILE, "w", newline="")
    _csv_writer = csv.writer(_csv_fh)

    _csv_writer.writerow(CSV_HEADER)
    _csv_fh.flush()


def _close_log():
    global _csv_fh

    if _csv_fh:
        _csv_fh.flush()
        _csv_fh.close()


def _fmt(v):
    return f"{v:.4f}" if v is not None else ""


def log_row(global_t, move_t, ee, angles, vels, torques):

    row = [
        f"{global_t:.5f}",
        f"{move_t:.5f}"
    ]

    row += [_fmt(v) for v in (ee or [None] * 6)]
    row += [_fmt(v) for v in (angles or [None] * 6)]
    row += [_fmt(v) for v in (vels or [None] * 6)]
    row += [_fmt(v) for v in (torques or [None] * 6)]

    with _csv_lock:
        _csv_writer.writerow(row)
        _csv_fh.flush()


# ── Background monitor ────────────────────────────────────────────────────────
def monitor(duration=120, interval=0.5):

    print(
        f"{'Time':>6} | "
        f"{'J1':>7} {'J2':>7} {'J3':>7} "
        f"{'J4':>7} {'J5':>7} {'J6':>7}"
    )

    print("-" * 60)

    start = time.time()

    while time.time() - start < duration and not stop_monitor.is_set():

        t = time.time() - start

        code_a, angles = arm.get_servo_angle()

        if code_a == 0:
            print(
                f"[Angles] {t:5.1f}s | "
                + " ".join(f"{a:7.2f}" for a in angles)
            )

        time.sleep(interval)


monitor_thread = threading.Thread(
    target=monitor,
    kwargs={"duration": 120, "interval": 0.5},
    daemon=True
)

# ── Mode helpers ──────────────────────────────────────────────────────────────
def _enter_velocity_mode():
    arm.set_mode(4)
    arm.set_state(0)
    time.sleep(0.1)


def _enter_position_mode():
    arm.set_mode(0)
    arm.set_state(0)
    time.sleep(0.1)


# ── Delayed velocity controller ───────────────────────────────────────────────
def move_by_velocity(target_angles, timeout=TIMEOUT, delay_s=INPUT_DELAY_S):

    cmd_buffer   = deque()
    current_vels = [0.0] * 6

    _enter_velocity_mode()

    settle = 0

    t_start   = time.time()
    next_tick = t_start

    while True:

        # ── Fixed-rate timing ───────────────────────────────────────────
        next_tick += DT

        sleep_time = next_tick - time.time()

        if sleep_time > 0:
            time.sleep(sleep_time)

        t_now = time.time()

        move_t   = t_now - t_start
        global_t = t_now - GLOBAL_START_TIME

        if move_t > timeout:
            print("[Velocity] WARNING: move timed out.")
            break

        # ── Read robot state ────────────────────────────────────────────
        code_a, angles = arm.get_servo_angle()
        angles = angles[:6] if code_a == 0 else angles

        if code_a != 0:
            continue

        code_s, states = arm.get_joint_states()
        act_vels = states[1][:6] if code_s == 0 else None

        code_t, torques_r = arm.get_joints_torque()
        torques = list(torques_r)[:6] if code_t == 0 else None

        code_e, ee_pose = arm.get_position()
        ee = list(ee_pose) if code_e == 0 else None

        # ── Compute controller output ───────────────────────────────────
        vel_cmds  = []
        all_close = True

        for j in range(6):

            err = target_angles[j] - angles[j]

            v_cmd = KP[j] * err

            v_cmd = max(
                -V_MAX[j],
                min(V_MAX[j], v_cmd)
            )

            if abs(err) > ANGLE_TOL:
                all_close = False
            else:
                v_cmd = 0.0

            vel_cmds.append(v_cmd)

        # ── Delay buffer ────────────────────────────────────────────────
        cmd_buffer.append((t_now + delay_s, vel_cmds))

        while cmd_buffer and t_now >= cmd_buffer[0][0]:
            _, current_vels = cmd_buffer.popleft()

        # ── Send velocity command ───────────────────────────────────────
        arm.vc_set_joint_velocity(current_vels)

        # ── Log CURRENT robot state only ────────────────────────────────
        log_row(
            global_t=global_t,
            move_t=move_t,
            ee=ee,
            angles=list(angles),
            vels=list(act_vels) if act_vels is not None else None,
            torques=torques,
        )

        # ── Settling logic ──────────────────────────────────────────────
        if all_close:

            settle += 1

            if settle >= SETTLE_COUNT:
                break

        else:
            settle = 0

    # ── Drain delayed commands ──────────────────────────────────────────
    while cmd_buffer:

        send_at, vels = cmd_buffer.popleft()

        wait = send_at - time.time()

        if wait > 0:
            time.sleep(wait)

        arm.vc_set_joint_velocity(vels)

    # ── Stop robot ──────────────────────────────────────────────────────
    arm.vc_set_joint_velocity([0.0] * 6)

    _enter_position_mode()


# ── Adaptive gripper ──────────────────────────────────────────────────────────
def adaptive_grip(open_pos=350, close_speed=500):

    arm.set_gripper_position(open_pos, wait=True)

    time.sleep(0.2)

    arm.set_gripper_position(
        0,
        speed=close_speed,
        wait=False
    )

    prev_pos      = open_pos
    stall_count   = 0
    gripped_width = 0

    while True:

        time.sleep(0.05)

        code, pos = arm.get_gripper_position()

        if code != 0:
            break

        if abs(prev_pos - pos) < 2:
            stall_count += 1
        else:
            stall_count = 0
            prev_pos = pos

        if stall_count >= 5:

            gripped_width = pos

            print(
                f"[Grip] Contact at width={gripped_width:.1f}"
            )

            break

        if pos <= 5:

            print(
                "[Grip] Warning: fully closed, nothing detected."
            )

            break

    return gripped_width


# ── Main sequence ─────────────────────────────────────────────────────────────
_open_log()

monitor_thread.start()

# arm.set_gripper_position(500, wait=True)

move_by_velocity([   0, -30,  0, 0, 0, 0])
move_by_velocity([-120, -30,  0, 0, 0, 0])
move_by_velocity([-120, -18, -4, 0, 0, 0])

# gripped_width = adaptive_grip(
#     open_pos=500,
#     close_speed=500
# )

# if gripped_width == 0:
#
#     print("[Abort] Nothing gripped. Returning home.")
#
#     move_by_velocity([-120, -30, -4, 0, 0, 0])
#
#     arm.move_gohome(wait=True)
#
#     stop_monitor.set()
#     monitor_thread.join()
#
#     _close_log()
#
#     exit()

# arm.set_tcp_load(0.3, [0, 0, 30])
# arm.set_state(0)

move_by_velocity([-120, -30, -4, 0, 0, 0])
move_by_velocity([-160, -30, -4, 0, 0, 0])
move_by_velocity([-160, -18, -4, 0, 0, 0])

# arm.set_gripper_position(850, wait=True)

# arm.set_tcp_load(0, [0, 0, 30])
# arm.set_state(0)

move_by_velocity([-160, -40, -4, 0, 0, 0])
move_by_velocity([ -60, -20, -4, 0, 0, 0])
move_by_velocity([   0,   0,  0, 0, 0, 0])

# ── Cleanup ───────────────────────────────────────────────────────────────────
stop_monitor.set()

monitor_thread.join()

_close_log()

print(f"[Done] Telemetry saved to {LOG_FILE}")