#!/usr/bin/env python3
"""
Live IMU visualizer for the ESP32 firmware.

Top-left : 3D position trail (m)
Top-right: orientation sphere (body axes, gravity, recorded normals)
Bottom   : x/y/z position vs time

Keys (click the plot window first):
  v          toggle sphere frame: WORLD (device rotates) <-> BODY (world rotates)
  n m z g a  forwarded to the ESP32 (capture normal, print angle, reset pos, calibrate)

pip install pyserial numpy matplotlib
python viz.py --port COM5          (Windows)
python viz.py --port /dev/ttyUSB0  (Linux)
Close idf.py monitor first: only one program can hold the port.
"""
import argparse
import re
import threading
import time
from collections import deque

import numpy as np
import serial
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

# ---------------------------------------------------------------- shared state
lock = threading.Lock()
TRAIL = 3000
t_buf = deque(maxlen=TRAIL)
p_buf = deque(maxlen=TRAIL)          # (x, y, z)
euler = (0.0, 0.0, 0.0)              # roll, pitch, yaw in degrees
stationary = False
normals = []                         # each is a body-frame unit vector
state = {"frame": "world"}
t0 = time.time()

NORMAL_RE = re.compile(r"Normal captured:\s*([-+0-9.eE]+),\s*([-+0-9.eE]+),\s*([-+0-9.eE]+)")


def reader(ser):
    global euler, stationary
    while True:
        try:
            line = ser.readline().decode(errors="ignore").strip()
        except serial.SerialException:
            print("Serial port lost")
            return
        if not line:
            continue
        m = NORMAL_RE.search(line)
        if m:
            n = np.array([float(v) for v in m.groups()])
            norm = np.linalg.norm(n)
            if norm > 1e-6:
                with lock:
                    normals.append(n / norm)
            continue
        parts = line.split(",")
        if len(parts) == 7:
            try:
                v = [float(p) for p in parts]
            except ValueError:
                continue
            with lock:
                euler = (v[0], v[1], v[2])
                p_buf.append((v[3], v[4], v[5]))
                t_buf.append(time.time() - t0)
                stationary = v[6] > 0.5
        else:
            print(line)  # pass other firmware messages through to the terminal


# ---------------------------------------------------------------- math
def rot_matrix(roll, pitch, yaw):
    """Body -> world rotation, ZYX convention (yaw, then pitch, then roll), degrees in."""
    r, p, y = np.radians([roll, pitch, yaw])
    cr, sr, cp, sp, cy, sy = np.cos(r), np.sin(r), np.cos(p), np.sin(p), np.cos(y), np.sin(y)
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp,     cp * sr,                cp * cr],
    ])


# ---------------------------------------------------------------- figure
for k in list(plt.rcParams):
    if k.startswith("keymap."):
        plt.rcParams[k] = []         # free up n, m, z, g, a, v, ... from matplotlib defaults

fig = plt.figure(figsize=(13, 8))
gs = fig.add_gridspec(2, 2, height_ratios=[2, 1])
ax_pos = fig.add_subplot(gs[0, 0], projection="3d")
ax_sph = fig.add_subplot(gs[0, 1], projection="3d")
ax_ts = fig.add_subplot(gs[1, :])

# position trail
(trail_line,) = ax_pos.plot([], [], [], color="tab:blue", lw=1.2)
(trail_dot,) = ax_pos.plot([], [], [], "o", color="tab:red")
ax_pos.set_xlabel("x (m)")
ax_pos.set_ylabel("y (m)")
ax_pos.set_zlabel("z (m)")
ax_pos.set_title("Position")

# sphere
u, v = np.mgrid[0:2 * np.pi:24j, 0:np.pi:12j]
ax_sph.plot_wireframe(np.cos(u) * np.sin(v), np.sin(u) * np.sin(v), np.cos(v),
                      color="gray", alpha=0.15, lw=0.5)
ax_sph.set_xlim(-1.2, 1.2)
ax_sph.set_ylim(-1.2, 1.2)
ax_sph.set_zlim(-1.2, 1.2)
ax_sph.set_box_aspect((1, 1, 1))
ax_sph.set_xlabel("X")
ax_sph.set_ylabel("Y")
ax_sph.set_zlabel("Z")

axis_lines = [ax_sph.plot([0, 1], [0, 0], [0, 0], color=c, lw=3, label=l)[0]
              for c, l in zip(("red", "green", "blue"), ("X axis", "Y axis", "Z axis"))]
(grav_line,) = ax_sph.plot([0, 0], [0, 0], [0, 1], color="black", lw=2.5, ls="--", label="gravity (up)")
normal_lines = []
ax_sph.legend(loc="upper left", fontsize=8)
title_sph = ax_sph.set_title("")

# time series
ts_lines = [ax_ts.plot([], [], label=l)[0] for l in ("x", "y", "z")]
ax_ts.set_xlabel("time (s)")
ax_ts.set_ylabel("position (m)")
ax_ts.legend(loc="upper left")
ax_ts.grid(alpha=0.3)


def update(_):
    with lock:
        e = euler
        stat = stationary
        P = np.array(p_buf) if p_buf else np.zeros((0, 3))
        T = np.array(t_buf)
        N = list(normals)

    R = rot_matrix(*e)

    # ---- sphere
    if state["frame"] == "world":
        axes = R                          # columns are body axes seen from the world
        grav = np.array([0.0, 0.0, 1.0])
        nvecs = [R @ n for n in N]        # recorded normals ride along with the device
    else:
        axes = np.eye(3)                  # body axes stay put
        grav = R.T @ np.array([0.0, 0.0, 1.0])
        nvecs = N

    for i, ln in enumerate(axis_lines):
        a = axes[:, i]
        ln.set_data_3d([0, a[0]], [0, a[1]], [0, a[2]])
    grav_line.set_data_3d([0, grav[0]], [0, grav[1]], [0, grav[2]])

    while len(normal_lines) < len(nvecs):
        k = len(normal_lines)
        color = plt.cm.tab10(k % 10)
        (ln,) = ax_sph.plot([0, 0], [0, 0], [0, 0], color=color, lw=2.2, label=f"normal {k + 1}")
        normal_lines.append(ln)
        ax_sph.legend(loc="upper left", fontsize=8)
    for ln, n in zip(normal_lines, nvecs):
        ln.set_data_3d([0, n[0]], [0, n[1]], [0, n[2]])

    title_sph.set_text(f"{state['frame'].upper()} frame   "
                       f"R {e[0]:.1f}  P {e[1]:.1f}  Y {e[2]:.1f}   "
                       f"{'STILL' if stat else 'moving'}")

    # ---- position
    if len(P):
        trail_line.set_data_3d(P[:, 0], P[:, 1], P[:, 2])
        trail_dot.set_data_3d([P[-1, 0]], [P[-1, 1]], [P[-1, 2]])
        lim = max(0.1, float(np.abs(P).max()) * 1.1)
        ax_pos.set_xlim(-lim, lim)
        ax_pos.set_ylim(-lim, lim)
        ax_pos.set_zlim(-lim, lim)
        for i, ln in enumerate(ts_lines):
            ln.set_data(T, P[:, i])
        ax_ts.set_xlim(T[0], max(T[-1], T[0] + 1))
        ylim = max(0.05, float(np.abs(P).max()) * 1.1)
        ax_ts.set_ylim(-ylim, ylim)


def on_key(event):
    k = (event.key or "").lower()
    if k == "v":
        state["frame"] = "body" if state["frame"] == "world" else "world"
    elif k in ("n", "m", "z", "g", "a") and ser is not None:
        ser.write(k.encode())


ap = argparse.ArgumentParser()
ap.add_argument("--port", required=True)
ap.add_argument("--baud", type=int, default=115200)
args = ap.parse_args()

ser = serial.Serial(args.port, args.baud, timeout=1)
threading.Thread(target=reader, args=(ser,), daemon=True).start()

fig.canvas.mpl_connect("key_press_event", on_key)
anim = FuncAnimation(fig, update, interval=50, cache_frame_data=False)
plt.tight_layout()
plt.show()