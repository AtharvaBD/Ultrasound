import collections
import serial
import matplotlib.pyplot as plt

PORT, BAUD, WINDOW = "COM8", 115200, 400   # WINDOW = samples shown

ser = serial.Serial(PORT, BAUD, timeout=0.05)   # opening the port resets the ESP32

print("Waiting for CSV header...")
cols = None
while cols is None:
    line = ser.readline().decode(errors="ignore").strip()
    if line.startswith("t_ms,"):
        cols = line.split(",")
print("Columns:", cols)

groups = [("Accel (g)", ["ax_g", "ay_g", "az_g"]),
          ("Gyro (deg/s)", ["gx_dps", "gy_dps", "gz_dps"])]
if "mx_uT" in cols:
    groups.append(("Mag (uT)", ["mx_uT", "my_uT", "mz_uT"]))

t = collections.deque(maxlen=WINDOW)
data = {c: collections.deque(maxlen=WINDOW) for _, g in groups for c in g}

fig, axes = plt.subplots(len(groups), 1, sharex=True, figsize=(10, 3 * len(groups)))
if len(groups) == 1:
    axes = [axes]
lines = {}
for ax, (title, names) in zip(axes, groups):
    ax.set_title(title, loc="left")
    ax.grid(True, alpha=0.3)
    for n in names:
        (lines[n],) = ax.plot([], [], label=n)
    ax.legend(loc="upper right", ncol=3)
axes[-1].set_xlabel("time (s)")
plt.tight_layout()
plt.ion()
plt.show()

while plt.fignum_exists(fig.number):
    got = False
    while ser.in_waiting:
        raw = ser.readline().decode(errors="ignore").strip()
        parts = raw.split(",")
        if len(parts) != len(cols) or parts[0] == "t_ms":
            continue                       # skips log lines and repeated headers
        row = dict(zip(cols, parts))
        try:
            t.append(float(row["t_ms"]) / 1000.0)
            for c in data:
                data[c].append(float(row[c]))
            fig.suptitle(f"steps={row['steps']}  tap={row['tap']}  orient={row['orient']}")
            got = True
        except ValueError:
            continue
    if got:
        for n, ln in lines.items():
            ln.set_data(t, data[n])
        for ax in axes:
            ax.relim()
            ax.autoscale_view()
    plt.pause(0.03)

ser.close()