"""
pointcloud_viewer.py
--------------------
Interactive 3D point cloud of TCP poses coloured by KDE density.
All three axes share the same scale (equal aspect ratio).
Use the sliders to rotate the view, then click "Spremi PNG" to export.

Usage:
    python pointcloud_viewer.py                          # uses tool_poses.csv next to script
    python pointcloud_viewer.py path/to/tool_poses.csv   # explicit path

Dependencies:
    pip install pandas numpy matplotlib scipy
"""

import sys
from pathlib import Path

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.colors import Normalize
from matplotlib.widgets import Slider, Button
from scipy.stats import gaussian_kde

# ---------------------------------------------------------------------------
# Load data
# ---------------------------------------------------------------------------
csv_path = Path(sys.argv[1]) if len(sys.argv) > 1 else Path(__file__).parent / "tool_poses.csv"
df = pd.read_csv(csv_path)
x, y, z = df["x"].values, df["y"].values, df["z"].values

# Subsample for KDE (full 19k is slow)
rng = np.random.default_rng(42)
idx = rng.choice(len(x), size=3000, replace=False)
xs, ys, zs = x[idx], y[idx], z[idx]

# KDE density
print("Computing KDE density …")
density = gaussian_kde(np.vstack([xs, ys, zs]))(np.vstack([xs, ys, zs]))
norm = Normalize(vmin=density.min(), vmax=density.max())

# Zoom limits (2–98th percentile per axis)
x2, x98 = float(np.percentile(xs, 2)),  float(np.percentile(xs, 98))
y2, y98 = float(np.percentile(ys, 2)),  float(np.percentile(ys, 98))
z2, z98 = float(np.percentile(zs, 2)),  float(np.percentile(zs, 98))

# Equal aspect: expand each axis to the same half-range around its centre
half = max(x98 - x2, y98 - y2, z98 - z2) / 2.0
cx, cy, cz = (x2 + x98) / 2, (y2 + y98) / 2, (z2 + z98) / 2
xlim = (cx - half, cx + half)
ylim = (cy - half, cy + half)
zlim = (cz - half, cz + half)

print(f"Equal axis half-range: {half:.4f} m")
print(f"xlim={xlim}\nylim={ylim}\nzlim={zlim}")

# ---------------------------------------------------------------------------
# Figure layout
# ---------------------------------------------------------------------------
fig = plt.figure(figsize=(11, 8))
fig.subplots_adjust(left=0.05, right=0.88, bottom=0.25, top=0.95)

ax = fig.add_subplot(111, projection="3d")

def draw(elev=25, azim=45):
    ax.cla()
    sc = ax.scatter(xs, ys, zs, c=density, cmap="plasma", norm=norm,
                    s=4, alpha=0.6, linewidths=0)
    ax.set_xlim(*xlim)
    ax.set_ylim(*ylim)
    ax.set_zlim(*zlim)
    ax.set_xlabel("x (m)", fontsize=9, labelpad=6)
    ax.set_ylabel("y (m)", fontsize=9, labelpad=6)
    ax.set_zlabel("z (m)", fontsize=9, labelpad=6)
    ax.set_title("Raspodjela poza TCP-a u koordinatnom sustavu kalupa", fontsize=11, pad=12)
    ax.xaxis.pane.fill = False
    ax.yaxis.pane.fill = False
    ax.zaxis.pane.fill = False
    ax.grid(True, linestyle="--", alpha=0.4)
    # Force equal box aspect so tick spacing looks uniform
    ax.set_box_aspect([1, 1, 1])
    ax.view_init(elev=elev, azim=azim)
    return sc

sc = draw()

# Colorbar
cbar_ax = fig.add_axes([0.90, 0.30, 0.02, 0.55])
plt.colorbar(sc, cax=cbar_ax).set_label("Gustoća točaka", fontsize=10)

# ---------------------------------------------------------------------------
# Sliders
# ---------------------------------------------------------------------------
ax_elev = fig.add_axes([0.10, 0.13, 0.75, 0.03])
ax_azim = fig.add_axes([0.10, 0.07, 0.75, 0.03])

slider_elev = Slider(ax_elev, "Elevacija",  -90,  90, valinit=25,  valstep=1)
slider_azim = Slider(ax_azim, "Azimut",    -180, 180, valinit=45,  valstep=1)

def update(_):
    draw(elev=slider_elev.val, azim=slider_azim.val)
    fig.canvas.draw_idle()

slider_elev.on_changed(update)
slider_azim.on_changed(update)

# ---------------------------------------------------------------------------
# Save button
# ---------------------------------------------------------------------------
ax_btn = fig.add_axes([0.40, 0.01, 0.18, 0.04])
btn = Button(ax_btn, "Spremi PNG", hovercolor="lightgreen")

def save(_):
    out = Path(__file__).parent / "tool_poses_pointcloud.png"
    for widget_ax in [ax_elev, ax_azim, ax_btn]:
        widget_ax.set_visible(False)
    fig.savefig(out, dpi=180, bbox_inches="tight")
    for widget_ax in [ax_elev, ax_azim, ax_btn]:
        widget_ax.set_visible(True)
    fig.canvas.draw_idle()
    print(f"Saved → {out}")

btn.on_clicked(save)

plt.show()