# /// script
# requires-python = ">=3.11"
# dependencies = ["matplotlib"]
# ///
"""Plot row 18 contact-force-readout per-active-pair contact patch:
top-down 2D scatter of the 81 active top-face vertices in (x, y),
with marker color encoding per-pair pressure (Pa, viridis colormap)
and marker size encoding per-pair force magnitude (|force_z|, N).

The mixed BC (bottom full-pin + top penalty-contact + sides free)
produces non-uniform pressure across the 9×9 top-face grid: corner
vertices penetrate further than interior vertices under the
Saint-Venant boundary-layer pattern, so corner pressure is highest.
The pattern reads as a "cross" of higher-pressure corner+edge
vertices framing a lower-pressure interior — a visual readout of
how the bottom-pin's lateral constraint propagates through the cube.

Title shows F_R_total + n_active_pairs + asserted ε + effective
modulus.

Usage:
    uv run examples/sim-soft/contact-force-readout/plot.py
"""

import json
from pathlib import Path

import matplotlib.pyplot as plt
import matplotlib.cm as cm
from matplotlib.colors import Normalize

ROOT = Path(__file__).parent
JSON_PATH = ROOT / "out" / "contact_force_readout.json"
PNG_PATH = ROOT / "out" / "contact_force_readout.png"

with JSON_PATH.open() as f:
    data = json.load(f)

scalars = data["scalars"]
pairs = data["pairs"]

# Per-pair (x, y) in mm; pressure (Pa); |force_z| (mN).
xs_mm = [p["x"] * 1000.0 for p in pairs]
ys_mm = [p["y"] * 1000.0 for p in pairs]
pressures = [p["pressure"] for p in pairs]
force_z_mn = [abs(p["force_z"]) * 1000.0 for p in pairs]

# Asserted scalars for title.
f_r_total = scalars["f_r_total"]
n_active = scalars["n_active_pairs"]
eps_pct = scalars["eps"] * 100.0
e_eff = scalars["effective_modulus"]
e_young = scalars["e_young"]
m_constrained = scalars["m_constrained"]
mean_pressure = scalars["mean_pressure"]
max_pressure = scalars["max_pressure"]
min_pressure = scalars["min_pressure"]

fig, ax = plt.subplots(figsize=(9.0, 8.0))

# Marker size scaled to the force_z magnitude range. Map smallest
# |force_z| to ~80 pt², largest to ~400 pt² so the variation is
# visually legible without dominating the colormap signal.
fz_min, fz_max = min(force_z_mn), max(force_z_mn)
size_min, size_max = 80.0, 400.0
if fz_max > fz_min:
    sizes = [
        size_min + (size_max - size_min) * (fz - fz_min) / (fz_max - fz_min)
        for fz in force_z_mn
    ]
else:
    sizes = [size_min] * len(force_z_mn)

norm = Normalize(vmin=min(pressures), vmax=max(pressures))
sc = ax.scatter(
    xs_mm,
    ys_mm,
    c=pressures,
    cmap="viridis",
    norm=norm,
    s=sizes,
    edgecolor="black",
    linewidth=0.5,
)

cbar = fig.colorbar(sc, ax=ax, label="Per-pair pressure (Pa, uniform-area approx)")

ax.set_xlabel("x (mm)")
ax.set_ylabel("y (mm)")
ax.set_aspect("equal")
ax.grid(True, alpha=0.3)
ax.set_xlim(-1.0, 11.0)
ax.set_ylim(-1.0, 11.0)

plt.title(
    "contact-force-readout: 81-pair contact patch (top-down view)\n"
    f"F_R_total = {-f_r_total:.4e} N (rigid reaction)  "
    f"ε = {eps_pct:.4f}%  E_eff = {e_eff:.3e} Pa\n"
    f"pressure: mean = {mean_pressure:.3e} Pa  "
    f"max = {max_pressure:.3e} Pa  min = {min_pressure:.3e} Pa  "
    f"(marker size ∝ |force_z|)",
    fontsize=10,
)
plt.tight_layout()
plt.savefig(PNG_PATH, dpi=150)
print(f"saved: {PNG_PATH}")
