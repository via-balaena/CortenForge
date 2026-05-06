# /// script
# requires-python = ">=3.11"
# dependencies = ["matplotlib"]
# ///
"""Plot Hertz sphere-plane results: per-vertex penalty force_z vs r, overlaid
on the Hertz analytic pressure profile p(r).

Two y-axes — left for Hertz pressure (kPa), right for per-vertex penalty force
(mN). The two carry different units (continuous pressure vs discrete force per
vertex), so the comparison is qualitative-shape: radial decay profile, finite
at r = 0, zero at r = a. Quantitative pressure reconstruction would need
per-vertex Voronoi cell areas (an honest follow-on; FEM produces force per
vertex).

Vertical dashed lines mark `r = a_FEM` (coral) and `r = a_Hertz` (blue) — the
gap IS the asserted `rel_err_a`. Title shows the asserted scalars numerically.

Usage:
    uv run examples/sim-soft/hertz-sphere-plane/plot.py
"""

import json
from pathlib import Path

import matplotlib.pyplot as plt

ROOT = Path(__file__).parent
JSON_PATH = ROOT / "out" / "hertz_sphere_plane.json"
PNG_PATH = ROOT / "out" / "hertz_sphere_plane.png"

with JSON_PATH.open() as f:
    data = json.load(f)

scalars = data["scalars"]
vertices = data["vertices"]
analytic = data["analytic"]

# Convert to physical-friendly units (mm radius, kPa pressure, mN force).
v_r_mm = [v["r"] * 1000.0 for v in vertices]
v_force_mn = [v["force_z"] * 1000.0 for v in vertices]
a_r_mm = [a["r"] * 1000.0 for a in analytic]
a_p_kpa = [a["p_hertz"] / 1000.0 for a in analytic]

a_fem_mm = scalars["a_fem_h4"] * 1000.0
a_hertz_mm = scalars["a_hertz"] * 1000.0
rel_err_pct = scalars["rel_err_a_h4"] * 100.0
rel_err_gate_pct = scalars["rel_err_gate"] * 100.0
cauchy_ratio = scalars["cauchy_ratio_a"]
n_active = scalars["n_active_h4"]

fig, ax_p = plt.subplots(figsize=(9.0, 6.0))

# Left axis — Hertz analytic pressure (kPa).
ax_p.plot(
    a_r_mm,
    a_p_kpa,
    color="tab:blue",
    lw=2.0,
    label=f"Hertz analytic $p(r)$  (a_Hertz = {a_hertz_mm:.3f} mm)",
)
ax_p.set_xlabel("Horizontal radius $r$ (mm)")
ax_p.set_ylabel("Hertz pressure $p(r)$ (kPa)", color="tab:blue")
ax_p.tick_params(axis="y", labelcolor="tab:blue")
ax_p.axvline(a_hertz_mm, color="tab:blue", ls="--", lw=1.0, alpha=0.6)
ax_p.set_xlim(left=0.0)
ax_p.set_ylim(bottom=0.0)

# Right axis — per-vertex penalty force_z (mN).
ax_f = ax_p.twinx()
ax_f.scatter(
    v_r_mm,
    v_force_mn,
    color="coral",
    s=24,
    label=f"FEM penalty force_z @ h/4  (n_active = {n_active})",
)
ax_f.set_ylabel("Per-vertex penalty force_z (mN)", color="coral")
ax_f.tick_params(axis="y", labelcolor="coral")
ax_f.axvline(a_fem_mm, color="coral", ls="--", lw=1.0, alpha=0.8)
ax_f.set_ylim(bottom=0.0)

# Combined legend in the upper-right.
h_p, l_p = ax_p.get_legend_handles_labels()
h_f, l_f = ax_f.get_legend_handles_labels()
ax_p.legend(h_p + h_f, l_p + l_f, loc="upper right")

plt.title(
    "Hertz sphere-plane (V-3, h/4 = 0.75 mm): "
    f"rel_err_a = {rel_err_pct:.2f}% (gate < {rel_err_gate_pct:.0f}%; "
    f"Cauchy = {cauchy_ratio:.3f})"
)
plt.tight_layout()
plt.savefig(PNG_PATH, dpi=150)
print(f"saved: {PNG_PATH}")
