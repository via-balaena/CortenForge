# /// script
# requires-python = ">=3.11"
# dependencies = ["matplotlib"]
# ///
"""Plot compressive-block results: per-refinement F_R FEM scatter
overlaid on the two pure-BC analytic bound lines (uniaxial-stress lower,
uniaxial-strain upper). The shaded band between the bounds is the
physically valid range for the mixed BC (bottom full-pin + top
penalty-contact + sides free); FEM scatter falling inside the band IS
the asserted two-bound bracket.

Per the compressive-block fixture's "Deviation 1" derivation: adding lateral constraints
can only stiffen the response, removing them can only soften it — so
F_R for the mixed BC must satisfy
    F_us(ε) = E · A · ε  ≤  F_R(ε)  ≤  F_strain(ε) = M_c · A · ε
at the equilibrium ε. The cube aspect ratio places F_R close to the
F_us lower bound (Saint-Venant boundary-layer regime — bottom-pin's
lateral constraint is geometrically confined to a thin layer, leaving
most of the cube interior in uniaxial-stress regime).

Title shows the asserted Cauchy ratio + n=8 effective modulus +
rel_pos_in_bounds.

Usage:
    uv run examples/sim-soft/compressive-block/plot.py
"""

import json
from pathlib import Path

import matplotlib.pyplot as plt

ROOT = Path(__file__).parent
JSON_PATH = ROOT / "out" / "compressive_block.json"
PNG_PATH = ROOT / "out" / "compressive_block.png"

with JSON_PATH.open() as f:
    data = json.load(f)

scalars = data["scalars"]
analytic = data["analytic"]

# Convert to physical-friendly units (% strain, mN force).
a_eps_pct = [a["eps"] * 100.0 for a in analytic]
a_f_us_mn = [a["f_us"] * 1000.0 for a in analytic]
a_f_strain_mn = [a["f_strain"] * 1000.0 for a in analytic]

# Per-refinement FEM scatter points.
fem_points = [
    ("n=2", scalars["eps_n2"] * 100.0, scalars["f_r_fem_n2"] * 1000.0),
    ("n=4", scalars["eps_n4"] * 100.0, scalars["f_r_fem_n4"] * 1000.0),
    ("n=8", scalars["eps_n8"] * 100.0, scalars["f_r_fem_n8"] * 1000.0),
]

# Asserted scalars for title.
e_young = scalars["e_young"]
m_constrained = scalars["m_constrained"]
effective_modulus_n8 = scalars["effective_modulus_n8"]
rel_pos = scalars["rel_pos_in_bounds_n8"]
cauchy_ratio = scalars["cauchy_ratio"]

fig, ax = plt.subplots(figsize=(11.0, 6.0))

# Two analytic bound lines.
ax.plot(
    a_eps_pct,
    a_f_us_mn,
    color="tab:red",
    lw=2.0,
    label=f"$F_{{us}} = E \\cdot A \\cdot \\varepsilon$  (E = {e_young:.2e} Pa, lower)",
)
ax.plot(
    a_eps_pct,
    a_f_strain_mn,
    color="tab:blue",
    lw=2.0,
    label=f"$F_{{strain}} = M_c \\cdot A \\cdot \\varepsilon$  (M_c = {m_constrained:.2e} Pa, upper)",
)
# Shaded valid region between bounds.
ax.fill_between(
    a_eps_pct,
    a_f_us_mn,
    a_f_strain_mn,
    color="tab:gray",
    alpha=0.15,
    label="Mixed-BC valid region (F_us ≤ F_R ≤ F_strain)",
)

# FEM scatter at each refinement level.
for label, eps_pct, f_r_mn in fem_points:
    ax.scatter(
        eps_pct,
        f_r_mn,
        s=80,
        color="coral",
        edgecolor="black",
        zorder=5,
        label=f"FEM {label}: ε = {eps_pct:.4f} %, $F_R$ = {f_r_mn:.2f} mN",
    )

ax.set_xlabel("Compressive strain $\\varepsilon = 1 - \\lambda_z$ (%)")
ax.set_ylabel("Reaction force $F_R$ (mN)")
ax.set_xlim(left=0.0)
ax.set_ylim(bottom=0.0)
ax.legend(loc="upper left", fontsize=9)
ax.grid(True, alpha=0.3)

plt.title(
    "compressive-block (compressive block): FEM in two-bound bracket — "
    f"E_eff(n=8) = {effective_modulus_n8:.3e} Pa, "
    f"rel_pos = {rel_pos:.3f}, Cauchy = {cauchy_ratio:.3f}"
)
plt.tight_layout()
plt.savefig(PNG_PATH, dpi=150)
print(f"saved: {PNG_PATH}")
