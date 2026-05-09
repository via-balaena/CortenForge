# /// script
# requires-python = ">=3.11"
# dependencies = ["matplotlib"]
# ///
"""Plot the row 22 quasi-static ramp's force-displacement curve.

Reads `out/scan_fit_3layer_sleeve_ramp.json` (specifically the `ramp_curve`
array of per-step records), produces a dual-axis plot of penetration depth
vs force_total_z (left) and depth vs max_disp (right). Post-v2.5 the
force_total_z grows smoothly + convexly from ~1.1 N at 0.5 mm to ~23 N at
6 mm; the deep-penetration signature lives in the Newton iter-count growth
(annotated above each force-curve point) since the body stiffens nonlinearly
approaching the NH validity-domain boundary.

Mirrors row 5 / row 14 / hertz-sphere-plane's `plot.py` PEP 723 structure.

Usage:
    uv run examples/sim-soft/scan-fit-3layer-sleeve-ramp/plot_ramp.py
"""

import json
from pathlib import Path

import matplotlib.pyplot as plt


def main() -> None:
    here = Path(__file__).resolve().parent
    json_path = here / "out" / "scan_fit_3layer_sleeve_ramp.json"
    if not json_path.exists():
        raise SystemExit(
            f"missing {json_path}; run "
            "`cargo run -p example-sim-soft-scan-fit-3layer-sleeve-ramp --release` first"
        )

    with json_path.open() as f:
        document = json.load(f)

    ramp_curve = document["ramp_curve"]
    depths_mm = [r["depth_m"] * 1000.0 for r in ramp_curve]
    force_z_n = [r["force_total_z_n"] for r in ramp_curve]
    max_disp_mm = [r["max_displacement_magnitude_m"] * 1000.0 for r in ramp_curve]
    iters = [r["iter_count"] for r in ramp_curve]

    fig, ax_force = plt.subplots(figsize=(8, 5))
    ax_disp = ax_force.twinx()

    color_force = "#c44e52"  # coral
    color_disp = "#4c72b0"  # blue

    line_force = ax_force.plot(
        depths_mm,
        force_z_n,
        marker="o",
        color=color_force,
        label="force_total_z (N)",
    )
    line_disp = ax_disp.plot(
        depths_mm,
        max_disp_mm,
        marker="s",
        color=color_disp,
        label="max_disp (mm)",
        linestyle="--",
    )

    ax_force.set_xlabel("probe penetration depth (mm)")
    ax_force.set_ylabel("force_total_z (N)", color=color_force)
    ax_disp.set_ylabel("max_disp (mm)", color=color_disp)
    ax_force.tick_params(axis="y", labelcolor=color_force)
    ax_disp.tick_params(axis="y", labelcolor=color_disp)
    ax_force.grid(alpha=0.3)

    # Annotate iter count above each force-curve point — communicates
    # solver effort per step (8/8/9 → 11/11/13 → ... → 22/30/61). 12-pt
    # vertical offset clears the data marker; semi-transparent white
    # bbox keeps the digits readable against the curve where the
    # annotation overlaps it (last 2-3 steps where the curve climbs
    # steeply).
    for d_mm, f_n, it in zip(depths_mm, force_z_n, iters):
        ax_force.annotate(
            f"{it}",
            (d_mm, f_n),
            textcoords="offset points",
            xytext=(0, 12),
            ha="center",
            fontsize=8,
            color=color_force,
            bbox={"boxstyle": "round,pad=0.15", "facecolor": "white", "edgecolor": "none", "alpha": 0.7},
        )

    final = document["scalars"]["final_step"]
    title = (
        f"row 22 quasi-static ramp — {len(ramp_curve)} × "
        f"{ramp_curve[1]['depth_m'] * 1000.0 - ramp_curve[0]['depth_m'] * 1000.0:.1f} mm "
        f"to {final['depth_m'] * 1000.0:.1f} mm\n"
        f"final force_z = {final['force_total_z_n']:+.1f} N, "
        f"max_disp = {final['max_displacement_magnitude_m'] * 1000.0:.2f} mm "
        f"(annotations = Newton iter count per step)"
    )
    ax_force.set_title(title, fontsize=10)

    # Legend in upper-left: the convex force curve and the linear-ish
    # max_disp curve both climb left-to-right, leaving the upper-left
    # corner empty. Upper-right would crowd the rightmost data points
    # (depth 5-6 mm) plus the iter-count annotations there.
    lines = line_force + line_disp
    labels = [line.get_label() for line in lines]
    ax_force.legend(lines, labels, loc="upper left")

    # Add headroom so the "61" annotation at step 12 (top-right corner,
    # ~12 pts above force_z = 23.1 N) doesn't crowd the chart frame.
    ax_force.margins(y=0.08)
    ax_disp.margins(y=0.08)

    fig.tight_layout()
    out_path = here / "out" / "ramp_curve.png"
    fig.savefig(out_path, dpi=150)
    print(f"wrote {out_path}")


if __name__ == "__main__":
    main()
