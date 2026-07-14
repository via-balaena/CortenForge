# /// script
# requires-python = ">=3.10"
# dependencies = [
#     "matplotlib>=3.8",
#     "numpy>=1.26",
# ]
# ///
"""Render `out/neo_hookean/force_stretch.json` as a 2x2 panel figure.

Run with `uv run plot.py` from the stress-test directory (PEP 723
inline deps install on first run; subsequent runs reuse the cached
env). The figure saves to `out/neo_hookean/force_stretch.png` and opens
an interactive window if a display is available.

Per inventory Q4 visualization convention, JSON-only rows (4, 5, 6)
get matplotlib post-hoc visualization rather than a viewer pass —
this script is the canonical post-hoc visual aid for row 5's curve
(the `neo_hookean` module of `example-stretch-stress-test`).
"""

from __future__ import annotations

import json
import os
import sys
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np


HERE = Path(__file__).parent
JSON_PATH = HERE / "out" / "neo_hookean" / "force_stretch.json"
PNG_PATH = HERE / "out" / "neo_hookean" / "force_stretch.png"


def main() -> int:
    if not JSON_PATH.exists():
        print(
            f"error: {JSON_PATH} does not exist. "
            f"Run `cargo run -p example-stretch-stress-test --release` first.",
            file=sys.stderr,
        )
        return 1

    with JSON_PATH.open() as f:
        data = json.load(f)

    sweep = data["stretch_sweep"]
    scene = data["scene"]
    bracket = scene["in_domain_bracket_lambda_approx"]
    mu = scene["material_uniform"]["mu_Pa"]
    lam = scene["material_uniform"]["lambda_Pa"]

    lambdas = np.array([r["lambda"] for r in sweep])
    lambda_t = np.array([r["lambda_t"] for r in sweep])
    j_vol = np.array([r["J"] for r in sweep])
    p11_obs = np.array([r["P_11"]["observed"] for r in sweep])
    p11_ana = np.array([r["P_11"]["analytic"] for r in sweep])
    psi_obs = np.array([r["psi"]["observed"] for r in sweep])

    fig, axes = plt.subplots(2, 2, figsize=(12, 9), constrained_layout=True)
    fig.suptitle(
        f"neo_hookean: traction-free F = diag(λ, λ_t, λ_t)\n"
        f"compressible NH (μ = {mu:.2g} Pa, Λ = {lam:.2g} Pa, ν ≈ 0.4); "
        f"in-domain λ ≈ [{bracket[0]:.4f}, {bracket[1]:.4f}]\n"
        f"observed (mat.first_piola / mat.energy) and analytic agree to "
        f"f64 ULP — see asserts in src/neo_hookean.rs",
        fontsize=11,
    )

    # Shared decorations.
    def shade_out_of_domain(ax: plt.Axes) -> None:
        ax.axvspan(0.0, bracket[0], alpha=0.10, color="red", lw=0)
        ax.axvspan(bracket[1], bracket[1] * 1.05, alpha=0.10, color="red", lw=0)
        ax.axvline(1.0, color="gray", lw=0.5, ls="--", alpha=0.6)

    # ── Panel (0,0): P_11(λ) — canonical NH stress-strain. ──
    ax = axes[0, 0]
    ax.axhline(0.0, color="gray", lw=0.5, ls="--", alpha=0.6)
    ax.plot(lambdas, p11_obs, "-o", color="C0", label="observed (mat.first_piola)")
    ax.plot(lambdas, p11_ana, "x", color="C1", ms=8, label="analytic (closed form)")
    shade_out_of_domain(ax)
    ax.set_xlabel("stretch λ")
    ax.set_ylabel("P_11 [Pa]")
    ax.set_title("First Piola P_11 (axial) — canonical NH stress-strain")
    ax.legend(loc="upper left")
    ax.grid(True, alpha=0.3)

    # ── Panel (0,1): ψ(λ) — energy bowl, min at rest. ──
    ax = axes[0, 1]
    ax.plot(lambdas, psi_obs, "-o", color="C2")
    shade_out_of_domain(ax)
    ax.set_xlabel("stretch λ")
    ax.set_ylabel("ψ [J/m³]")
    ax.set_title("Energy density ψ — bowl with minimum at rest (ψ(1)=0 exact)")
    ax.grid(True, alpha=0.3)

    # ── Panel (1,0): λ_t(λ) — transverse response. ──
    ax = axes[1, 0]
    ax.axhline(1.0, color="gray", lw=0.5, ls="--", alpha=0.6)
    ax.plot(lambdas, lambda_t, "-o", color="C3")
    shade_out_of_domain(ax)
    ax.set_xlabel("stretch λ")
    ax.set_ylabel("λ_t (transverse stretch)")
    ax.set_title("Transverse λ_t — Newton solution to traction-free condition")
    ax.grid(True, alpha=0.3)

    # ── Panel (1,1): J(λ) — volumetric ratio, explains the ψ asymmetry. ──
    # |ln J| is far larger compressive (J → 0.53) than tensile (J → 1.11),
    # so the (Λ/2)(ln J)² term in ψ blows up much more on the left. The
    # P_11 panel's compressive divergence and ψ's lopsided bowl both follow
    # directly from this curve.
    ax = axes[1, 1]
    ax.axhline(1.0, color="gray", lw=0.5, ls="--", alpha=0.6)
    ax.plot(lambdas, j_vol, "-o", color="C4")
    shade_out_of_domain(ax)
    ax.set_xlabel("stretch λ")
    ax.set_ylabel("J = λ · λ_t²")
    ax.set_title("Volumetric ratio J — drives Λ(ln J)² asymmetry in ψ")
    ax.grid(True, alpha=0.3)

    PNG_PATH.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(PNG_PATH, dpi=150)
    print(f"saved: {PNG_PATH}")

    if os.environ.get("DISPLAY") or sys.platform == "darwin":
        plt.show()

    return 0


if __name__ == "__main__":
    sys.exit(main())
