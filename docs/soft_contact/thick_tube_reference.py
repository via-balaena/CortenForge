#!/usr/bin/env python3
# /// script
# requires-python = ">=3.11"
# dependencies = ["numpy>=2,<3", "scipy>=1.14,<2"]
#
# [tool.uv]
# exclude-newer = "2026-09-24T00:00:00Z"
# ///
"""Reference contact pressure for a thick neo-Hookean or Yeoh tube on a rigid, frictionless mandrel.

This is the oracle for the explicit solver's first experiment
(`docs/SOFT_CONTACT_ARCHITECTURE_RECON.md` §15). It solves the tube's radial
equilibrium for the SAME compressible material the solver uses, in f64, so the
kill criterion compares like with like. The exact incompressible answer
(Haughton & Ogden) is only its ν → 0.5 limit.

Material: Psi = mu/2 (I1 - 3) - mu ln J + c2 (I1 - 3)^2 + lam/2 (ln J)^2, the solver's
compressible Yeoh (neo-Hookean at c2 = 0).
Reference radii A < R < B, deformed radius r(R), axial stretch lz.
Principal stretches l_r = r', l_t = r/R, l_z; nominal stresses
    P_i = dPsi/dl_i = mu (l_i - 1/l_i) + 4 c2 (I1 - 3) l_i + lam ln J / l_i.
Radial equilibrium: dP_r/dR + (P_r - P_t)/R = 0.
Inner wall on the mandrel: r(A) = a. Outer wall: free, P_r(B) = 0, or cased,
r(B) = B. Axial: prescribed lz (plane strain is lz = 1), or free ends, where
lz makes the axial force zero. Pressure is Cauchy: p = -P_r(A) / (l_t(A) lz).

Run:  uv run docs/soft_contact/thick_tube_reference.py
It runs its own checks before printing the reference tables: the incompressible
limit (neo-Hookean and Yeoh), the Lamé limits of the free and the cased wall (and
of the Yeoh wall, whose small-strain λ is λ + 8 c2), and tolerance stability.
A failed check raises, so the checks survive `python -O`.

Golden values:  uv run docs/soft_contact/thick_tube_reference.py --golden \
                    sim/L0/soft-explicit/src/fixtures/golden.rs
writes, after the same checks, K2's corners, the confined case and the Yeoh case
(plan §15a, 15d.8, 16h) as Rust constants: c2/mu, p/mu, lz, and the derivatives the
gap-corrected reference linearizes with (plan 15d.1). A regenerated file is a
reviewed change.
"""
import numpy as np
from scipy.integrate import solve_ivp
from scipy.optimize import brentq

class CheckFailed(RuntimeError):
    """A self-check of the oracle failed."""


def _check(ok, what):
    if not ok:
        raise CheckFailed(what)


def P_i(li, lr, lt, lz, mu, lam, c2=0.0):
    """The nominal stress along stretch li, one of the principal stretches lr, lt, lz."""
    J = lr * lt * lz
    i1_minus_3 = lr**2 + lt**2 + lz**2 - 3.0
    return mu * (li - 1.0 / li) + 4.0 * c2 * i1_minus_3 * li + lam * np.log(J) / li

def dP_dl_r(lr, lt, lz, mu, lam, c2=0.0):
    # d P_r / d l_r at fixed l_t, l_z (J = lr lt lz):
    # mu (1 + 1/lr^2) + 4 c2 ((I1 - 3) + 2 lr^2) + lam (1 - ln J) / lr^2
    J = lr * lt * lz
    i1_minus_3 = lr**2 + lt**2 + lz**2 - 3.0
    return mu * (1.0 + 1.0 / lr**2) + 4.0 * c2 * (i1_minus_3 + 2.0 * lr**2) + lam * (1.0 - np.log(J)) / lr**2

def dP_dl_t_of_Pr(lr, lt, lz, mu, lam, c2=0.0):
    # d P_r / d l_t at fixed l_r: 8 c2 lr lt + lam / (lr lt)
    return 8.0 * c2 * lr * lt + lam / (lr * lt)

def shoot(lr_A, A, B, a, lz, mu, lam, rtol, atol, strict=False, c2=0.0):
    """Integrate from R = A with r(A) = a, r'(A) = lr_A; return (sol, P_r(B))."""
    def rhs(R, y):
        r, lr = y
        lt = r / R
        Pr = P_i(lr, lr, lt, lz, mu, lam, c2)
        Pt = P_i(lt, lr, lt, lz, mu, lam, c2)
        dPr_dR = -(Pr - Pt) / R           # equilibrium
        dlt_dR = (lr - lt) / R            # d(r/R)/dR
        dlr_dR = (dPr_dR - dP_dl_t_of_Pr(lr, lt, lz, mu, lam, c2) * dlt_dR) / dP_dl_r(lr, lt, lz, mu, lam, c2)
        return [lr, dlr_dR]
    sol = solve_ivp(rhs, (A, B), [a, lr_A], method="DOP853", rtol=rtol, atol=atol, dense_output=True)
    # Bracket probes may leave the feasible region; only the final solve must reach R = B.
    _check(not strict or sol.status == 0, f"integration stopped early: {sol.message}")
    rB, lrB = sol.y[:, -1]
    return sol, P_i(lrB, lrB, rB / B, lz, mu, lam, c2)

def _bracket(f, lo, hi, shrink, grow, max_steps=200):
    """Widen [lo, hi] until f changes sign across it; raise if it never does."""
    for _ in range(max_steps):
        if f(lo) <= 0:
            break
        lo *= shrink
    for _ in range(max_steps):
        if f(hi) >= 0:
            break
        hi *= grow
    _check(f(lo) <= 0 <= f(hi), f"no bracket in [{lo}, {hi}]")
    return lo, hi


def solve(A, B, a, lz, mu, lam, rtol=1e-12, atol=1e-14, c2=0.0):
    f = lambda lr_A: shoot(lr_A, A, B, a, lz, mu, lam, rtol, atol, c2=c2)[1]
    lo, hi = 0.3, 1.2
    lo, hi = _bracket(f, lo, hi, 0.9, 1.1)
    lr_A = brentq(f, lo, hi, xtol=1e-15, rtol=1e-15, maxiter=500)
    sol, _ = shoot(lr_A, A, B, a, lz, mu, lam, rtol, atol, strict=True, c2=c2)
    lt_A = a / A
    p = -P_i(lr_A, lr_A, lt_A, lz, mu, lam, c2) / (lt_A * lz)
    return p, sol

def axial_force(sol, A, B, lz, mu, lam, n=4001, c2=0.0):
    """Axial force N = 2 pi int_A^B P_z R dR (nominal, per reference area)."""
    R = np.linspace(A, B, n)
    r, lr = sol.sol(R)
    lt = r / R
    Pz = P_i(lz, lr, lt, lz, mu, lam, c2)
    return 2 * np.pi * np.trapezoid(Pz * R, R)

def solve_free_ends(A, B, a, mu, lam, c2=0.0):
    """Generalised plane strain with zero axial force: find lz with N(lz) = 0."""
    g = lambda lz: axial_force(solve(A, B, a, lz, mu, lam, c2=c2)[1], A, B, lz, mu, lam, c2=c2)
    lz = brentq(g, 0.7, 1.05, xtol=1e-13)
    p, sol = solve(A, B, a, lz, mu, lam, c2=c2)
    return p, lz

def haughton_ogden(A, B, a, lz, mu):
    la = a / A
    lb = np.sqrt((1.0 + (la**2 * lz - 1.0) / (B / A) ** 2) / lz)
    return mu * (np.log(la / lb) / lz + (lb**-2 - la**-2) / (2 * lz**2))

def lame_plane_strain(A, B, a, mu, lam):
    ua = a - A
    C2 = ua / (mu * A / ((lam + mu) * B**2) + 1.0 / A)
    return 2 * mu * C2 * (1.0 / A**2 - 1.0 / B**2)

def lame_cased_plane_strain(A, B, a, mu, lam):
    """Small-strain pressure of a plane-strain tube held at r = B (u = C1 r + C2/r, u(B) = 0)."""
    C2 = (a - A) / (1.0 / A - A / B**2)
    return 2 * mu * C2 / A**2 + 2 * (lam + mu) * C2 / B**2


def solve_cased(A, B, a, lz, mu, lam, rtol=1e-12, atol=1e-14):
    """Outer wall held by a rigid case: r(B) = B. Shoot on r'(A) to hit it."""
    f = lambda lr_A: shoot(lr_A, A, B, a, lz, mu, lam, rtol, atol)[0].y[0, -1] - B
    lo, hi = 0.05, 1.2
    lo, hi = _bracket(f, lo, hi, 0.5, 1.1)
    lr_A = brentq(f, lo, hi, xtol=1e-15, rtol=1e-15, maxiter=500)
    sol, _ = shoot(lr_A, A, B, a, lz, mu, lam, rtol, atol, strict=True)
    lt_A = a / A
    return -P_i(lr_A, lr_A, lt_A, lz, mu, lam) / (lt_A * lz), sol

def solve_cased_free_ends(A, B, a, mu, lam):
    g = lambda lz: axial_force(solve_cased(A, B, a, lz, mu, lam)[1], A, B, lz, mu, lam)
    lo, hi = 1.0, 1.02
    while not (np.isfinite(g(hi)) and g(hi) > 0):  # axial force turns tensile once lz is large enough
        hi = 1.0 + 2.0 * (hi - 1.0)
        _check(hi < 1.6, "no bracket for lz")
    _check(g(lo) < 0, "no bracket for lz")
    lz = brentq(g, lo, hi, xtol=1e-13)
    return solve_cased(A, B, a, lz, mu, lam)[0], lz


def _incompressible_stretches(R, A, a, lz):
    """The closed-form kinematics of an incompressible tube on the mandrel: r^2 = a^2 + (R^2 - A^2)/lz."""
    r = np.sqrt(a * a + (R * R - A * A) / lz)
    lt = r / R
    return r, 1.0 / (lt * lz), lt


def _twice_w1(lr, lt, lz, mu, c2):
    """2 dW/dI1 of the incompressible Yeoh W = mu/2 (I1 - 3) + c2 (I1 - 3)^2."""
    return mu + 4.0 * c2 * (lr * lr + lt * lt + lz * lz - 3.0)


def pressure_incompressible(A, B, a, lz, mu, c2=0.0):
    """Contact pressure of an incompressible Yeoh tube on the mandrel (independent of the ODE).

    With the closed-form kinematics, s_tt - s_rr = 2 W1 (lt^2 - lr^2), and radial equilibrium with a free
    outer wall gives p = int_a^r(B) (s_tt - s_rr)/r dr, taken over R with dr = R/(r lz) dR.
    """
    from scipy.integrate import quad

    def integrand(R):
        r, lr, lt = _incompressible_stretches(R, A, a, lz)
        return _twice_w1(lr, lt, lz, mu, c2) * (lt * lt - lr * lr) / r * R / (r * lz)

    return quad(integrand, A, B, epsabs=1e-14, epsrel=1e-13)[0]


def axial_force_incompressible(lz, A, B, a, mu, c2=0.0):
    """Total axial force on an incompressible Yeoh tube on the mandrel (independent of the ODE).

    Kinematics are closed form, r^2 = a^2 + (R^2 - A^2)/lz. From d(r^2 s_rr)/dr = r (s_rr + s_tt),
    F = pi int_A^B 2 W1 (2 lz^2 - lr^2 - lt^2) R/lz dR + pi a^2 P, with lt = r/R and lr = 1/(lt lz).
    Free ends mean F = 0; in Haughton & Ogden's reduced force N, that is N + pi a^2 P = 0.
    """
    from scipy.integrate import quad

    def integrand(R):
        _, lr, lt = _incompressible_stretches(R, A, a, lz)
        return _twice_w1(lr, lt, lz, mu, c2) * (2 * lz * lz - lr * lr - lt * lt) * R / lz

    N = np.pi * quad(integrand, A, B, epsabs=1e-14, epsrel=1e-13)[0]
    return N + np.pi * a * a * pressure_incompressible(A, B, a, lz, mu, c2)


def _checks():
    mu, A = 1.0, 1.0
    nu_inc = 0.49999
    lam_inc = 2 * nu_inc / (1 - 2 * nu_inc)
    # 1. Haughton-Ogden's hand-checked value, and the incompressible limit, at two axial stretches.
    _check(abs(haughton_ogden(A, 2.0, 1.3, 1.0, mu) - 0.31338) < 1e-5, "Haughton-Ogden check value")
    for lz in (1.0, 0.9):
        p = solve(A, 2.0, 1.3, lz, mu, lam_inc)[0]
        _check(abs(p / haughton_ogden(A, 2.0, 1.3, lz, mu) - 1) < 1e-4, f"incompressible limit at lz={lz}: {p}")
    p, lz = solve_free_ends(A, 2.0, 1.3, mu, lam_inc)
    _check(abs(p / haughton_ogden(A, 2.0, 1.3, lz, mu) - 1) < 1e-4, f"incompressible limit, free ends: {p}")
    # ... and the free-ends axial stretch itself, against the independent incompressible axial balance.
    lz_inc = brentq(lambda z: axial_force_incompressible(z, A, 2.0, 1.3, mu), 0.7, 1.05, xtol=1e-14)
    _check(abs(lz - lz_inc) < 1e-5, f"free-ends lz {lz} vs {lz_inc}")
    # ... and the Yeoh term the same way. The quadrature first reproduces Haughton-Ogden at c2 = 0.
    _check(abs(pressure_incompressible(A, 2.0, 1.3, 0.9, mu) / haughton_ogden(A, 2.0, 1.3, 0.9, mu) - 1) < 1e-10,
           "incompressible quadrature against Haughton-Ogden")
    c2 = YEOH_C2_OVER_MU
    for lz in (1.0, 0.9):
        p = solve(A, 2.0, 1.3, lz, mu, lam_inc, c2=c2)[0]
        _check(abs(p / pressure_incompressible(A, 2.0, 1.3, lz, mu, c2) - 1) < 1e-4, f"Yeoh incompressible limit at lz={lz}: {p}")
    p, lz = solve_free_ends(A, 2.0, 1.3, mu, lam_inc, c2=c2)
    lz_inc = brentq(lambda z: axial_force_incompressible(z, A, 2.0, 1.3, mu, c2), 0.7, 1.05, xtol=1e-14)
    _check(abs(lz - lz_inc) < 1e-5, f"Yeoh free-ends lz {lz} vs {lz_inc}")
    _check(abs(p / pressure_incompressible(A, 2.0, 1.3, lz, mu, c2) - 1) < 1e-4, f"Yeoh incompressible limit, free ends: {p}")
    # 2. The Lamé limit: the relative gap shrinks in proportion to the interference.
    lam = 2 * 0.45 / 0.1
    gaps = [solve(A, 2.0, 1 + e, 1.0, mu, lam)[0] / lame_plane_strain(A, 2.0, 1 + e, mu, lam) - 1 for e in (1e-2, 1e-3)]
    _check(abs(gaps[1]) < abs(gaps[0]) / 5, f"Lame limit, free wall: {gaps}")
    # ... and of the cased wall held in plane strain, the confined case.
    gaps = [solve_cased(A, 2.0, 1 + e, 1.0, mu, lam)[0] / lame_cased_plane_strain(A, 2.0, 1 + e, mu, lam) - 1 for e in (1e-2, 1e-3)]
    _check(abs(gaps[1]) < abs(gaps[0]) / 5, f"Lame limit, cased wall: {gaps}")
    # ... and of the Yeoh wall, whose small-strain lambda is lambda + 8 c2 (c2 (I1 - 3)^2 = 4 c2 (tr eps)^2).
    c2 = 0.5
    gaps = [solve(A, 2.0, 1 + e, 1.0, mu, lam, c2=c2)[0] / lame_plane_strain(A, 2.0, 1 + e, mu, lam + 8 * c2) - 1 for e in (1e-2, 1e-3)]
    _check(abs(gaps[1]) < abs(gaps[0]) / 5, f"Lame limit, Yeoh wall: {gaps}")
    # 3. Tolerance stability.
    lam = 2 * 0.495 / 0.01
    p10 = solve(A, 2.0, 1.3, 1.0, mu, lam, rtol=1e-10, atol=1e-12)[0]
    p12 = solve(A, 2.0, 1.3, 1.0, mu, lam)[0]
    _check(abs(p10 / p12 - 1) < 1e-8, f"tolerance stability: {p10} vs {p12}")
    print("checks passed: incompressible limit, neo-Hookean and Yeoh (lz = 1, 0.9, free ends: pressure and lz), "
          "Lamé limits (free, cased, Yeoh), tolerance")


def _tables():
    mu, A = 1.0, 1.0
    print("\nFree outer wall, p/mu (plane strain | free ends, with lz)")
    for la in (1.1, 1.3):
        for BA in (1.5, 2.0):
            for nu in (0.49, 0.495):
                lam = 2 * nu / (1 - 2 * nu)
                pps = solve(A, BA, la, 1.0, mu, lam)[0]
                pfe, lz = solve_free_ends(A, BA, la, mu, lam)
                print(f"  la={la} B/A={BA} nu={nu:<6} {pps:.5f} | {pfe:.5f} (lz={lz:.5f})  free vs plane {100 * (pfe / pps - 1):+.2f} %")
    print("\nOuter wall by case, la=1.1, B/A=2, p/mu (free wall, free ends | cased, free ends | cased, plane strain)")
    for nu in (0.4, 0.475, 0.49, 0.495, 0.4995):
        lam = 2 * nu / (1 - 2 * nu)
        pf = solve_free_ends(A, 2.0, 1.1, mu, lam)[0]
        pc, lzc = solve_cased_free_ends(A, 2.0, 1.1, mu, lam)
        pcp = solve_cased(A, 2.0, 1.1, 1.0, mu, lam)[0]
        print(f"  nu={nu:<7} {pf:.5f} | {pc:.5f} (lz={lzc:.4f}) | {pcp:.4f}")
    print("\nThe Yeoh term's increment, p_Yeoh / p_NH - 1, B/A=2, nu=0.49, free wall, free ends (plan 16h)")
    lam = 2 * 0.49 / (1 - 2 * 0.49)
    for name, c2 in (("ECOFLEX_00_30", 2050.0 / 23000.0), ("DRAGON_SKIN_10A", 4460.0 / 51000.0)):
        rises = []
        for la in (1.1, 1.3):
            pn = solve_free_ends(A, 2.0, la, mu, lam)[0]
            py = solve_free_ends(A, 2.0, la, mu, lam, c2=c2)[0]
            rises.append(f"la={la}: {100 * (py / pn - 1):+.2f} %")
        print(f"  {name:<16} c2/mu={c2:.4f}  " + "  ".join(rises))
    print("\nCompressible vs the incompressible formula, la=1.3, B/A=2, plane strain")
    for nu in (0.4, 0.49, 0.495):
        lam = 2 * nu / (1 - 2 * nu)
        p = solve(A, 2.0, 1.3, 1.0, mu, lam)[0]
        print(f"  nu={nu:<6} p/mu={p:.6f}  vs Haughton-Ogden {100 * (p / haughton_ogden(A, 2.0, 1.3, 1.0, mu) - 1):+.2f} %")


# The Yeoh case's C2/mu: ECOFLEX_00_30's C2 (2 050 Pa) over its mu (23 kPa), `sim-soft`'s silicone table (plan 16h).
YEOH_C2_OVER_MU = 2050.0 / 23000.0

# K2's corners, the confined case and the Yeoh case (plan §15a, 15d.8, 16h): (a/A, B/A, nu, c2/mu, walls).
GOLDEN_CASES = [
    (1.1, 2.0, 0.49, 0.0, "Free"),
    (1.1, 2.0, 0.495, 0.0, "Free"),
    (1.3, 2.0, 0.49, 0.0, "Free"),
    (1.3, 2.0, 0.495, 0.0, "Free"),
    (1.1, 2.0, 0.49, 0.0, "Cased"),
    (1.3, 2.0, 0.49, YEOH_C2_OVER_MU, "Free"),
]

# The plan's own values for the neo-Hookean cases (§15b's table, 15d.8), to 5 significant figures.
PLAN_VALUES = {
    (1.1, 0.49, "Free"): (0.12352, 0.98523),
    (1.1, 0.495, "Free"): (0.12365, 0.98511),
    (1.3, 0.49, "Free"): (0.30507, 0.95779),
    (1.3, 0.495, "Free"): (0.30544, 0.95747),
    (1.1, 0.49, "Cased"): (4.1417, 1.0),
}

# The plan's Yeoh increment (16h's table, a scratch run of this oracle's extension): p_Yeoh / p_NH - 1, in %,
# at (a/A, nu) with c2/mu = YEOH_C2_OVER_MU, free wall, free ends, to 2 decimals.
PLAN_YEOH_INCREMENT = {(1.3, 0.49): 4.22}


def golden_values(la, BA, nu, walls, c2=0.0, h=1e-6):
    """p/mu, lz, d(p/mu)/d(a/A) at fixed lz, and d(p/mu)/dlz at fixed a, by central differences."""
    mu, A = 1.0, 1.0
    lam = 2 * nu / (1 - 2 * nu)
    if walls == "Free":
        p, lz = solve_free_ends(A, BA, la, mu, lam, c2=c2)
        at = lambda a, z: solve(A, BA, a, z, mu, lam, c2=c2)[0]
    else:
        _check(c2 == 0.0, "the cased oracle is neo-Hookean only")
        lz = 1.0
        p = solve_cased(A, BA, la, lz, mu, lam)[0]
        at = lambda a, z: solve_cased(A, BA, a, z, mu, lam)[0]
    per_a = (at(la + h, lz) - at(la - h, lz)) / (2 * h)
    per_lz = (at(la, lz + h) - at(la, lz - h)) / (2 * h)
    return p, lz, per_a, per_lz


def _rust_case(la, BA, nu, c2, walls, values):
    p, lz, per_a, per_lz = values
    fields = [
        ("mandrel_ratio", repr(la)),
        ("thickness_ratio", repr(BA)),
        ("poisson", repr(nu)),
        ("c2_over_mu", repr(float(c2))),
        ("walls", f"Walls::{walls}"),
        ("pressure_over_mu", repr(float(p))),
        ("axial_stretch", repr(float(lz))),
        ("pressure_per_mandrel_ratio", repr(float(per_a))),
        ("pressure_per_axial_stretch", repr(float(per_lz))),
    ]
    body = "".join(f"        {name}: {value},\n" for name, value in fields)
    return f"    TubeCase {{\n{body}    }},\n"


def write_golden(path):
    rows = []
    neo_hookean = {}
    for la, BA, nu, c2, walls in GOLDEN_CASES:
        values = golden_values(la, BA, nu, walls, c2)
        if c2 == 0.0:
            plan_p, plan_lz = PLAN_VALUES[(la, nu, walls)]
            _check(abs(values[0] - plan_p) <= 0.5e-4 * max(1.0, plan_p), f"{la} {nu} {walls}: p/mu {values[0]} vs plan {plan_p}")
            _check(abs(values[1] - plan_lz) <= 0.5e-5, f"{la} {nu} {walls}: lz {values[1]} vs plan {plan_lz}")
            neo_hookean[(la, BA, nu, walls)] = values[0]
        else:
            increment = 100 * (values[0] / neo_hookean[(la, BA, nu, walls)] - 1)
            plan = PLAN_YEOH_INCREMENT[(la, nu)]
            _check(abs(increment - plan) <= 0.005, f"{la} {nu} Yeoh increment {increment:.4f} % vs plan {plan} %")
        rows.append(_rust_case(la, BA, nu, c2, walls, values))
    text = (
        "// Generated by `uv run docs/soft_contact/thick_tube_reference.py --golden <this file>`.\n"
        "// Do not edit: regenerate it, and review the change (plan §16g).\n"
        "\n"
        "//! The thick-tube oracle's golden values: K2's corners, the confined case and\n"
        "//! the Yeoh case (plan §15a, 15d.8, 16h), from\n"
        "//! `docs/soft_contact/thick_tube_reference.py`.\n"
        "\n"
        "use super::tube::{TubeCase, Walls};\n"
        "\n"
        "/// The cases, in the oracle's units (`A = 1`, `mu = 1`).\n"
        "// Full-precision digits, as the oracle printed them.\n"
        "#[allow(clippy::unreadable_literal)]\n"
        f"pub const THICK_TUBE: [TubeCase; {len(rows)}] = [\n"
        + "".join(rows)
        + "];\n"
    )
    with open(path, "w", encoding="utf-8") as f:
        f.write(text)
    print(f"wrote {len(rows)} cases to {path}")


if __name__ == "__main__":
    import sys

    np.seterr(invalid="ignore")  # the lz bracket search probes infeasible stretches; they return NaN and are rejected
    _checks()
    if len(sys.argv) == 3 and sys.argv[1] == "--golden":
        write_golden(sys.argv[2])
    else:
        _tables()
