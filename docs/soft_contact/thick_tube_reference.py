#!/usr/bin/env python3
# /// script
# requires-python = ">=3.11"
# dependencies = ["numpy>=2,<3", "scipy>=1.14,<2"]
#
# [tool.uv]
# exclude-newer = "2026-09-24T00:00:00Z"
# ///
"""Reference contact pressure for a thick neo-Hookean tube on a rigid, frictionless mandrel.

This is the oracle for the explicit solver's first experiment
(`docs/SOFT_CONTACT_ARCHITECTURE_RECON.md` §15). It solves the tube's radial
equilibrium for the SAME compressible material the solver uses, in f64, so the
kill criterion compares like with like. The exact incompressible answer
(Haughton & Ogden) is only its ν → 0.5 limit.

Material: Psi = mu/2 (I1 - 3) - mu ln J + lam/2 (ln J)^2.
Reference radii A < R < B, deformed radius r(R), axial stretch lz.
Principal stretches l_r = r', l_t = r/R, l_z; nominal stresses
    P_i = dPsi/dl_i = mu (l_i - 1/l_i) + lam ln J / l_i.
Radial equilibrium: dP_r/dR + (P_r - P_t)/R = 0.
Inner wall on the mandrel: r(A) = a. Outer wall: free, P_r(B) = 0, or cased,
r(B) = B. Axial: prescribed lz (plane strain is lz = 1), or free ends, where
lz makes the axial force zero. Pressure is Cauchy: p = -P_r(A) / (l_t(A) lz).

Run:  uv run docs/soft_contact/thick_tube_reference.py
It runs its own checks before printing the reference tables: the incompressible
limit, the Lamé limits of the free and the cased wall, and tolerance stability.
A failed check raises, so the checks survive `python -O`.
"""
import numpy as np
from scipy.integrate import solve_ivp
from scipy.optimize import brentq

class CheckFailed(RuntimeError):
    """A self-check of the oracle failed."""


def _check(ok, what):
    if not ok:
        raise CheckFailed(what)


def P_i(li, J, mu, lam):
    return mu * (li - 1.0 / li) + lam * np.log(J) / li

def dP_dl_r(lr, lt, lz, mu, lam):
    # d P_r / d l_r at fixed l_t, l_z (J = lr lt lz): mu (1 + 1/lr^2) + lam (1 - ln J) / lr^2
    J = lr * lt * lz
    return mu * (1.0 + 1.0 / lr**2) + lam * (1.0 - np.log(J)) / lr**2

def dP_dl_t_of_Pr(lr, lt, lz, mu, lam):
    # d P_r / d l_t at fixed l_r: lam / (lr lt)
    return lam / (lr * lt)

def shoot(lr_A, A, B, a, lz, mu, lam, rtol, atol, strict=False):
    """Integrate from R = A with r(A) = a, r'(A) = lr_A; return (sol, P_r(B))."""
    def rhs(R, y):
        r, lr = y
        lt = r / R
        J = lr * lt * lz
        Pr = P_i(lr, J, mu, lam)
        Pt = P_i(lt, J, mu, lam)
        dPr_dR = -(Pr - Pt) / R           # equilibrium
        dlt_dR = (lr - lt) / R            # d(r/R)/dR
        dlr_dR = (dPr_dR - dP_dl_t_of_Pr(lr, lt, lz, mu, lam) * dlt_dR) / dP_dl_r(lr, lt, lz, mu, lam)
        return [lr, dlr_dR]
    sol = solve_ivp(rhs, (A, B), [a, lr_A], method="DOP853", rtol=rtol, atol=atol, dense_output=True)
    # Bracket probes may leave the feasible region; only the final solve must reach R = B.
    _check(not strict or sol.status == 0, f"integration stopped early: {sol.message}")
    rB, lrB = sol.y[:, -1]
    J = lrB * (rB / B) * lz
    return sol, P_i(lrB, J, mu, lam)

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


def solve(A, B, a, lz, mu, lam, rtol=1e-12, atol=1e-14):
    f = lambda lr_A: shoot(lr_A, A, B, a, lz, mu, lam, rtol, atol)[1]
    lo, hi = 0.3, 1.2
    lo, hi = _bracket(f, lo, hi, 0.9, 1.1)
    lr_A = brentq(f, lo, hi, xtol=1e-15, rtol=1e-15, maxiter=500)
    sol, _ = shoot(lr_A, A, B, a, lz, mu, lam, rtol, atol, strict=True)
    lt_A = a / A
    J_A = lr_A * lt_A * lz
    p = -P_i(lr_A, J_A, mu, lam) / (lt_A * lz)
    return p, sol

def axial_force(sol, A, B, lz, mu, lam, n=4001):
    """Axial force N = 2 pi int_A^B P_z R dR (nominal, per reference area)."""
    R = np.linspace(A, B, n)
    r, lr = sol.sol(R)
    lt = r / R
    J = lr * lt * lz
    Pz = P_i(lz, J, mu, lam)
    return 2 * np.pi * np.trapezoid(Pz * R, R)

def solve_free_ends(A, B, a, mu, lam):
    """Generalised plane strain with zero axial force: find lz with N(lz) = 0."""
    g = lambda lz: axial_force(solve(A, B, a, lz, mu, lam)[1], A, B, lz, mu, lam)
    lz = brentq(g, 0.7, 1.05, xtol=1e-13)
    p, sol = solve(A, B, a, lz, mu, lam)
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
    J_A = lr_A * lt_A * lz
    return -P_i(lr_A, J_A, mu, lam) / (lt_A * lz), sol

def solve_cased_free_ends(A, B, a, mu, lam):
    g = lambda lz: axial_force(solve_cased(A, B, a, lz, mu, lam)[1], A, B, lz, mu, lam)
    lo, hi = 1.0, 1.02
    while not (np.isfinite(g(hi)) and g(hi) > 0):  # axial force turns tensile once lz is large enough
        hi = 1.0 + 2.0 * (hi - 1.0)
        _check(hi < 1.6, "no bracket for lz")
    _check(g(lo) < 0, "no bracket for lz")
    lz = brentq(g, lo, hi, xtol=1e-13)
    return solve_cased(A, B, a, lz, mu, lam)[0], lz


def axial_force_incompressible(lz, A, B, a, mu):
    """Total axial force on an incompressible neo-Hookean tube on the mandrel (independent of the ODE).

    Kinematics are closed form, r^2 = a^2 + (R^2 - A^2)/lz. From d(r^2 s_rr)/dr = r (s_rr + s_tt),
    F = pi int_A^B mu (2 lz^2 - lr^2 - lt^2) R/lz dR + pi a^2 P, with lt = r/R and lr = 1/(lt lz).
    Free ends mean F = 0; in Haughton & Ogden's reduced force N, that is N + pi a^2 P = 0.
    """
    from scipy.integrate import quad

    def integrand(R):
        r = np.sqrt(a * a + (R * R - A * A) / lz)
        lt = r / R
        lr = 1.0 / (lt * lz)
        return mu * (2 * lz * lz - lr * lr - lt * lt) * R / lz

    N = np.pi * quad(integrand, A, B, epsabs=1e-14, epsrel=1e-13)[0]
    return N + np.pi * a * a * haughton_ogden(A, B, a, lz, mu)


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
    # 2. The Lamé limit: the relative gap shrinks in proportion to the interference.
    lam = 2 * 0.45 / 0.1
    gaps = [solve(A, 2.0, 1 + e, 1.0, mu, lam)[0] / lame_plane_strain(A, 2.0, 1 + e, mu, lam) - 1 for e in (1e-2, 1e-3)]
    _check(abs(gaps[1]) < abs(gaps[0]) / 5, f"Lame limit, free wall: {gaps}")
    # ... and of the cased wall held in plane strain, the confined case.
    gaps = [solve_cased(A, 2.0, 1 + e, 1.0, mu, lam)[0] / lame_cased_plane_strain(A, 2.0, 1 + e, mu, lam) - 1 for e in (1e-2, 1e-3)]
    _check(abs(gaps[1]) < abs(gaps[0]) / 5, f"Lame limit, cased wall: {gaps}")
    # 3. Tolerance stability.
    lam = 2 * 0.495 / 0.01
    p10 = solve(A, 2.0, 1.3, 1.0, mu, lam, rtol=1e-10, atol=1e-12)[0]
    p12 = solve(A, 2.0, 1.3, 1.0, mu, lam)[0]
    _check(abs(p10 / p12 - 1) < 1e-8, f"tolerance stability: {p10} vs {p12}")
    print("checks passed: incompressible limit (lz = 1, 0.9, free ends: pressure and lz), Lamé limits (free, cased), tolerance")


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
    print("\nCompressible vs the incompressible formula, la=1.3, B/A=2, plane strain")
    for nu in (0.4, 0.49, 0.495):
        lam = 2 * nu / (1 - 2 * nu)
        p = solve(A, 2.0, 1.3, 1.0, mu, lam)[0]
        print(f"  nu={nu:<6} p/mu={p:.6f}  vs Haughton-Ogden {100 * (p / haughton_ogden(A, 2.0, 1.3, 1.0, mu) - 1):+.2f} %")


if __name__ == "__main__":
    np.seterr(invalid="ignore")  # the lz bracket search probes infeasible stretches; they return NaN and are rejected
    _checks()
    _tables()
