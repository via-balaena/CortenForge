#!/usr/bin/env python3
# /// script
# requires-python = ">=3.11"
# dependencies = ["numpy>=2,<3"]
#
# [tool.uv]
# exclude-newer = "2026-09-24T00:00:00Z"
# ///
"""How far the Dundurs coupling moves K6's stick zone: a rigid cylinder on an elastic half-plane.

K6 (`docs/SOFT_CONTACT_ARCHITECTURE_RECON.md` §16b) judges the explicit solver's stick zone against the
closed forms of Cattaneo-Mindlin (loading) and Mindlin-Deresiewicz (unloading). Those assume the normal
and tangential problems are uncoupled, which for a rigid indenter holds only at nu = 0.5. At nu < 0.5 a
rigid body leaves Dundurs' beta = (1 - 2 nu) / (2 (1 - nu)). This script measures what that coupling does
to the stick zone's half-width, so K6's error budget has a number that can be re-run.

Method: a plane-strain boundary-element solution with piecewise-constant tractions on a uniform grid,
and an incremental active set (out of contact, sticking, slipping either way).
- Surface displacements of a half-plane under a line load (Popov et al., Handbook of Plane Contact
  Mechanics, eq. 2.7, after Barber 2018): the normal one is -(1 - nu)/(pi mu) ln|x| per unit normal
  load, plus (1 - 2 nu)/(4 mu) sgn(x) per unit tangential load; the tangential one mirrors it.
- Load path (force control, the normal load held after step 1):
  1. the normal load rises to the value whose frictionless Hertz half-width is a = 1;
  2. the tangential load rises to 0.8 f P;
  3. it falls back to 0.
- A sticking element keeps its tangential displacement increment equal to the indenter's shift; a
  slipping one carries f p in the direction it slips.

Run:  uv run docs/soft_contact/cattaneo_mindlin_reference.py
It checks itself before reporting, and a failed check raises (so the checks survive `python -O`):
- at nu = 0.5 (beta = 0) it reproduces both closed forms within one element;
- the coupling's effect at nu = 0.49 agrees between two grids within one coarse element;
- the instrument responds: at nu = 0.475 the effect is at least three times that at 0.49, and two elements.

The solver was first written by a cold reviewer of the step-2 design (2026-09-24) and reviewed and
hardened before it was committed. It is small-strain and half-plane: it bounds the coupling only, not the
explicit solver's finite strain, finite domain or element.
"""
import numpy as np

FRICTION = 0.3
PEAK = 0.8  # the tangential load's peak, as a fraction of f P
BAND = (0.2, 0.8)  # K6 judges samples whose load fraction lies in this band


class CheckFailed(RuntimeError):
    """A self-check failed."""


def _check(ok, what):
    if not ok:
        raise CheckFailed(what)


def _influence(x, d):
    """Per-element integrals over each element [xi - d/2, xi + d/2]: of -ln|x - xi| and of sgn(x - xi)."""
    lower = x - d / 2
    upper = x + d / 2
    X = x[:, None]

    def antiderivative(t):  # G(xi) = (x - xi) ln|x - xi| - (x - xi), so dG/dxi = -ln|x - xi|
        with np.errstate(divide="ignore", invalid="ignore"):
            log_part = np.where(t == 0, 0.0, t * np.log(np.abs(t)))
        return log_part - t

    log_kernel = antiderivative(X - upper[None, :]) - antiderivative(X - lower[None, :])
    sign_kernel = np.abs(X - lower[None, :]) - np.abs(X - upper[None, :])
    return log_kernel, sign_kernel


def run(nu, elements, span=1.3, normal_steps=60, loading_steps=80, unloading_steps=80):
    """Return (a, rows, h): rows are (phase, load fraction, half-width / a, closed form, offset / a)."""
    shear_modulus = 1.0
    direct = (1 - nu) / shear_modulus
    coupled = (1 - 2 * nu) / (4 * shear_modulus)
    h = 2 * span / elements
    x = -span + h * (np.arange(elements) + 0.5)
    log_kernel, sign_kernel = _influence(x, h)
    k_normal_normal = direct / np.pi * log_kernel
    k_normal_tangential = coupled * sign_kernel
    k_tangential_normal = -coupled * sign_kernel
    k_tangential_tangential = direct / np.pi * log_kernel
    radius = 1.0
    normal_load = np.pi / (2 * radius * direct)  # frictionless Hertz: a^2 = 2 P R (1 - nu) / (pi mu) = 1

    OUT, STICK, SLIP_POSITIVE, SLIP_NEGATIVE = 0, 1, 2, 3
    state = {
        "status": np.zeros(elements, dtype=int),
        "u_previous": np.zeros(elements),
        "shift_previous": 0.0,
    }

    def solve(total_normal, total_tangential):
        status = state["status"].copy()
        for _ in range(200):
            active = np.where(status != OUT)[0]
            if len(active) == 0:
                status[np.argmin(np.abs(x))] = STICK
                continue
            n = len(active)
            # Unknowns: p on the active elements, q on them, the approach, the indenter's shift.
            matrix = np.zeros((2 * n + 2, 2 * n + 2))
            rhs = np.zeros(2 * n + 2)
            matrix[:n, :n] = k_normal_normal[np.ix_(active, active)]
            matrix[:n, n:2 * n] = k_normal_tangential[np.ix_(active, active)]
            matrix[:n, 2 * n] = -1.0
            rhs[:n] = -x[active] ** 2 / (2 * radius)
            for row, i in enumerate(active):
                r = n + row
                if status[i] == STICK:
                    matrix[r, :n] = k_tangential_normal[i, active]
                    matrix[r, n:2 * n] = k_tangential_tangential[i, active]
                    matrix[r, 2 * n + 1] = -1.0
                    rhs[r] = state["u_previous"][i] - state["shift_previous"]
                else:
                    direction = 1.0 if status[i] == SLIP_POSITIVE else -1.0
                    matrix[r, n + row] = 1.0
                    matrix[r, row] = -direction * FRICTION
            matrix[2 * n, :n] = h
            rhs[2 * n] = total_normal
            matrix[2 * n + 1, n:2 * n] = h
            rhs[2 * n + 1] = total_tangential
            solution = np.linalg.solve(matrix, rhs)
            p = np.zeros(elements)
            q = np.zeros(elements)
            p[active] = solution[:n]
            q[active] = solution[n:2 * n]
            approach, shift = solution[2 * n], solution[2 * n + 1]
            normal_displacement = k_normal_normal @ p + k_normal_tangential @ q
            tangential_displacement = k_tangential_normal @ p + k_tangential_tangential @ q
            gap = x ** 2 / (2 * radius) - approach + normal_displacement

            changed = False
            updated = status.copy()
            for i in range(elements):
                if status[i] == OUT:
                    if gap[i] < -1e-14:
                        updated[i] = STICK
                        changed = True
                elif p[i] < 0:
                    updated[i] = OUT
                    changed = True
                elif status[i] == STICK:
                    if abs(q[i]) > FRICTION * p[i] * (1 + 1e-12):
                        updated[i] = SLIP_POSITIVE if q[i] > 0 else SLIP_NEGATIVE
                        changed = True
                else:
                    direction = 1.0 if status[i] == SLIP_POSITIVE else -1.0
                    # The indenter's motion relative to the surface, this increment.
                    relative = (shift - state["shift_previous"]) - (
                        tangential_displacement[i] - state["u_previous"][i]
                    )
                    if relative * direction < -1e-15:
                        updated[i] = STICK
                        changed = True
            if not changed:
                state["status"] = status
                state["u_previous"] = tangential_displacement
                state["shift_previous"] = shift
                return
            status = updated
        raise CheckFailed(f"active set did not converge at nu={nu}, {elements} elements")

    def stick_zone():
        sticking = np.where(state["status"] == STICK)[0]
        if len(sticking) == 0:
            return 0.0, 0.0
        half_width = (x[sticking].max() - x[sticking].min() + h) / 2
        centre = (x[sticking].max() + x[sticking].min()) / 2
        return half_width, centre

    for step in range(1, normal_steps + 1):
        solve(normal_load * step / normal_steps, 0.0)
    contact = np.where(state["status"] != OUT)[0]
    a = (x[contact].max() - x[contact].min() + h) / 2

    rows = []
    peak = PEAK * FRICTION * normal_load
    # Load fractions come from the step index, so every nu samples the same fractions exactly.
    for step in range(1, loading_steps + 1):
        fraction = PEAK * step / loading_steps
        solve(normal_load, fraction * FRICTION * normal_load)
        half_width, centre = stick_zone()
        rows.append(("load", fraction, half_width / a, np.sqrt(1 - fraction), centre / a))
    for step in range(1, unloading_steps + 1):
        fraction = PEAK * step / unloading_steps  # the fall from the peak, over f P
        solve(normal_load, peak - fraction * FRICTION * normal_load)
        half_width, centre = stick_zone()
        rows.append(("unload", fraction, half_width / a, np.sqrt(1 - fraction / 2), centre / a))
    return a, rows, h


def _in_band(rows, phase):
    low, high = BAND
    return [r for r in rows if r[0] == phase and low <= r[1] <= high + 1e-9]


def closed_form_error(rows, phase):
    """Largest |half-width - closed form| / a over K6's band."""
    return max(abs(r[2] - r[3]) for r in _in_band(rows, phase))


def coupling_effect(rows, rows_uncoupled, phase):
    """Largest |half-width(nu) - half-width(0.5)| / a over K6's band, step by step on the same grid."""
    low, high = BAND
    _check(len(rows) == len(rows_uncoupled), "the two runs took different load steps")
    pairs = [
        (r, b)
        for r, b in zip(rows, rows_uncoupled)
        if r[0] == phase and low <= b[1] <= high + 1e-9
    ]
    _check(all(r[1] == b[1] for r, b in pairs), "paired steps have different load fractions")
    return max(abs(r[2] - b[2]) for r, b in pairs)


def largest_offset(rows, phase):
    return max(abs(r[4]) for r in _in_band(rows, phase))


def _checks_and_report():
    coarse, fine = 520, 1040
    results = {}
    for elements in (coarse, fine):
        for nu in (0.5, 0.495, 0.49):
            results[(nu, elements)] = run(nu, elements)
    results[(0.475, coarse)] = run(0.475, coarse)
    h_coarse = results[(0.5, coarse)][2]

    # 1. Uncoupled: both closed forms within one element.
    for elements in (coarse, fine):
        a, rows, h = results[(0.5, elements)]
        _check(abs(a - 1) <= h, f"Hertz half-width {a} at {elements} elements")
        for phase in ("load", "unload"):
            error = closed_form_error(rows, phase)
            _check(error <= h, f"closed form, {phase}, {elements} elements: {error:.4f} > {h:.4f}")
    # 2. The coupling's effect at nu = 0.49 agrees between the grids within one coarse element.
    effects = {}
    for elements in (coarse, fine):
        rows_uncoupled = results[(0.5, elements)][1]
        for phase in ("load", "unload"):
            effects[(phase, elements)] = coupling_effect(results[(0.49, elements)][1], rows_uncoupled, phase)
    for phase in ("load", "unload"):
        spread = abs(effects[(phase, coarse)] - effects[(phase, fine)])
        _check(spread <= h_coarse, f"coupling at nu=0.49, {phase}: grids differ by {spread:.4f}")
    # 3. The instrument responds to beta: by two coarse elements at least, and three times the effect at 0.49.
    at_475 = coupling_effect(results[(0.475, coarse)][1], results[(0.5, coarse)][1], "load")
    _check(
        at_475 >= max(3 * effects[("load", coarse)], 2 * h_coarse),
        f"nu=0.475 moves the zone only {at_475:.4f}",
    )
    print("checks passed: closed forms at beta = 0 (two grids), grid agreement at nu = 0.49, response at nu = 0.475")

    print(f"\nThe coupling's effect on the stick zone's half-width, over load fractions {BAND[0]}-{BAND[1]}")
    print("(|half-width(nu) - half-width(0.5)| / a, largest; offset = the zone's centre / a, largest)")
    for nu in (0.495, 0.49, 0.475):
        beta = (1 - 2 * nu) / (2 * (1 - nu))
        grids = (coarse, fine) if nu != 0.475 else (coarse,)
        for elements in grids:
            rows = results[(nu, elements)][1]
            rows_uncoupled = results[(0.5, elements)][1]
            h = results[(nu, elements)][2]
            print(
                f"  nu={nu:<6} beta={beta:.4f} h=a/{1 / h:.0f}: "
                f"loading {coupling_effect(rows, rows_uncoupled, 'load'):.4f} "
                f"(offset {largest_offset(rows, 'load'):.4f}), "
                f"unloading {coupling_effect(rows, rows_uncoupled, 'unload'):.4f} "
                f"(offset {largest_offset(rows, 'unload'):.4f})"
            )


if __name__ == "__main__":
    _checks_and_report()
