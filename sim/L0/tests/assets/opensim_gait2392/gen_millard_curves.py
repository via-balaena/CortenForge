"""Regenerate `millard_curves_opensim.json` — dense samples of OpenSim's default
Millard2012 fiber curves (active force-length, passive force-length,
force-velocity). Ground truth for the unit-level validation of our Rust port
(`sim/L0/core/src/forward/millard.rs`), graded by `tools/cf-osim`'s
`millard_force_cross_check`.

Run:
    /tmp/osim-venv/bin/python gen_millard_curves.py gait2392.osim millard_curves_opensim.json

All four target muscles use OpenSim's *default* curve parameters (asserted here),
so one shared curve set suffices. Each curve is sampled across its domain plus a
little into the linear-extrapolation region.
"""
import json
import sys
import opensim as osim

osim.Logger.setLevelString("Error")
OSIM = sys.argv[1]
OUT = sys.argv[2]
MUSCLES = ["rect_fem_r", "vas_int_r", "bifemlh_r", "semimem_r"]

model = osim.Model(OSIM)
model.initSystem()


def ev(curve, x):
    return curve.calcValue(osim.Vector(1, x))


def frange(lo, hi, n):
    return [lo + (hi - lo) * i / (n - 1) for i in range(n)]


# Collect each muscle's default curve parameters and assert they all match.
def curve_params(mus):
    a = mus.getActiveForceLengthCurve()
    p = mus.getFiberForceLengthCurve()
    f = mus.getForceVelocityCurve()
    return {
        "active": {
            "min_norm_active_fiber_length": a.get_min_norm_active_fiber_length(),
            "transition_norm_fiber_length": a.get_transition_norm_fiber_length(),
            "max_norm_active_fiber_length": a.get_max_norm_active_fiber_length(),
            "shallow_ascending_slope": a.get_shallow_ascending_slope(),
            "minimum_value": a.get_minimum_value(),
        },
        "passive": {
            "strain_at_zero_force": p.get_strain_at_zero_force(),
            "strain_at_one_norm_force": p.get_strain_at_one_norm_force(),
        },
        "force_velocity": {
            "concentric_slope_at_vmax": f.get_concentric_slope_at_vmax(),
            "concentric_slope_near_vmax": f.get_concentric_slope_near_vmax(),
            "isometric_slope": f.get_isometric_slope(),
            "eccentric_slope_at_vmax": f.get_eccentric_slope_at_vmax(),
            "eccentric_slope_near_vmax": f.get_eccentric_slope_near_vmax(),
            "max_eccentric_velocity_force_multiplier": f.get_max_eccentric_velocity_force_multiplier(),
            "concentric_curviness": f.get_concentric_curviness(),
            "eccentric_curviness": f.get_eccentric_curviness(),
        },
    }


ref = None
for name in MUSCLES:
    mus = osim.Millard2012EquilibriumMuscle.safeDownCast(model.getForceSet().get(name))
    pr = curve_params(mus)
    if ref is None:
        ref = pr
    elif pr != ref:
        raise SystemExit(f"{name} curve params differ from default: {pr} != {ref}")

# Sample the shared curves (use rect_fem_r's curve objects).
mus = osim.Millard2012EquilibriumMuscle.safeDownCast(model.getForceSet().get("rect_fem_r"))
afl = mus.getActiveForceLengthCurve()
pfl = mus.getFiberForceLengthCurve()
fvc = mus.getForceVelocityCurve()

# Domains slightly extended into the linear-extrapolation regions.
active = [{"x": x, "y": ev(afl, x)} for x in frange(0.3, 1.95, 200)]
passive = [{"x": x, "y": ev(pfl, x)} for x in frange(0.95, 1.85, 200)]
force_velocity = [{"x": x, "y": ev(fvc, x)} for x in frange(-1.1, 1.1, 200)]

with open(OUT, "w") as fp:
    json.dump(
        {
            "source": "OpenSim " + osim.GetVersion(),
            "model": OSIM.split("/")[-1],
            "note": "default Millard2012 fiber curves (shared by all 4 target muscles)",
            "params": ref,
            "active_force_length": active,
            "passive_force_length": passive,
            "force_velocity": force_velocity,
        },
        fp,
        indent=1,
    )

print(f"OpenSim {osim.GetVersion()}: sampled {len(active)} AFL + {len(passive)} PFL "
      f"+ {len(force_velocity)} FV points; defaults shared across all 4 muscles.")
