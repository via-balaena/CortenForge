"""Regenerate `scaled_moment_arms_opensim.json` — the real-OpenSim reference for
the A3 **differential oracle**: it grades our per-axis morph (`cf_msk_lib::realize`)
against OpenSim's own ScaleTool.

The keystone of A3 validation: a *dialed* body has no real subject, so there is no
moment-arm oracle for it — EXCEPT that our morph (per-segment, per-axis scaling of
bone offsets + muscle attachment points, joint coupling left symbolic) is exactly
what OpenSim's ScaleTool does. So for any scale vector we can scale gait2392 with
real OpenSim, compute moment arms, and grade our `realize`d model against it. This
turns "uniform scale = exact dilation (analytic)" into "anisotropic per-axis =
matches OpenSim's own scaling (empirical)".

Run (needs the `opensim` PyPI wheel):

    uv venv --python 3.12 /tmp/osim-venv
    uv pip install --python /tmp/osim-venv opensim
    /tmp/osim-venv/bin/python gen_scaled_moment_arms.py gait2392.osim scaled_moment_arms_opensim.json

Each config is per-body scale FACTORS as an OpenSim body-frame Vec3 [x, y, z],
where **y is axial (length)** and **x = z is transverse (girth)** — the gait2392
limb long axis is body-frame y. This ordering is `cf_msk_lib::SegmentScale.vector()`
= (transverse, axial, transverse), so the Rust cross-check reads these factors back
into a `BodyParams` directly. The knee is swept 0 -> -100 deg (the same ROM as
`gen_moment_arms.py`), all other coordinates at their defaults.
"""
import json
import math
import sys
import opensim as osim

osim.Logger.setLevelString("Error")  # silence missing-geometry warnings

OSIM = sys.argv[1]
OUT = sys.argv[2]
MUSCLES = ["rect_fem_r", "vas_int_r", "bifemlh_r", "semimem_r"]
ANGLES_DEG = [-(5.0 * i) for i in range(21)]  # 0, -5, ... -100

# Scale grid — exercises length (axial=y) and girth (transverse=x,z) on each
# segment, plus a "realistic subject" mix. Bodies omitted stay at scale 1.
CONFIGS = {
    "uniform_1.137": {"pelvis": (1.137,) * 3, "femur_r": (1.137,) * 3,
                      "tibia_r": (1.137,) * 3, "talus_r": (1.137,) * 3},
    "femur_axial_1.2": {"femur_r": (1.0, 1.2, 1.0)},
    "femur_transverse_1.3": {"femur_r": (1.3, 1.0, 1.3)},
    "tibia_axial_0.9": {"tibia_r": (1.0, 0.9, 1.0)},
    "tibia_transverse_1.2": {"tibia_r": (1.2, 1.0, 1.2)},
    # A plausible taller/leaner subject: longer + slightly thicker thigh, longer +
    # slightly leaner shank.
    "realistic_mix": {"femur_r": (1.05, 1.10, 1.05), "tibia_r": (0.95, 1.08, 0.95)},
}


def moment_arms(factors):
    """Scale gait2392 by `factors` (per-body Vec3) and return knee moment arms."""
    model = osim.Model(OSIM)
    state = model.initSystem()
    if factors:
        scales = osim.ScaleSet()
        for body, (sx, sy, sz) in factors.items():
            sc = osim.Scale()
            sc.setSegmentName(body)
            sc.setScaleFactors(osim.Vec3(sx, sy, sz))
            sc.setApply(True)
            scales.cloneAndAppend(sc)
        if not model.scale(state, scales, True):  # preserveMassDist=True
            raise RuntimeError("model.scale returned False")
        state = model.initSystem()  # fresh state for the scaled model
    coord = model.getCoordinateSet().get("knee_angle_r")
    out = {}
    for m in MUSCLES:
        musc = osim.Muscle.safeDownCast(model.getForceSet().get(m))
        rows = []
        for adeg in ANGLES_DEG:
            arad = math.radians(adeg)
            coord.setValue(state, arad, True)  # enforce constraints
            model.realizePosition(state)
            rows.append({
                "angle_deg": adeg,
                "angle_rad": arad,
                "moment_arm_m": musc.computeMomentArm(state, coord),
            })
        out[m] = rows
    return out


configs = {}
for name, factors in CONFIGS.items():
    configs[name] = {
        "factors": {b: list(v) for b, v in factors.items()},
        "muscles": moment_arms(factors),
    }

with open(OUT, "w") as f:
    json.dump({
        "source": "OpenSim " + osim.GetVersion(),
        "model": OSIM.split("/")[-1],
        "coordinate": "knee_angle_r",
        "convention": "moment_arm = -d(length)/d(coordinate), meters; "
                      "factors are body-frame Vec3 [x,y,z], y=axial(length), x=z=transverse(girth)",
        "configs": configs,
    }, f, indent=1)

print(f"OpenSim {osim.GetVersion()}  scaled-knee moment arms (mm) at 0 / 50 / 100 deg flexion:")
for name in CONFIGS:
    print(f"  [{name}]")
    for m in MUSCLES:
        r = configs[name]["muscles"][m]
        print(f"    {m:<11} {r[0]['moment_arm_m']*1000:7.2f} "
              f"{r[10]['moment_arm_m']*1000:7.2f} {r[20]['moment_arm_m']*1000:7.2f}")
