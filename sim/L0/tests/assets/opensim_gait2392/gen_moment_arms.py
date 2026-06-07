"""Regenerate `knee_moment_arms_opensim.json` — the real-OpenSim reference that
`cf-osim`'s `opensim_cross_check` test grades the oracle against.

Run (needs the `opensim` PyPI wheel; arm64/x86 macOS + Linux + Windows wheels
exist for CPython 3.11-3.13):

    uv venv --python 3.12 /tmp/osim-venv
    uv pip install --python /tmp/osim-venv opensim
    /tmp/osim-venv/bin/python gen_moment_arms.py gait2392.osim knee_moment_arms_opensim.json

Sweeps knee_angle_r 0 -> -100 deg (flexion) and records each target muscle's
moment arm about the knee (OpenSim convention: r = -d(length)/d(coordinate), the
total derivative through the coupled CustomJoint). All other coordinates stay at
their defaults (hip etc. neutral), matching the cf-osim knee-only study.
"""
import json
import math
import sys
import opensim as osim

osim.Logger.setLevelString("Error")  # silence missing-geometry warnings

OSIM = sys.argv[1]
OUT = sys.argv[2]
MUSCLES = ["rect_fem_r", "vas_int_r", "bifemlh_r", "semimem_r"]

model = osim.Model(OSIM)
state = model.initSystem()
coord = model.getCoordinateSet().get("knee_angle_r")

results = {}
angles_deg = [-(5.0 * i) for i in range(21)]  # 0, -5, ... -100

for mname in MUSCLES:
    musc = osim.Muscle.safeDownCast(model.getForceSet().get(mname))
    rows = []
    for adeg in angles_deg:
        arad = math.radians(adeg)
        coord.setValue(state, arad, True)  # enforce constraints
        model.realizePosition(state)
        rows.append({
            "angle_deg": adeg,
            "angle_rad": arad,
            "moment_arm_m": musc.computeMomentArm(state, coord),
            "length_m": musc.getGeometryPath().getLength(state),
        })
    results[mname] = rows

with open(OUT, "w") as f:
    json.dump({
        "source": "OpenSim " + osim.GetVersion(),
        "model": OSIM.split("/")[-1],
        "coordinate": "knee_angle_r",
        "convention": "moment_arm = -d(length)/d(coordinate), meters",
        "muscles": results,
    }, f, indent=1)

print(f"OpenSim {osim.GetVersion()}  moment arms (mm) at 0 / 50 / 100 deg flexion:")
for m in MUSCLES:
    r = results[m]
    print(f"  {m:<11} {r[0]['moment_arm_m']*1000:7.2f} "
          f"{r[10]['moment_arm_m']*1000:7.2f} {r[20]['moment_arm_m']*1000:7.2f}")
