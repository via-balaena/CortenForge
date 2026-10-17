#!/usr/bin/env python3
"""Generate the touch golden data from the unfused MuJoCo 3.5.0 oracle.

Run it with the oracle's own interpreter (build_mujoco_oracle.sh builds it):

    <workdir>/venv/bin/python -I sim/L0/tests/scripts/gen_touch_reference.py \\
        sim/L0/tests/assets/golden/touch

It refuses any other interpreter, as gen_census_golden.py does.

MuJoCo's touch sensor adds, for each contact of its site's body in contact
order, the normal force `mj_contactForce` decodes for that contact (a
pyramidal contact's facets summed first, `mju_decodePyramid`), skipping a
normal force not above 0 and a contact whose normal's ray misses the site's
zone (`engine_sensor.c:887-930`). The cases: a free box resting on a plane
(four contacts) and a free sphere resting beside it (one), each with a touch
sensor on a site that encloses it, at condim 1, 3, 4 and 6 under the
pyramidal cone and condim 3 under the elliptic one. For each, MuJoCo's state
(qpos, qvel, qacc_warmstart, time) before steps 100 to 119, and after
mj_forward at that state: ncon, efc_force and sensordata; and one more
state, step 100's with the sphere's contact rows' forces negated and
mj_sensorAcc run again, so that contact's normal force is below 0 and the
sphere's touch reads 0.
"""
import json
import os
import sys

import mujoco

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from gen_census_golden import finite_or_string, oracle  # noqa: E402

STEPS = range(100, 120)
CASES = (("pyramidal", 1), ("pyramidal", 3), ("pyramidal", 4), ("pyramidal", 6), ("elliptic", 3))


def xml(cone, condim):
    return f"""<mujoco model="touch_{cone}_{condim}">
  <option timestep="0.002" cone="{cone}"/>
  <worldbody>
    <geom type="plane" size="2 2 0.1" condim="{condim}"/>
    <body name="box" pos="0 0 0.12" euler="0.05 -0.03 0.2">
      <freejoint/>
      <geom type="box" size="0.2 0.15 0.1" mass="2" condim="{condim}"/>
      <site name="zone" type="box" size="0.5 0.5 0.5"/>
    </body>
    <body name="ball" pos="0.6 0 0.06">
      <freejoint/>
      <geom name="ball_geom" type="sphere" size="0.05" mass="0.3" condim="{condim}"/>
      <site name="ballzone" type="sphere" size="0.2"/>
    </body>
  </worldbody>
  <sensor>
    <touch site="zone"/>
    <touch site="ballzone"/>
  </sensor>
</mujoco>"""


def floats(a):
    return [float(x) for x in a]


def case(cone, condim):
    text = xml(cone, condim)
    m = mujoco.MjModel.from_xml_string(text)
    d = mujoco.MjData(m)
    states = []
    for step in range(max(STEPS) + 1):
        if step in STEPS:
            st = {"step": step, "qpos": floats(d.qpos), "qvel": floats(d.qvel),
                  "qacc_warmstart": floats(d.qacc_warmstart), "time": float(d.time)}
            mujoco.mj_forward(m, d)
            if d.ncon != 5:
                sys.exit(f"touch_{cone}_{condim} step {step}: {d.ncon} contacts")
            st.update({"ncon": int(d.ncon), "efc_force": floats(d.efc_force),
                       "sensordata": floats(d.sensordata)})
            states.append(st)
            if step == min(STEPS):
                states.append(negated(m, d, dict(st)))
        mujoco.mj_step(m, d)
    return {"xml": text, "states": states}


def negated(m, d, st):
    """`st`, after its mj_forward, with the sphere's contact rows' forces
    negated and mj_sensorAcc run again; `d` is left as it was."""
    saved = d.efc_force.copy()
    ball = m.geom("ball_geom").id
    rows = [i for i in range(d.nefc)
            if d.efc_type[i] in (mujoco.mjtConstraint.mjCNSTR_CONTACT_PYRAMIDAL,
                                 mujoco.mjtConstraint.mjCNSTR_CONTACT_ELLIPTIC,
                                 mujoco.mjtConstraint.mjCNSTR_CONTACT_FRICTIONLESS)
            and ball in d.contact[d.efc_id[i]].geom]
    if not rows:
        sys.exit("no rows of the sphere's contact")
    d.efc_force[rows] = -d.efc_force[rows]
    mujoco.mj_sensorAcc(m, d)
    st.update({"negated_sphere_rows": rows, "efc_force": floats(d.efc_force),
               "sensordata": floats(d.sensordata)})
    if st["sensordata"][1] != 0:
        sys.exit(f"the sphere's touch with its normal negated: {st['sensordata'][1]}")
    d.efc_force[:] = saved
    return st


def main():
    if len(sys.argv) != 2:
        sys.exit(__doc__)
    marker = oracle()
    cases = {f"{cone}_{condim}": case(cone, condim) for cone, condim in CASES}
    out = {"oracle": marker, "cases": finite_or_string(cases)}
    with open(os.path.join(sys.argv[1], "touch.json"), "w", encoding="utf-8") as f:
        json.dump(out, f, separators=(",", ":"))
        f.write("\n")
    print(f"{len(cases)} cases -> {sys.argv[1]}")


if __name__ == "__main__":
    main()
