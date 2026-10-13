#!/usr/bin/env python3
"""Generate the RK4 sleep golden data from the unfused MuJoCo 3.5.0 oracle.

Run it with the oracle's own interpreter (build_mujoco_oracle.sh builds it):

    <workdir>/venv/bin/python -I sim/L0/tests/scripts/gen_rk4_sleep_reference.py \\
        sim/L0/tests/assets/golden/sleep

It refuses any other interpreter, as gen_census_golden.py does: the PyPI wheel
fuses multiply-adds.

It writes rk4_sleep.json. Under RK4, MuJoCo wakes a sleeping tree that nothing
touched, in two ways (registry D-RK4-SLEEP):

1. mj_RungeKutta resets the state to the step's start and calls mj_advance,
   whose sleep step evaluates the forward pass again from the velocity stage
   (engine_forward.c:899-905): a tree put to sleep keeps the last stage's
   frames while its qpos is the step's start, and the next step's kinematics
   find them changed (engine_core_smooth.c:163-179) and wake it.
2. The stages integrate every dof (engine_forward.c:1086-1090), a sleeping
   dof with the stale acceleration it keeps, so the stage's forward pass
   finds a sleeping tree moving and wakes it (engine_sleep.c:124-153).

For each fixture below it steps MuJoCo `nstep` times twice. `mujoco`: as it
is, the number of steps at which any tree's sleep changes (`transitions`),
the first four of them, and the steps after which a sleeping tree's stored
xpos or xquat differ from the kinematics of its qpos (`stale_frames`).
`intervened`: after every step, each body of a sleeping tree takes the frames
the kinematics give its qpos (xpos, xquat, xmat, xipos, ximat, and its geoms'
and sites' xpos and xmat, from a second MjData of the model with sleep
disabled), and the tree's dofs' qacc_smooth is set to 0: tree_asleep at each
step where it changes (`changes`: [step, tree_asleep]), and qpos, qvel,
sensordata and act after every 50th step and the steps within 2 of a change
(`samples`). It refuses a fixture whose intervened run does not sleep exactly
once and never wake, or whose plain run does not wake again.

Fixtures, all RK4 with sleep enabled:
- box: a box resting on the plane, with framelinvel and framepos sensors;
- actuated_hinge: a damped hinge with a filtered affine actuator, an
  actuator-force sensor, an accelerometer and a joint-velocity sensor;
- chain: two damped hinges in one tree, released from 60 degrees.

`two_trees` holds, for the box beside a hinged pendulum that never sleeps
(a velocimeter, framelinvel and an accelerometer at the pendulum's tip), the
step at which the box first sleeps and MuJoCo's sensordata after that step,
with the box allowed to sleep (`xml_sleeps`, `sleeps`) and with it never
sleeping (`xml_never`, `never`).

`stage_wake` holds a box asleep from the start and a sphere falling onto it,
and the step after which MuJoCo's box is first awake (`wake`).
"""

import json
import os
import sys

import mujoco
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from gen_census_golden import finite_or_string, oracle  # noqa: E402

RK4 = '<option timestep="0.002" integrator="RK4"><flag sleep="enable"/></option>'
PLANE = '<geom type="plane" size="5 5 0.1"/>'
BOX = '<body name="b" pos="0 0 0.0995"{policy}><freejoint/><geom type="box" size="0.1 0.1 0.1" mass="1"/></body>'

FIXTURES = {
    "box": (f"""<mujoco>{RK4}
<worldbody>{PLANE}
{BOX.format(policy="")}</worldbody>
<sensor><framelinvel objtype="body" objname="b"/><framepos objtype="body" objname="b"/></sensor></mujoco>""",
            3000),
    "actuated_hinge": (f"""<mujoco>{RK4}
<worldbody>
<body name="l1" pos="0 0 1" sleep="allowed"><joint name="j1" type="hinge" axis="0 1 0" damping="3"/>
  <geom type="capsule" fromto="0 0 0 0.3 0 0" size="0.03" mass="1"/><site name="s1" pos="0.3 0 0"/></body>
</worldbody>
<actuator><general name="a1" joint="j1" dyntype="filter" dynprm="0.05" gainprm="1" biastype="affine"
  biasprm="0 -20 -1"/></actuator>
<sensor><actuatorfrc actuator="a1"/><accelerometer site="s1"/><jointvel joint="j1"/></sensor></mujoco>""",
                       1500),
    "chain": (f"""<mujoco>{RK4}
<worldbody>
<body name="l1" pos="0 0 1" euler="0 60 0"><joint name="j1" type="hinge" axis="0 1 0" damping="0.5"/>
  <geom type="capsule" fromto="0 0 0 0 0 -0.3" size="0.03" mass="1"/>
  <body name="l2" pos="0 0 -0.3"><joint name="j2" type="hinge" axis="0 1 0" damping="0.5"/>
    <geom type="capsule" fromto="0 0 0 0 0 -0.3" size="0.03" mass="1"/></body></body>
</worldbody></mujoco>""",
              4500),
}

TWO_TREES = f"""<mujoco>{RK4}
<worldbody>{PLANE}
{BOX}
<body name="p" pos="2 0 1" sleep="never"><joint name="jp" axis="0 1 0"/>
  <geom type="capsule" fromto="0 0 0 0.3 0 0" size="0.03" mass="1"/><site name="tip" pos="0.3 0 0"/></body>
</worldbody>
<sensor><velocimeter site="tip"/><framelinvel objtype="site" objname="tip"/><accelerometer site="tip"/></sensor>
</mujoco>"""


STAGE_WAKE = f"""<mujoco>{RK4}
<worldbody>{PLANE}
{BOX.format(policy=' sleep="init"')}
<body name="s" pos="0 0 0.5"><freejoint/><geom type="sphere" size="0.1" mass="1"/></body>
</worldbody></mujoco>"""


def ints(a):
    return [int(x) for x in np.asarray(a).ravel()]


def floats(a):
    return [float(x) for x in np.asarray(a).ravel()]


def fk_model(xml):
    m = mujoco.MjModel.from_xml_string(xml)
    m.opt.enableflags &= ~int(mujoco.mjtEnableBit.mjENBL_SLEEP)
    return m, mujoco.MjData(m)


def stale(m, d, d2, bodies):
    return any(not (np.array_equal(d.xpos[b], d2.xpos[b]) and np.array_equal(d.xquat[b], d2.xquat[b]))
               for b in bodies)


def intervene(m, d, d2, bodies, t):
    for b in bodies:
        for f in ("xpos", "xquat", "xmat", "xipos", "ximat"):
            getattr(d, f)[b] = getattr(d2, f)[b]
    for g in range(m.ngeom):
        if m.geom_bodyid[g] in bodies:
            d.geom_xpos[g] = d2.geom_xpos[g]
            d.geom_xmat[g] = d2.geom_xmat[g]
    for s in range(m.nsite):
        if m.site_bodyid[s] in bodies:
            d.site_xpos[s] = d2.site_xpos[s]
            d.site_xmat[s] = d2.site_xmat[s]
    a0, na = m.tree_dofadr[t], m.tree_dofnum[t]
    d.qacc_smooth[a0:a0 + na] = 0


def run(xml, nstep, intervened):
    m = mujoco.MjModel.from_xml_string(xml)
    d = mujoco.MjData(m)
    m2, d2 = fk_model(xml)
    asleep, changes, stale_frames, rows = [], [], 0, []
    prev = ints(d.tree_asleep)
    for k in range(nstep):
        mujoco.mj_step(m, d)
        d2.qpos[:] = d.qpos
        mujoco.mj_kinematics(m2, d2)
        frames_stale = False
        for t in range(m.ntree):
            if d.tree_asleep[t] < 0:
                continue
            bodies = range(m.tree_bodyadr[t], m.tree_bodyadr[t] + m.tree_bodynum[t])
            frames_stale |= stale(m, d, d2, bodies)
            if intervened:
                intervene(m, d, d2, bodies, t)
        stale_frames += frames_stale
        now = ints(d.tree_asleep)
        if now != prev:
            changes.append([k, now])
        prev = now
        asleep.append(bool((d.tree_asleep >= 0).any()))
        rows.append({"qpos": floats(d.qpos), "qvel": floats(d.qvel), "sensordata": floats(d.sensordata),
                     "act": floats(d.act)})
    transitions = [k for k in range(nstep) if asleep[k] != (asleep[k - 1] if k else False)]
    return transitions, stale_frames, changes, rows


def fixture(name, xml, nstep):
    transitions, stale_frames, _, _ = run(xml, nstep, False)
    fixed, _, changes, rows = run(xml, nstep, True)
    if len(fixed) != 1 or len(transitions) < 3:
        sys.exit(f"{name}: intervened transitions {fixed[:4]}, plain {transitions[:4]}")
    near = {c[0] + j for c in changes for j in range(-2, 3)}
    keep = sorted(k for k in range(nstep) if k % 50 == 49 or k in near)
    return {"name": name, "xml": xml, "nstep": nstep,
            "mujoco": {"transitions": len(transitions), "first": transitions[:4], "stale_frames": stale_frames},
            "intervened": {"changes": changes,
                           "samples": [dict(step=k, **rows[k]) for k in keep]}}


def two_trees():
    out = {}
    for policy, key in (("", "sleeps"), (' sleep="never"', "never")):
        xml = TWO_TREES.replace("{policy}", policy)
        out[f"xml_{key}"] = xml
        m = mujoco.MjModel.from_xml_string(xml)
        d = mujoco.MjData(m)
        for k in range(300):
            mujoco.mj_step(m, d)
            if key == "sleeps" and d.tree_asleep[0] >= 0 and "step" not in out:
                out["step"] = k
            if k == out.get("step"):
                out[key] = floats(d.sensordata)
    if "never" not in out:
        sys.exit("two_trees: the box never slept")
    return out


def stage_wake():
    m = mujoco.MjModel.from_xml_string(STAGE_WAKE)
    d = mujoco.MjData(m)
    if d.tree_asleep[0] < 0:
        sys.exit("stage_wake: the box starts awake")
    for k in range(300):
        mujoco.mj_step(m, d)
        if d.tree_asleep[0] < 0:
            return {"xml": STAGE_WAKE, "wake": k}
    sys.exit("stage_wake: the box never woke")


def write(path, doc):
    with open(path, "w", encoding="utf-8") as f:
        json.dump(doc, f, separators=(",", ":"))
        f.write("\n")


def main():
    if len(sys.argv) != 2:
        sys.exit(__doc__)
    marker = oracle()
    doc = {"oracle": marker,
           "fixtures": finite_or_string([fixture(name, *f) for name, f in FIXTURES.items()]),
           "two_trees": finite_or_string(two_trees()),
           "stage_wake": stage_wake()}
    write(os.path.join(sys.argv[1], "rk4_sleep.json"), doc)
    print(f"{len(FIXTURES)} fixtures -> {sys.argv[1]}")


if __name__ == "__main__":
    main()
