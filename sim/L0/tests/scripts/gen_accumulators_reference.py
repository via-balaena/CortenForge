#!/usr/bin/env python3
"""Generate the body-accumulator golden data from the unfused MuJoCo 3.5.0 oracle.

Run it with the oracle's own interpreter (build_mujoco_oracle.sh builds it):

    <workdir>/venv/bin/python -I sim/L0/tests/scripts/gen_accumulators_reference.py \\
        sim/L0/tests/assets/golden/accumulators

It refuses any other interpreter, as gen_census_golden.py does.

MuJoCo's `mj_rnePostConstraint` (`engine_core_smooth.c:2492-2683`) fills
cfrc_ext (applied wrenches, contacts, connects and welds), cacc and cfrc_int,
which the acceleration-stage sensors read. Each case writes its XML, a state,
and after mj_forward and mj_rnePostConstraint the sensordata and every
body's cfrc_ext, moved from MuJoCo's frame (at the subtree centre of mass of
the body's root) to the body's origin xpos, where sim-core keeps it:

- free_box: a rotated free box (geom off its origin) with a hinged child,
  accelerometer, framelinacc (site, body, xbody), frameangacc, force and
  torque on a box site and accelerometer, force and torque on a child site;
  at qpos0 with qvel (0.1, -0.2, 0.3, -0.4, 0.5, -0.1, 0.7), and the
  sensordata after steps 1, 10 and 100.
- connect_force: a hinged pendulum whose tip is connected to the world,
  turned 0.3 rad; force and torque at a site on it.
- weld_force: a box on a hinge welded to the world at its own pose (no
  relpose: sim-mjcf ignores it until Rigid-loading L41), turned 0.3 rad; force and torque at a site on it (a weld's
  force and torque act at its anchors, the other way round from a
  connect's).
- contact_torque: a free, frictionless sphere (condim 1 on both geoms:
  MuJoCo takes the larger; geom off its body's origin, the centre of mass
  elsewhere) 5 mm into a plane: cfrc_ext holds the contact's torque about
  the body's origin.
- applied_wrench: a body on a ball joint, its centre of mass off its
  origin, with an xfrc_applied wrench; force and torque at its origin.
- world_wrench: applied_wrench with an xfrc_applied wrench on the world body
  too, which MuJoCo leaves out of cfrc_ext (its loop starts at body 1).
- contact_c{3,4,6}_{pyramidal,elliptic}: a free sphere (geom and centre of
  mass off its origin) 2 mm into a plane, spinning and rolling, with an
  applied wrench, under each cone and condim 3, 4 and 6; it also records
  MuJoCo's contacts (pos, frame, dim), efc_force and qacc, which the test
  writes into sim-core's Data so that cfrc_ext reads MuJoCo's solve.
- contact_two_bodies: the same, a sphere on a larger sphere held by a
  vertical slide (condim 4, elliptic), so that both of the contact's bodies
  move; the centres are on one vertical, so the contact frame is the plane
  cases' (a normal off the axes takes another frame here, which P42 ports).
- connect_two_bodies, weld_two_bodies: a connect and a weld between two
  moving bodies (a hinged body and a hinge-and-slide one; a hinged body and
  a free one), out of their constraint, with force and torque sensors on
  both, and the sensordata after steps 1, 10 and 100.
"""
import json
import os
import sys

import mujoco
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from gen_census_golden import finite_or_string, oracle  # noqa: E402

FREE_BOX = """<mujoco model="free_box">
  <option timestep="0.002"/>
  <worldbody>
    <body name="box" pos="0 0 1" quat="0.95 0.15 -0.1 0.24">
      <freejoint/>
      <geom type="box" size="0.1 0.06 0.04" pos="0.02 0.01 0" mass="2" contype="0" conaffinity="0"/>
      <site name="s_box" pos="0.08 -0.03 0.02"/>
      <body name="child" pos="0.15 0 0">
        <joint name="h" type="hinge" axis="0 1 0" damping="0.1"/>
        <geom type="capsule" fromto="0 0 0 0.2 0 0" size="0.02" mass="0.5" contype="0" conaffinity="0"/>
        <site name="s_child" pos="0.2 0 0"/>
      </body>
    </body>
  </worldbody>
  <sensor>
    <accelerometer site="s_box"/>
    <framelinacc objtype="site" objname="s_box"/>
    <framelinacc objtype="body" objname="box"/>
    <framelinacc objtype="xbody" objname="box"/>
    <frameangacc objtype="site" objname="s_box"/>
    <force site="s_box"/>
    <torque site="s_box"/>
    <accelerometer site="s_child"/>
    <force site="s_child"/>
    <torque site="s_child"/>
  </sensor>
</mujoco>"""

CONNECT_FORCE = """<mujoco model="connect_force">
  <worldbody>
    <body name="p" pos="0 0 1">
      <joint name="h" type="hinge" axis="0 1 0"/>
      <geom type="capsule" fromto="0 0 0 0.3 0 0" size="0.02" mass="1" contype="0" conaffinity="0"/>
      <site name="s" pos="0.1 0 0.01"/>
    </body>
  </worldbody>
  <equality>
    <connect body1="p" anchor="0.3 0 0"/>
  </equality>
  <sensor>
    <force site="s"/>
    <torque site="s"/>
  </sensor>
</mujoco>"""

WELD_FORCE = """<mujoco model="weld_force">
  <worldbody>
    <body name="w" pos="0 0 1">
      <joint name="h" type="hinge" axis="0.2 0.3 1"/>
      <geom type="box" size="0.1 0.05 0.03" pos="0.15 0.05 0" mass="1" contype="0" conaffinity="0"/>
      <site name="s" pos="0.05 -0.02 0.01"/>
    </body>
  </worldbody>
  <equality>
    <weld body1="w"/>
  </equality>
  <sensor>
    <force site="s"/>
    <torque site="s"/>
  </sensor>
</mujoco>"""

CONTACT_TORQUE = """<mujoco model="contact_torque">
  <worldbody>
    <geom type="plane" size="1 1 0.1" condim="1"/>
    <body name="ball" pos="0 0 0.095">
      <freejoint/>
      <geom type="sphere" size="0.1" pos="0.02 0 0" condim="1"/>
      <inertial pos="0.03 0 0.02" mass="1" diaginertia="0.004 0.005 0.006"/>
      <site name="s" pos="0 0 0"/>
    </body>
  </worldbody>
  <sensor>
    <force site="s"/>
    <torque site="s"/>
  </sensor>
</mujoco>"""

APPLIED = """<mujoco model="applied_wrench">
  <option gravity="0 0 -9.81"/>
  <worldbody>
    <body name="b" pos="0 0 1" quat="0.9 0.2 0.1 -0.3">
      <joint name="ball" type="ball"/>
      <geom type="box" size="0.08 0.05 0.03" contype="0" conaffinity="0"/>
      <inertial pos="0.04 -0.02 0.03" mass="1.5" diaginertia="0.003 0.004 0.005"/>
      <site name="s" pos="0 0 0"/>
    </body>
  </worldbody>
  <sensor>
    <force site="s"/>
    <torque site="s"/>
  </sensor>
</mujoco>"""

def contact_on_plane(condim, cone):
    return f"""<mujoco model="contact_c{condim}_{cone}">
  <option cone="{cone}"/>
  <worldbody>
    <geom type="plane" size="2 2 0.1" condim="{condim}" friction="0.8 0.05 0.02"/>
    <body name="ball" pos="0 0 0.098">
      <freejoint/>
      <geom type="sphere" size="0.1" pos="0.01 0 0" condim="{condim}" friction="0.8 0.05 0.02"/>
      <inertial pos="0.02 -0.01 0.01" mass="1.2" diaginertia="0.01 0.012 0.014"/>
      <site name="s" pos="0.03 0.02 -0.01"/>
    </body>
  </worldbody>
  <sensor>
    <force site="s"/>
    <torque site="s"/>
  </sensor>
</mujoco>"""


CONTACT_TWO = """<mujoco model="contact_two_bodies">
  <option cone="elliptic"/>
  <worldbody>
    <body name="base" pos="0 0 0.5">
      <joint type="slide" axis="0 0 1"/>
      <geom type="sphere" size="0.3" condim="4" friction="0.8 0.05 0.02"/>
      <site name="sb" pos="0.1 0.05 0"/>
    </body>
    <body name="ball" pos="0 0 0.898" quat="0.95 0.1 0.2 0.1">
      <freejoint/>
      <geom type="sphere" size="0.1" condim="4" friction="0.8 0.05 0.02"/>
      <inertial pos="0.02 -0.01 0.01" mass="1.2" diaginertia="0.01 0.012 0.014"/>
      <site name="s" pos="0.03 0.02 -0.01"/>
    </body>
  </worldbody>
  <sensor>
    <force site="s"/>
    <torque site="s"/>
    <force site="sb"/>
    <torque site="sb"/>
  </sensor>
</mujoco>"""


CONNECT_TWO = """<mujoco model="connect_two_bodies">
  <option timestep="0.002"/>
  <worldbody>
    <body name="a" pos="0 0 1">
      <joint name="ha" type="hinge" axis="0 1 0"/>
      <geom type="capsule" fromto="0 0 0 0.3 0 0" size="0.02" mass="1" contype="0" conaffinity="0"/>
      <site name="sa" pos="0.1 0 0.01"/>
    </body>
    <body name="b" pos="0.6 0 1" quat="0.95 0.1 0.2 0.1">
      <joint name="hb" type="hinge" axis="1 0 1"/>
      <joint name="sb" type="slide" axis="0 0 1"/>
      <geom type="capsule" fromto="0 0.01 0 -0.3 0.01 0" size="0.02" mass="0.7" contype="0" conaffinity="0"/>
      <site name="sb" pos="-0.1 0.01 0"/>
    </body>
  </worldbody>
  <equality>
    <connect body1="a" body2="b" anchor="0.3 0 0"/>
  </equality>
  <sensor>
    <force site="sa"/>
    <torque site="sa"/>
    <force site="sb"/>
    <torque site="sb"/>
  </sensor>
</mujoco>"""

WELD_TWO = """<mujoco model="weld_two_bodies">
  <option timestep="0.002"/>
  <worldbody>
    <body name="a" pos="0 0 1">
      <joint name="ha" type="hinge" axis="0 1 0"/>
      <geom type="box" size="0.1 0.05 0.03" pos="0.1 0 0" mass="1" contype="0" conaffinity="0"/>
      <site name="sa" pos="0.1 0 0.01"/>
    </body>
    <body name="b" pos="0.5 0.1 1.1" quat="0.95 0.1 0.2 0.1">
      <freejoint/>
      <geom type="box" size="0.05 0.05 0.05" pos="0.02 0.01 0" mass="0.7" contype="0" conaffinity="0"/>
      <site name="sb" pos="-0.03 0.01 0"/>
    </body>
  </worldbody>
  <equality>
    <weld body1="a" body2="b" anchor="0.2 0.05 -0.02" solimp="0.85 0.95 0.02 0.3 2.5"/>
  </equality>
  <sensor>
    <force site="sa"/>
    <torque site="sa"/>
    <force site="sb"/>
    <torque site="sb"/>
  </sensor>
</mujoco>"""

WRENCH = [0.3, -0.7, 1.1, 0.05, 0.02, -0.04]          # force, torque
WORLD_WRENCH = [2.0, 3.0, -1.0, 0.5, -0.4, 0.3]


def floats(a):
    return [float(x) for x in np.asarray(a).ravel()]


def stable(d, what):
    for w in (mujoco.mjtWarning.mjWARN_BADQACC, mujoco.mjtWarning.mjWARN_BADQVEL,
              mujoco.mjtWarning.mjWARN_BADQPOS):
        if d.warning[w].number:
            sys.exit(f"{what}: MuJoCo warned {w.name}")


def cfrc_ext_at_origin(m, d):
    """cfrc_ext ([torque; force]) moved from the subtree COM to each body's xpos."""
    out = []
    for b in range(m.nbody):
        c = d.subtree_com[m.body_rootid[b]]
        tq, f = d.cfrc_ext[b, :3], d.cfrc_ext[b, 3:]
        out.append(floats(np.concatenate([tq - np.cross(d.xpos[b] - c, f), f])))
    return out


def state(xml, qpos=None, qvel=None, xfrc=None, steps=(), solve=False):
    m = mujoco.MjModel.from_xml_string(xml)
    d = mujoco.MjData(m)
    if qpos is not None:
        for i, v in (qpos.items() if isinstance(qpos, dict) else ()):
            d.qpos[i] = v
    if qvel is not None:
        d.qvel[:] = qvel
    if qpos is not None and isinstance(qpos, list):
        d.qpos[:] = qpos
    for b, w in (xfrc or {}).items():
        d.xfrc_applied[b] = w
    mujoco.mj_forward(m, d)
    mujoco.mj_rnePostConstraint(m, d)
    stable(d, "state")
    out = {"xml": xml, "qpos": floats(d.qpos), "qvel": floats(d.qvel),
           "xfrc_applied": {str(b): w for b, w in (xfrc or {}).items()},
           "sensordata": floats(d.sensordata), "cfrc_ext": cfrc_ext_at_origin(m, d)}
    if solve:
        if d.ncon == 0:
            sys.exit("solve: no contact")
        out["contacts"] = [{"pos": floats(c.pos), "frame": floats(c.frame), "dim": int(c.dim),
                            "efc_address": int(c.efc_address)} for c in d.contact[:d.ncon]]
        out["efc_force"] = floats(d.efc_force)
        out["qacc"] = floats(d.qacc)
    if steps:
        traj = {}
        for step in range(1, max(steps) + 1):
            mujoco.mj_step(m, d)
            if step in steps:
                traj[str(step)] = {"qpos": floats(d.qpos), "qvel": floats(d.qvel),
                                   "sensordata": floats(d.sensordata)}
        stable(d, "trajectory")
        out["traj"] = traj
    return out


def main():
    if len(sys.argv) != 2:
        sys.exit(__doc__)
    marker = oracle()
    cases = {
        "free_box": state(FREE_BOX, qvel=[0.1, -0.2, 0.3, -0.4, 0.5, -0.1, 0.7], steps=(1, 10, 100)),
        "connect_force": state(CONNECT_FORCE, qpos={0: 0.3}),
        "weld_force": state(WELD_FORCE, qpos={0: 0.3}),
        "contact_torque": state(CONTACT_TORQUE),
        "applied_wrench": state(APPLIED, xfrc={1: WRENCH}),
        "world_wrench": state(APPLIED, xfrc={0: WORLD_WRENCH, 1: WRENCH}),
        "connect_two_bodies": state(CONNECT_TWO, qpos=[0.2, -0.3, 0.05], qvel=[0.3, -0.5, 0.2],
                                    steps=(1, 10, 100)),
        "weld_two_bodies": state(WELD_TWO, qpos=[0.05, 0.51, 0.09, 1.12, 0.94, 0.12, 0.21, 0.12],
                                 qvel=[0.3, 0.1, -0.2, 0.3, 0.4, -0.1, 0.2], steps=(1, 10, 100)),
    }
    for condim in (3, 4, 6):
        for cone in ("pyramidal", "elliptic"):
            cases[f"contact_c{condim}_{cone}"] = state(
                contact_on_plane(condim, cone), qvel=[0.2, -0.1, 0.0, 0.3, -0.2, 1.5],
                xfrc={1: [1.0, 0.5, 0.0, 0.02, -0.01, 0.3]}, solve=True)
    cases["contact_two_bodies"] = state(CONTACT_TWO, qvel=[0.1, 0.2, -0.1, 0.0, 0.3, -0.2, 1.5],
                                        xfrc={2: [1.0, 0.5, 0.0, 0.02, -0.01, 0.3]}, solve=True)
    doc = {"oracle": marker, "cases": finite_or_string(cases)}
    with open(os.path.join(sys.argv[1], "accumulators.json"), "w", encoding="utf-8") as f:
        json.dump(doc, f, separators=(",", ":"))
        f.write("\n")
    print(f"{len(cases)} cases -> {sys.argv[1]}")


if __name__ == "__main__":
    main()
