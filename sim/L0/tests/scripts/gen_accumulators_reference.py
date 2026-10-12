#!/usr/bin/env python3
"""Generate the body-accumulator golden data from the unfused MuJoCo 3.5.0 oracle.

Run it with the oracle's own interpreter (build_mujoco_oracle.sh builds it):

    <workdir>/venv/bin/python -I sim/L0/tests/scripts/gen_accumulators_reference.py \\
        sim/L0/tests/assets/golden/accumulators

It refuses any other interpreter, as gen_census_golden.py does.

MuJoCo's `mj_rnePostConstraint` (`engine_core_smooth.c:2477-2681`) fills
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
  the body's origin. (On a slide alone the contact force differs: MuJoCo's
  body_invweight0 for a body with no rotational dof is not sim-core's,
  ledger L112.)
- applied_wrench: a body on a ball joint, its centre of mass off its
  origin, with an xfrc_applied wrench; force and torque at its origin.
- world_wrench: applied_wrench with an xfrc_applied wrench on the world body
  too, which MuJoCo leaves out of cfrc_ext (its loop starts at body 1).
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


def state(xml, qpos=None, qvel=None, xfrc=None, steps=()):
    m = mujoco.MjModel.from_xml_string(xml)
    d = mujoco.MjData(m)
    if qpos is not None:
        for i, v in qpos.items():
            d.qpos[i] = v
    if qvel is not None:
        d.qvel[:] = qvel
    for b, w in (xfrc or {}).items():
        d.xfrc_applied[b] = w
    mujoco.mj_forward(m, d)
    mujoco.mj_rnePostConstraint(m, d)
    stable(d, "state")
    out = {"xml": xml, "qpos": floats(d.qpos), "qvel": floats(d.qvel),
           "xfrc_applied": {str(b): w for b, w in (xfrc or {}).items()},
           "sensordata": floats(d.sensordata), "cfrc_ext": cfrc_ext_at_origin(m, d)}
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
    }
    doc = {"oracle": marker, "cases": finite_or_string(cases)}
    with open(os.path.join(sys.argv[1], "accumulators.json"), "w", encoding="utf-8") as f:
        json.dump(doc, f, separators=(",", ":"))
        f.write("\n")
    print(f"{len(cases)} cases -> {sys.argv[1]}")


if __name__ == "__main__":
    main()
