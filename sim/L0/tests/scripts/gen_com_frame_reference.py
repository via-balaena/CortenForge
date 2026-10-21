#!/usr/bin/env python3
"""Generate the centre-of-mass frame golden data from the unfused MuJoCo 3.5.0 oracle.

Run it with the oracle's own interpreter (build_mujoco_oracle.sh builds it):

    <workdir>/venv/bin/python -I sim/L0/tests/scripts/gen_com_frame_reference.py \\
        sim/L0/tests/assets/golden/com_frame

It refuses any other interpreter, as gen_census_golden.py does.

MuJoCo takes a body's spatial quantities about its tree's subtree centre of
mass, `subtree_com[body_rootid[b]]`, in the world's orientation, `[angular;
linear]`: the dofs' motion axes `cdof` and the body inertias `cinert` (a
10-vector, `mju_inertCom`) in `mj_comPos` (`engine_core_smooth.c:245-350`),
the composite inertias `crb` and the mass matrix in `mj_crb` (`:1811-1885`),
the body velocities `cvel` and the axes' rates `cdof_dot` in `mj_comVel`
(`:2276-2343`), the bias force in `mj_rne` (`:2425-2488`), and the body
accelerations and forces `cacc`, `cfrc_int`, `cfrc_ext` in
`mj_rnePostConstraint` (`:2491-2683`); `dof_M0`, a simple dof's mass-matrix
diagonal, comes from `mj_setM0` at `qpos0` (`engine_setconst.c:36-59`). Read
through them: each body's velocity at its centre of mass, in the world's
orientation and in the inertial frame's (`mj_objectVelocity`,
`engine_core_util.c:720-774`), and the subtree velocities and angular
momenta (`mj_subtreeVel`, `engine_core_smooth.c:2346-2420`).

Each case records, after `mj_forward` at a moved state and then
`mj_rnePostConstraint`, what those functions read (the kinematics: `xpos`,
`xmat`, `xipos`, `ximat`, `xanchor`, `xaxis`, `subtree_com`) and what they
write, so a test can check that its inputs are MuJoCo's before it compares
the outputs, the compiled masses and inertias among them. Contacts are
disabled throughout: the bodies' forces here are the dynamics' own. Each
body's mass and inertia are given (`<inertial>`), not computed from geoms,
and the joint axes are axis-aligned: sim-mjcf computes a geom's moments and
a rotated geom's inertial frame, and normalises a tilted axis, otherwise than
MuJoCo (ledger L198, L195, L146). The cases:

- compos: two trees. One hangs a hinge with an offset anchor, a ball with an
  offset anchor under it, then a body with a slide and a hinge. The other is
  a free body carrying a hinged body, which carries a jointless body with
  mass; two subtree sensors make the forward pass compute the subtree
  velocities. There is no massless body: the loader places its inertial frame
  otherwise than MuJoCo, and refuses MuJoCo's way of giving it one (ledger
  L212).
- chain: a hinge-hinge-ball chain with a hinge off its root (shaped as the
  mass factor golden's chain).
- slow: compos at velocities 1e-7 of compos's.
- nogravity: compos with gravity disabled.
- simple: bodies MuJoCo marks simple (a hinge, two slides, a free joint, each
  centred and childless) beside one it does not (an off-centre hinge).
- mocap: a mocap body, a static body and a hinged body.
- sleeping: two double pendulums, the first asleep from the start
  (`sleep="init"`), the second moved. A sleeping body keeps what MuJoCo last
  computed for it; the awake bodies are not the first ones.
"""
import json
import os
import sys

import mujoco
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from gen_census_golden import finite_or_string, oracle  # noqa: E402

COMPOS = """<mujoco model="compos">
  <option timestep="0.002">
    <flag contact="disable"{gravity}/>
  </option>
  <worldbody>
    <body name="a" pos="0.1 0 1">
      <joint type="hinge" axis="0 1 0" pos="0 0 0.05"/>
      <inertial pos="0.01 0 -0.15" mass="1" diaginertia="0.0078 0.0078 0.0006"/>
      <body name="b" pos="0 0 -0.3">
        <joint type="ball" pos="0.01 0.02 0"/>
        <inertial pos="0.02 0 -0.12" mass="0.7" diaginertia="0.00373 0.00357 0.00058"/>
        <body name="c" pos="0 0.05 -0.25">
          <joint type="slide" axis="1 0 0"/>
          <joint type="hinge" axis="0 0 1" pos="0.02 0 0"/>
          <inertial pos="0.03 0.01 -0.08" mass="0.4" diaginertia="0.00097 0.00119 0.00045"/>
        </body>
      </body>
    </body>
    <body name="f" pos="0.5 0.2 0.8">
      <freejoint/>
      <inertial pos="0.02 -0.01 0.03" mass="1.3" diaginertia="0.00147 0.00472 0.00542"/>
      <body name="g" pos="0.1 0 0">
        <joint type="hinge" axis="0 1 0"/>
        <inertial pos="0.05 0 0" mass="0.3" diaginertia="0.0000625 0.00102 0.00104"/>
        <body name="w" pos="0.2 0 0">
          <inertial pos="0 0 0" mass="0.1" diaginertia="0.000016 0.000016 0.000016"/>
        </body>
      </body>
    </body>
  </worldbody>
  <sensor>
    <subtreelinvel body="a"/>
    <subtreeangmom body="f"/>
  </sensor>
</mujoco>"""

CHAIN = """<mujoco model="chain">
  <option timestep="0.005" integrator="Euler">
    <flag contact="disable"/>
  </option>
  <worldbody>
    <body name="a" pos="0 0 1">
      <joint type="hinge" axis="0 1 0"/>
      <inertial pos="0 0 -0.15" mass="1" diaginertia="0.0078 0.0078 0.0006"/>
      <body name="b" pos="0 0 -0.3">
        <joint type="hinge" axis="1 0 0"/>
        <inertial pos="0.02 0 -0.12" mass="0.7" diaginertia="0.0037 0.0036 0.0006"/>
        <body name="c" pos="0 0 -0.25">
          <joint type="ball"/>
          <inertial pos="0.03 0.01 -0.08" mass="0.4" diaginertia="0.00097 0.00119 0.00045"/>
        </body>
      </body>
      <body name="d" pos="0.2 0 0">
        <joint type="hinge" axis="0 0 1"/>
        <inertial pos="0.1 0 0" mass="0.5" diaginertia="0.0003 0.0017 0.0017"/>
      </body>
    </body>
  </worldbody>
</mujoco>"""

SIMPLE = """<mujoco model="simple">
  <option>
    <flag contact="disable"/>
  </option>
  <worldbody>
    <body name="hinge" pos="0 0 1">
      <joint type="hinge" axis="0 1 0" armature="0.05"/>
      <inertial pos="0 0 0" mass="2" diaginertia="0.1 0.2 0.3"/>
    </body>
    <body name="slides" pos="1 0 1">
      <joint type="slide" axis="1 0 0" armature="0.1"/>
      <joint type="slide" axis="0 0 1"/>
      <inertial pos="0 0 0" mass="1" diaginertia="0.1 0.1 0.1"/>
    </body>
    <body name="free" pos="0 1 1">
      <freejoint/>
      <inertial pos="0 0 0" mass="0.5" diaginertia="0.01 0.02 0.03"/>
    </body>
    <body name="offset" pos="0 2 1">
      <joint type="hinge" axis="1 0 0"/>
      <inertial pos="0.05 0 0" mass="1" diaginertia="0.001 0.0035 0.0042"/>
    </body>
  </worldbody>
</mujoco>"""

MOCAP = """<mujoco model="mocap">
  <option>
    <flag contact="disable"/>
  </option>
  <worldbody>
    <body name="mocap" mocap="true" pos="0.3 0 1">
      <inertial pos="0 0 0" mass="1" diaginertia="0.0067 0.0067 0.0067"/>
    </body>
    <body name="static" pos="0 0 0.5">
      <inertial pos="0 0 0" mass="1" diaginertia="0.004 0.004 0.004"/>
    </body>
    <body name="pendulum" pos="0 0 1">
      <joint type="hinge" axis="0 1 0"/>
      <inertial pos="0.01 0 -0.2" mass="1" diaginertia="0.0134 0.0134 0.0003"/>
    </body>
  </worldbody>
</mujoco>"""

SLEEPING = """<mujoco model="sleeping">
  <option timestep="0.002" integrator="Euler">
    <flag contact="disable" sleep="enable"/>
  </option>
  <worldbody>
    <body name="resting" pos="1 0 1" sleep="init">
      <joint type="hinge" axis="0 1 0"/>
      <inertial pos="0 0 -0.2" mass="1" diaginertia="0.0134 0.0134 0.0003"/>
      <body pos="0 0 -0.4">
        <joint type="hinge" axis="0 1 0"/>
        <inertial pos="0.01 0 -0.1" mass="0.5" diaginertia="0.0018 0.0019 0.0002"/>
      </body>
    </body>
    <body name="moving" pos="0 0 1">
      <joint type="hinge" axis="0 1 0"/>
      <inertial pos="0 0 -0.2" mass="1" diaginertia="0.0134 0.0134 0.0003"/>
      <body pos="0 0 -0.4">
        <joint type="hinge" axis="1 0 0"/>
        <inertial pos="0 0 -0.1" mass="0.5" diaginertia="0.0017 0.0017 0.00013"/>
      </body>
    </body>
  </worldbody>
</mujoco>"""


def floats(a):
    return [float(x) for x in np.asarray(a).ravel()]


def ints(a):
    return [int(x) for x in np.asarray(a).ravel()]


def moved_state(m, seed, scale=1.0, trees=None):
    """A seeded state: each joint (of the bodies whose tree is in `trees`, every
    joint when it is None) turned or moved, normalised quaternions, velocities
    in (-2, 2) times `scale`."""
    rng = np.random.default_rng(seed)
    qpos = m.qpos0.copy()
    qvel = np.zeros(m.nv)
    for j in range(m.njnt):
        if trees is not None and m.body_treeid[m.jnt_bodyid[j]] not in trees:
            continue
        a, v = m.jnt_qposadr[j], m.jnt_dofadr[j]
        t = m.jnt_type[j]
        if t == mujoco.mjtJoint.mjJNT_FREE:
            qpos[a:a + 3] += rng.uniform(-0.2, 0.2, 3)
            q = rng.normal(size=4)
            qpos[a + 3:a + 7] = q / np.linalg.norm(q)
            qvel[v:v + 6] = rng.uniform(-2, 2, 6) * scale
        elif t == mujoco.mjtJoint.mjJNT_BALL:
            q = rng.normal(size=4)
            qpos[a:a + 4] = q / np.linalg.norm(q)
            qvel[v:v + 3] = rng.uniform(-2, 2, 3) * scale
        else:
            qpos[a] = rng.uniform(-0.8, 0.8)
            qvel[v] = rng.uniform(-2, 2) * scale
    return qpos, qvel


def record(xml, seed, scale=1.0, trees=None, simple=None):
    m = mujoco.MjModel.from_xml_string(xml)
    if simple is not None and ints(m.dof_simplenum) != simple:
        sys.exit(f"{m.names[:20]}: dof_simplenum {ints(m.dof_simplenum)}, meant {simple}")
    d = mujoco.MjData(m)
    qpos, qvel = moved_state(m, seed, scale, trees)
    if trees is None:
        d.qpos[:] = qpos
        d.qvel[:] = qvel
    else:
        for j in range(m.njnt):
            if m.body_treeid[m.jnt_bodyid[j]] in trees:
                a, v, t = m.jnt_qposadr[j], m.jnt_dofadr[j], m.jnt_type[j]
                nq = {0: 7, 1: 4}.get(int(t), 1)
                nv = {0: 6, 1: 3}.get(int(t), 1)
                d.qpos[a:a + nq] = qpos[a:a + nq]
                d.qvel[v:v + nv] = qvel[v:v + nv]
    mujoco.mj_forward(m, d)
    if d.ncon or d.nefc:
        sys.exit(f"{xml[:40]}: {d.ncon} contacts, {d.nefc} rows")
    mujoco.mj_rnePostConstraint(m, d)
    mujoco.mj_subtreeVel(m, d)
    objvel = np.zeros((m.nbody, 2, 6))
    for b in range(m.nbody):
        for local in (0, 1):
            mujoco.mj_objectVelocity(m, d, mujoco.mjtObj.mjOBJ_BODY, b, objvel[b, local], local)
    full = np.zeros((m.nv, m.nv))
    mujoco.mj_fullM(m, full, d.qM)
    return {
        "xml": xml, "qpos": floats(d.qpos), "qvel": floats(d.qvel),
        "body_rootid": ints(m.body_rootid), "dof_simplenum": ints(m.dof_simplenum),
        "body_mass": floats(m.body_mass), "body_inertia": floats(m.body_inertia),
        "body_ipos": floats(m.body_ipos), "body_iquat": floats(m.body_iquat),
        "body_subtreemass": floats(m.body_subtreemass),
        "body_awake": ints(d.body_awake),
        "xpos": floats(d.xpos), "xmat": floats(d.xmat), "xipos": floats(d.xipos),
        "ximat": floats(d.ximat),
        "xanchor": floats(d.xanchor), "xaxis": floats(d.xaxis),
        "subtree_com": floats(d.subtree_com),
        "cdof": floats(d.cdof), "cinert": floats(d.cinert), "crb": floats(d.crb),
        "qM": floats(full), "dof_M0": floats(m.dof_M0),
        "cvel": floats(d.cvel), "cdof_dot": floats(d.cdof_dot),
        "objvel": floats(objvel[:, 0]), "objvel_local": floats(objvel[:, 1]),
        "subtree_linvel": floats(d.subtree_linvel), "subtree_angmom": floats(d.subtree_angmom),
        "qfrc_bias": floats(d.qfrc_bias),
        "cacc": floats(d.cacc), "cfrc_int": floats(d.cfrc_int), "cfrc_ext": floats(d.cfrc_ext),
    }


def main():
    if len(sys.argv) != 2:
        sys.exit(__doc__)
    marker = oracle()
    cases = {
        "compos": record(COMPOS.format(gravity=""), 123),
        "chain": record(CHAIN, 5),
        "slow": record(COMPOS.format(gravity=""), 123, scale=1e-7),
        "nogravity": record(COMPOS.format(gravity=' gravity="disable"'), 123),
        "simple": record(SIMPLE, 7, simple=[9, 8, 7, 6, 5, 4, 3, 2, 1, 0]),
        "mocap": record(MOCAP, 11),
        "sleeping": record(SLEEPING, 13, trees=[1]),
    }
    if cases["sleeping"]["body_awake"] == [1] * len(cases["sleeping"]["body_awake"]):
        sys.exit("sleeping: no body asleep")
    doc = {"oracle": marker, "cases": finite_or_string(cases)}
    with open(os.path.join(sys.argv[1], "com_frame.json"), "w", encoding="utf-8") as f:
        json.dump(doc, f, separators=(",", ":"))
        f.write("\n")
    print(f"{len(cases)} cases -> {sys.argv[1]}")


if __name__ == "__main__":
    main()
