#!/usr/bin/env python3
"""Generate the forward-kinematics golden data from the unfused MuJoCo 3.5.0 oracle.

Run it with the oracle's own interpreter (build_mujoco_oracle.sh builds it):

    <workdir>/venv/bin/python -I sim/L0/tests/scripts/gen_kinematics_reference.py \\
        sim/L0/tests/assets/golden/kinematics

It refuses any other interpreter, as gen_census_golden.py does.

MuJoCo's `mj_kinematics1` (`engine_core_smooth.c:40-187`) moves a hinge or a
slide by `qpos - qpos0`, turns a hinge or a ball about its anchor, and writes
each joint's `xanchor` and `xaxis` in the frame before it turns. The cases:

- hinge_ref: a pendulum whose hinge has `ref="30"`; `xpos` and `xquat` at
  qpos0, and qpos and qvel after steps 1, 10 and 100.
- slide_ref: a body on a slide with `ref="0.3"` and a `pos`, under gravity
  and a spring; `xpos` and the slide's `xanchor` at qpos0, and the trajectory.
- offset_ball, chain_offball: a ball joint whose `pos` is off its body's
  origin, alone and as the child of a hinge, from a turned and moving state;
  the trajectory, and `mjd_transitionFD`'s A (centred, eps 1e-6) at the start.
- frames: hinge, slide, ball, free and mocap bodies, geoms and sites, all
  turned, at a state whose quaternions are not of unit length (a ball's of
  length 1.3, a free joint's within mjMINVAL of 1, which mju_normalize4 leaves
  as it is, a mocap body's of length 0.9, and body b's `body_quat`, set to
  length 1.1); the model's frame fields and every pose `mj_kinematics`
  writes.
- dof_length: the three models of ledger L79; `dof_length`.
- quat_springs: a ball and a free joint with stiffness and a ballquat sensor,
  `qpos_spring` set to quaternions not of unit length, at two states;
  `qfrc_spring` and the sensor (`engine_passive.c:155-161`,
  `engine_sensor.c:572-575`).
- frame_quats: framequat sensors on a site, a geom, a body and an xbody,
  alone and against a site and another geom as reference frames, on a
  turned free body at two states, one where the site's and the geom's
  quaternions have w < 0: MuJoCo's `get_xquat` multiplies the body's `xquat`
  by the object's local quaternion (`engine_sensor.c:122-144`) and a
  reference frame's conjugate by the object's (`:632-651`); the sensordata.
"""
import json
import os
import sys

import mujoco
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from gen_census_golden import finite_or_string, oracle  # noqa: E402

STEPS = (1, 10, 100)
MJMINVAL = 1e-15

HINGE_REF = """<mujoco model="hinge_ref">
  <option timestep="0.002"/>
  <worldbody>
    <body name="p" pos="0 0 1">
      <joint name="h" type="hinge" axis="0 1 0" ref="30" damping="0.05"/>
      <geom type="box" size="0.15 0.02 0.02" pos="0.15 0 0" mass="1" contype="0" conaffinity="0"/>
    </body>
  </worldbody>
</mujoco>"""

SLIDE_REF = """<mujoco model="slide_ref">
  <option timestep="0.002"/>
  <worldbody>
    <body name="s" pos="0.1 0 1" quat="0.9659258262890683 0 0.25881904510252074 0">
      <joint name="x" type="slide" axis="1 0 0" ref="0.3" pos="0.05 0.02 -0.03" stiffness="5"/>
      <geom type="sphere" size="0.05" mass="1" contype="0" conaffinity="0"/>
    </body>
  </worldbody>
</mujoco>"""

OFFSET_BALL = """<mujoco model="offset_ball">
  <option timestep="0.002"/>
  <worldbody>
    <body name="b" pos="0 0 1" quat="0.9 0.2 0.3 -0.1">
      <joint name="ball" type="ball" pos="0 0 0.2" damping="0.02"/>
      <geom type="box" size="0.1 0.06 0.04" mass="1" contype="0" conaffinity="0"/>
    </body>
  </worldbody>
</mujoco>"""

CHAIN_OFFBALL = """<mujoco model="chain_offball">
  <option timestep="0.002"/>
  <worldbody>
    <body name="a" pos="0 0 1" quat="0.95 0.1 -0.2 0.2">
      <joint name="h" type="hinge" axis="0 1 0" damping="0.05"/>
      <geom type="box" size="0.12 0.03 0.03" pos="0.12 0 0" mass="1" contype="0" conaffinity="0"/>
      <body name="c" pos="0.25 0 0" quat="0.9 0 0.3 0.1">
        <joint name="ball" type="ball" pos="-0.05 0.02 0.08" damping="0.02"/>
        <geom type="box" size="0.08 0.05 0.03" mass="0.5" contype="0" conaffinity="0"/>
      </body>
    </body>
  </worldbody>
</mujoco>"""

FRAMES = """<mujoco model="frames">
  <worldbody>
    <geom name="ground" type="box" size="0.5 0.5 0.01" pos="0.1 -0.2 0" quat="0.98 0 0.1 0.15"
          contype="0" conaffinity="0"/>
    <body name="a" pos="0.1 0.2 1" quat="0.95 0.1 -0.2 0.2">
      <joint name="h" type="hinge" axis="0.2 1 0.3" pos="0.03 -0.02 0.05" ref="12"/>
      <geom type="box" size="0.1 0.03 0.02" pos="0.1 0.01 0" quat="0.9 0.3 0 0.1" mass="1"
            contype="0" conaffinity="0"/>
      <site name="sa" pos="0.05 0.02 -0.01" quat="0.8 0.2 0.4 0.1"/>
      <body name="b" pos="0.2 0 0.05" quat="0.9 0 0.3 0.1">
        <joint name="s" type="slide" axis="1 0.4 -0.2" pos="0.01 0.02 0.03" ref="0.1"/>
        <joint name="ball" type="ball" pos="-0.05 0.02 0.08"/>
        <geom type="capsule" size="0.02 0.05" pos="0.03 0 0" quat="0.7 0.7 0 0" mass="0.5"
              contype="0" conaffinity="0"/>
        <site name="sb" pos="0.02 -0.01 0.04"/>
      </body>
    </body>
    <body name="f" pos="0.5 0 0.5">
      <freejoint/>
      <geom type="box" size="0.05 0.04 0.03" pos="0.01 0 0.02" quat="0.6 0 0.8 0" mass="0.3"
            contype="0" conaffinity="0"/>
      <site name="sf" pos="0 0.03 0" quat="0.5 0.5 0.5 0.5"/>
    </body>
    <body name="m" mocap="true" pos="-0.3 0.2 0.4" quat="0.8 0.1 0.5 0.3">
      <geom type="sphere" size="0.03" pos="0.02 0 0" contype="0" conaffinity="0"/>
    </body>
  </worldbody>
</mujoco>"""

# Ledger L79 (book P28): each body holds one sphere of size 0.05 and mass 1.
DOF_LENGTH = {
    "hinge_then_offset_slide": """<mujoco>
  <worldbody>
    <body pos="0 0 1">
      <joint type="hinge" axis="0 1 0"/>
      <geom type="sphere" size="0.05" mass="1"/>
      <body pos="0.2 0 0">
        <joint type="slide" axis="1 0 0" pos="0.5 0 0"/>
        <geom type="sphere" size="0.05" mass="1"/>
      </body>
    </body>
  </worldbody>
</mujoco>""",
    "offset_ball": """<mujoco>
  <worldbody>
    <body pos="0 0 1">
      <joint type="ball" pos="0 0 0.4"/>
      <geom type="sphere" size="0.05" mass="1"/>
    </body>
  </worldbody>
</mujoco>""",
    "slide_ref_then_hinge": """<mujoco>
  <worldbody>
    <body pos="0 0 1">
      <joint type="slide" axis="1 0 0" ref="0.3"/>
      <joint type="hinge" axis="0 1 0"/>
      <geom type="sphere" size="0.05" mass="1"/>
    </body>
  </worldbody>
</mujoco>""",
}

QUAT_SPRINGS = """<mujoco model="quat_springs">
  <option gravity="0 0 0"/>
  <worldbody>
    <body name="b" pos="0 0 1">
      <joint name="ball" type="ball" stiffness="3"/>
      <geom type="sphere" size="0.05" mass="1" contype="0" conaffinity="0"/>
    </body>
    <body name="f" pos="0.5 0 1">
      <joint name="free" type="free" stiffness="2"/>
      <geom type="box" size="0.05 0.04 0.03" mass="1" contype="0" conaffinity="0"/>
    </body>
  </worldbody>
  <sensor>
    <ballquat joint="ball"/>
  </sensor>
</mujoco>"""


def floats(a):
    return [float(x) for x in np.asarray(a).ravel()]


def stable(d, what):
    for w in (mujoco.mjtWarning.mjWARN_BADQACC, mujoco.mjtWarning.mjWARN_BADQVEL,
              mujoco.mjtWarning.mjWARN_BADQPOS):
        if d.warning[w].number:
            sys.exit(f"{what}: MuJoCo warned {w.name}")


def near_unit(q):
    """`q` turned into a quaternion whose length differs from 1 by at most
    mjMINVAL but not by 0, so mju_normalize4 leaves it as it is."""
    q = np.asarray(q, dtype=float) / np.linalg.norm(q)
    for _ in range(64):
        n = np.sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3])
        if n != 1 and abs(n - 1) <= MJMINVAL:
            return q
        q[0] = np.nextafter(q[0], np.inf)
    sys.exit("no quaternion within mjMINVAL of unit length found")


def scaled(q, length):
    q = np.asarray(q, dtype=float)
    q = q / np.linalg.norm(q) * length
    n = np.sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3])
    assert abs(n - 1) > MJMINVAL, n
    return q


def trajectory(m, d, what):
    out = {}
    for step in range(1, max(STEPS) + 1):
        mujoco.mj_step(m, d)
        if step in STEPS:
            out[str(step)] = {"qpos": floats(d.qpos), "qvel": floats(d.qvel)}
    stable(d, what)
    return out


def ref_case(xml, joint_type):
    m = mujoco.MjModel.from_xml_string(xml)
    assert m.jnt_type[0] == joint_type and m.qpos0[0] != 0, m.qpos0
    d = mujoco.MjData(m)
    mujoco.mj_forward(m, d)
    out = {"xml": xml, "qpos0": floats(m.qpos0), "xpos": floats(d.xpos), "xquat": floats(d.xquat),
           "xanchor": floats(d.xanchor), "xaxis": floats(d.xaxis)}
    out["traj"] = trajectory(m, d, xml[:40])
    return out


def offset_ball_case(xml, qpos, qvel):
    m = mujoco.MjModel.from_xml_string(xml)
    ball = [j for j in range(m.njnt) if m.jnt_type[j] == mujoco.mjtJoint.mjJNT_BALL]
    assert len(ball) == 1 and np.any(m.jnt_pos[ball[0]]), "the ball's pos is off its origin"
    d = mujoco.MjData(m)
    d.qpos[:] = qpos
    adr = m.jnt_qposadr[ball[0]]
    d.qpos[adr:adr + 4] /= np.linalg.norm(d.qpos[adr:adr + 4])
    d.qvel[:] = qvel
    mujoco.mj_forward(m, d)
    stable(d, "start")
    out = {"xml": xml, "qpos": floats(d.qpos), "qvel": floats(d.qvel),
           "xpos": floats(d.xpos), "xanchor": floats(d.xanchor)}
    # mjd_transitionFD leaves a perturbed evaluation's poses in its data, so
    # it runs on a copy of the state.
    fd = mujoco.MjData(m)
    fd.qpos[:] = d.qpos
    fd.qvel[:] = d.qvel
    mujoco.mj_forward(m, fd)
    nx = 2 * m.nv + m.na
    a = np.zeros((nx, nx))
    mujoco.mjd_transitionFD(m, fd, 1e-6, 1, a, None, None, None)
    out["A"] = floats(a)
    out["traj"] = trajectory(m, d, xml[:40])
    return out


def frames():
    m = mujoco.MjModel.from_xml_string(FRAMES)
    d = mujoco.MjData(m)
    h, s, ball, free = 0, 1, 2, 6
    assert [m.jnt_type[j] for j in range(4)] == [mujoco.mjtJoint.mjJNT_HINGE, mujoco.mjtJoint.mjJNT_SLIDE,
                                                  mujoco.mjtJoint.mjJNT_BALL, mujoco.mjtJoint.mjJNT_FREE]
    assert list(m.jnt_qposadr) == [h, s, ball, free], list(m.jnt_qposadr)
    d.qpos[h] = 0.4
    d.qpos[s] = 0.07
    d.qpos[ball:ball + 4] = scaled([0.7, 0.2, -0.5, 0.3], 1.3)
    d.qpos[free:free + 3] = [0.5, 0.1, 0.8]
    d.qpos[free + 3:free + 7] = near_unit([0.3, -0.6, 0.2, 0.7])
    d.mocap_pos[0] = [-0.25, 0.3, 0.45]
    d.mocap_quat[0] = scaled([0.4, 0.3, -0.2, 0.8], 0.9)
    # body b's quaternion of length 1.1: each body's final mju_normalize4 has work to do.
    m.body_quat[2] = scaled(m.body_quat[2], 1.1)
    qpos, mocap_pos, mocap_quat = floats(d.qpos), floats(d.mocap_pos), floats(d.mocap_quat)
    mujoco.mj_kinematics(m, d)
    model = {k: floats(getattr(m, k)) for k in (
        "body_pos", "body_quat", "body_ipos", "body_iquat", "jnt_pos", "jnt_axis", "qpos0",
        "geom_pos", "geom_quat", "site_pos", "site_quat")}
    poses = {k: floats(getattr(d, k)) for k in (
        "xpos", "xquat", "xmat", "xipos", "ximat", "xanchor", "xaxis",
        "geom_xpos", "geom_xmat", "site_xpos", "site_xmat")}
    return {"xml": FRAMES, "qpos": qpos, "mocap_pos": mocap_pos, "mocap_quat": mocap_quat,
            "model": model, "poses": poses}


def dof_lengths():
    return {name: {"xml": xml, "dof_length": floats(mujoco.MjModel.from_xml_string(xml).dof_length)}
            for name, xml in DOF_LENGTH.items()}


def quat_springs():
    m = mujoco.MjModel.from_xml_string(QUAT_SPRINGS)
    assert list(m.jnt_qposadr) == [0, 4], list(m.jnt_qposadr)
    spring = floats(m.qpos_spring)
    spring[0:4] = floats(scaled([1.1, 0.1, -0.2, 0.05], 1.1))
    spring[7:11] = floats(scaled([0.9, -0.3, 0.1, 0.2], 0.95))
    m.qpos_spring[:] = spring
    states = []
    for ball, free in (([0.8, 0.3, -0.4, 0.2], [0.6, 0.2, 0.7, -0.3]),
                       ([0.3, -0.7, 0.5, 0.4], [0.2, 0.9, -0.1, 0.35])):
        d = mujoco.MjData(m)
        d.qpos[0:4] = near_unit(ball)
        d.qpos[4:7] = [0.55, -0.05, 1.02]
        d.qpos[7:11] = scaled(free, 1.25)
        qpos = floats(d.qpos)
        mujoco.mj_forward(m, d)
        states.append({"qpos": qpos, "qfrc_spring": floats(d.qfrc_spring),
                       "sensordata": floats(d.sensordata)})
    return {"xml": QUAT_SPRINGS, "qpos_spring": spring, "states": states}


FRAME_QUATS = """<mujoco model="frame_quats">
  <worldbody>
    <body name="f" pos="0 0 1" quat="0.3 0.5 0.6 0.55">
      <freejoint/>
      <inertial pos="0.01 -0.02 0.0" quat="0.9 0.1 0.3 0.2" mass="2" diaginertia="0.03 0.02 0.015"/>
      <geom name="g" type="box" size="0.1 0.08 0.05" pos="0.02 0 0" quat="0.25 0.8 0.2 0.5"
            contype="0" conaffinity="0"/>
      <site name="s" pos="0.05 0.02 0.01" quat="0.2 0.9 0.1 0.3"/>
      <site name="r" pos="-0.03 0.01 0" quat="0.1 -0.4 0.7 0.5"/>
      <geom name="g2" type="sphere" size="0.02" pos="-0.05 0 0.02" quat="0.6 -0.3 0.2 0.7"
            contype="0" conaffinity="0" mass="0"/>
    </body>
  </worldbody>
  <sensor>
    <framequat objtype="site" objname="s"/>
    <framequat objtype="geom" objname="g"/>
    <framequat objtype="body" objname="f"/>
    <framequat objtype="xbody" objname="f"/>
    <framequat objtype="site" objname="s" reftype="site" refname="r"/>
    <framequat objtype="geom" objname="g" reftype="geom" refname="g2"/>
    <framequat objtype="body" objname="f" reftype="site" refname="r"/>
  </sensor>
</mujoco>"""


def frame_quats():
    m = mujoco.MjModel.from_xml_string(FRAME_QUATS)
    states = []
    for quat in ([0.3, 0.5, 0.6, 0.55], [0.9, -0.2, 0.3, 0.1]):
        d = mujoco.MjData(m)
        d.qpos[0:3] = [0.1, -0.2, 1.0]
        d.qpos[3:7] = np.array(quat) / np.linalg.norm(quat)
        mujoco.mj_forward(m, d)
        states.append({"qpos": floats(d.qpos), "sensordata": floats(d.sensordata)})
    if not any(st["sensordata"][0] < 0 and st["sensordata"][4] < 0 for st in states):
        sys.exit("frame_quats: no state where the site's and the geom's quaternions have w < 0")
    return {"xml": FRAME_QUATS, "states": states}


def main():
    if len(sys.argv) != 2:
        sys.exit(__doc__)
    marker = oracle()
    cases = {
        "hinge_ref": ref_case(HINGE_REF, mujoco.mjtJoint.mjJNT_HINGE),
        "slide_ref": ref_case(SLIDE_REF, mujoco.mjtJoint.mjJNT_SLIDE),
        "offset_ball": offset_ball_case(OFFSET_BALL, [0.8, 0.3, -0.4, 0.33], [0.4, -0.3, 0.6]),
        "chain_offball": offset_ball_case(CHAIN_OFFBALL, [0.3, 0.7, -0.2, 0.5, 0.4],
                                          [0.5, 0.3, -0.6, 0.4]),
        "frames": frames(),
        "dof_length": dof_lengths(),
        "quat_springs": quat_springs(),
        "frame_quats": frame_quats(),
    }
    doc = {"oracle": marker, "cases": finite_or_string(cases)}
    with open(os.path.join(sys.argv[1], "kinematics.json"), "w", encoding="utf-8") as f:
        json.dump(doc, f, separators=(",", ":"))
        f.write("\n")
    print(f"{len(cases)} cases -> {sys.argv[1]}")


if __name__ == "__main__":
    main()
