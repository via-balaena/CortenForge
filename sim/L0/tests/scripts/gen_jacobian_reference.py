#!/usr/bin/env python3
"""Generate the Jacobian golden data from the unfused MuJoCo 3.5.0 oracle.

Run it with the oracle's own interpreter (build_mujoco_oracle.sh builds it):

    <workdir>/venv/bin/python -I sim/L0/tests/scripts/gen_jacobian_reference.py \\
        sim/L0/tests/assets/golden/jacobian

It refuses any other interpreter, as gen_census_golden.py does.

MuJoCo's `mj_jac` (`engine_core_util.c:172-220`) builds each column from
`cdof`, the motion dof in the global frame centred at its tree's subtree
centre of mass (`mj_comPos`, `engine_core_smooth.c:298-350`), and the
point's offset from that centre; `mj_applyFT` (`engine_support.c:446-501`)
multiplies its transposes with a force and a torque. The fixture: a hinge
off its body's origin, a ball off its body's origin below it, a body with a
slide and a hinge below that, a jointless body welded to it (its Jacobian
takes its weld body's dofs, `body_weldid`), and a free body, at a moved
state. Each case
records the model's frame and mass fields, which the test writes into
sim-core's model: the loader rounds a normalized axis or quaternion
otherwise than MuJoCo's compiler (Rigid-loading's), and the comparison is
of the Jacobians alone. The cases:

- jac: `subtree_com` (`mj_comPos` normalizes each subtree's moment by the
  reciprocal of its mass); for each body, a point near its centre of mass,
  and `mj_jac`'s jacp and jacr there.
- apply_ft: for each body, a force and a torque at that point, and the
  generalized force after `mj_applyFT` adds them, the force's term first, to
  a vector that is not 0 (so the order of the two additions shows).
- gravcomp: the fixture with gravity compensation on every body (0.7, 1, 0.4,
  1.3): `qfrc_gravcomp` after mj_forward.
- contacts: a tilted free box on a plane (condim 3), and a hinged arm whose
  capsule (condim 6) and the sphere on a ball joint below it (condim 4)
  touch the plane, with an elliptic cone, so that a contact's `efc_J` rows
  are its Jacobian's rows (`mj_contactJacobian` and `mj_instantiateContact`,
  `engine_core_constraint.c:907-1087`): for each contact, its geoms,
  position, frame and dimension, and those rows.
"""
import json
import os
import sys

import mujoco
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from gen_census_golden import finite_or_string, oracle  # noqa: E402

FIXTURE = """<mujoco model="jacobian">
  <option gravity="0.3 -0.2 -9.81"/>
  <worldbody>
    <body name="a" pos="0.1 0.2 1" quat="0.9 0.1 0.2 0.3"{gc0}>
      <joint type="hinge" axis="0.3 0.8 0.1" pos="0.02 -0.01 0.03"/>
      <geom type="box" size="0.03 0.04 0.15" pos="0.01 0.02 -0.15" quat="0.8 0.2 0.1 0.3" mass="1.1"/>
      <body name="b" pos="0.05 0 -0.3"{gc1}>
        <joint type="ball" pos="0.01 0.02 0"/>
        <geom type="capsule" fromto="0 0 0 0.1 0.05 -0.25" size="0.03" mass="0.7"/>
        <body name="c" pos="0.1 0.05 -0.25"{gc2}>
          <joint type="slide" axis="0.2 0.3 0.9"/>
          <joint type="hinge" axis="1 0 0"/>
          <geom type="sphere" size="0.05" pos="0.02 0 0" mass="0.3"/>
          <body name="w" pos="0.03 0 0.02">
            <geom type="sphere" size="0.02" pos="0.01 0 0" mass="0.1"/>
          </body>
        </body>
      </body>
    </body>
    <body name="f" pos="1 0 1"{gc3}>
      <freejoint/>
      <geom type="box" size="0.1 0.06 0.03" pos="0.02 0.01 0" mass="2"/>
    </body>
  </worldbody>
</mujoco>"""

GRAVCOMP = (' gravcomp="0.7"', ' gravcomp="1"', ' gravcomp="0.4"', ' gravcomp="1.3"')
# (w takes no gravity compensation: the format fields name a, b, c, f.)
OFFSET = np.array([0.03, -0.02, 0.05])
FORCE = np.array([1.3, -0.7, 2.1])
TORQUE = np.array([0.4, 0.9, -0.6])
QFRC0 = np.array([0.37, -1.21, 0.0043, 2.9, -0.61, 1.7e-3, 0.88, -3.1, 0.25, 1.9, -0.47, 0.031])


CONTACTS = """<mujoco model="contacts">
  <option cone="elliptic"/>
  <worldbody>
    <geom name="floor" type="plane" size="2 2 0.1"/>
    <body name="box" pos="0 0 0.09" quat="0.98 0.1 0.15 0.05">
      <freejoint/>
      <geom type="box" size="0.1 0.07 0.05" mass="1"/>
    </body>
    <body name="arm" pos="0.6 0 0.4">
      <joint type="hinge" axis="0 1 0" pos="0 0 0.1"/>
      <geom type="capsule" fromto="0 0 0 0.3 0 -0.36" size="0.04" condim="6" mass="0.5"/>
      <body name="ball" pos="0.3 0 -0.36">
        <joint type="ball"/>
        <geom type="sphere" size="0.06" pos="0.05 0 -0.02" condim="4" mass="0.3"/>
      </body>
    </body>
  </worldbody>
</mujoco>"""


def floats(a):
    return [float(x) for x in np.asarray(a).ravel()]


def fields(m):
    return {k: floats(getattr(m, k)) for k in (
        "body_pos", "body_quat", "body_ipos", "body_iquat", "body_mass", "body_inertia",
        "jnt_pos", "jnt_axis", "qpos0")}


def moved(m):
    d = mujoco.MjData(m)
    d.qpos[0] = 0.4
    d.qpos[1:5] = [0.9, 0.2, -0.3, 0.1]
    d.qpos[5:7] = [0.07, -0.5]
    d.qpos[7:10] = [1.1, 0.2, 0.9]
    d.qpos[10:14] = [0.7, 0.3, -0.5, 0.4]
    for a in (1, 10):
        d.qpos[a:a + 4] /= np.linalg.norm(d.qpos[a:a + 4])
    mujoco.mj_forward(m, d)
    return d


def main():
    if len(sys.argv) != 2:
        sys.exit(__doc__)
    marker = oracle()
    xml = FIXTURE.format(gc0="", gc1="", gc2="", gc3="")
    m = mujoco.MjModel.from_xml_string(xml)
    d = moved(m)
    jac, apply_ft = [], []
    if m.body_weldid[m.body("w").id] != m.body("c").id:
        sys.exit("jac: w is not welded to c")
    for b in range(1, m.nbody):
        point = d.xipos[b] + OFFSET
        jacp, jacr = np.zeros((3, m.nv)), np.zeros((3, m.nv))
        mujoco.mj_jac(m, d, jacp, jacr, point, b)
        jac.append({"body": b, "point": floats(point), "jacp": floats(jacp), "jacr": floats(jacr)})
        qfrc = QFRC0.copy()
        mujoco.mj_applyFT(m, d, FORCE, TORQUE, point, b, qfrc)
        apply_ft.append({"body": b, "point": floats(point), "force": floats(FORCE),
                         "torque": floats(TORQUE), "qfrc0": floats(QFRC0), "qfrc": floats(qfrc)})
    gxml = FIXTURE.format(**{f"gc{i}": g for i, g in enumerate(GRAVCOMP)})
    gm = mujoco.MjModel.from_xml_string(gxml)
    gd = moved(gm)
    if not np.any(gd.qfrc_gravcomp):
        sys.exit("gravcomp: qfrc_gravcomp is 0")
    cm = mujoco.MjModel.from_xml_string(CONTACTS)
    cd = mujoco.MjData(cm)
    cd.qpos[7] = 0.15
    cd.qpos[8:12] = [0.95, 0.1, -0.2, 0.15]
    cd.qpos[8:12] /= np.linalg.norm(cd.qpos[8:12])
    mujoco.mj_forward(cm, cd)
    if mujoco.mj_isSparse(cm):
        sys.exit("contacts: a sparse Jacobian")
    efc_j = np.asarray(cd.efc_J).reshape(cd.nefc, cm.nv)
    contacts = []
    for i in range(cd.ncon):
        c = cd.contact[i]
        a = c.efc_address
        contacts.append({"geom": [int(c.geom[0]), int(c.geom[1])], "pos": floats(c.pos),
                         "frame": floats(c.frame), "dim": int(c.dim),
                         "efc_J": floats(efc_j[a:a + c.dim])})
    if sorted({c["dim"] for c in contacts}) != [3, 4, 6]:
        sys.exit(f"contacts: dims {[c['dim'] for c in contacts]}")
    cases = {"jac": {"xml": xml, "model": fields(m), "qpos": floats(d.qpos),
                     "subtree_com": floats(d.subtree_com), "bodies": jac},
             "apply_ft": {"xml": xml, "model": fields(m), "qpos": floats(d.qpos), "bodies": apply_ft},
             "gravcomp": {"xml": gxml, "model": fields(gm), "qpos": floats(gd.qpos),
                          "qfrc_gravcomp": floats(gd.qfrc_gravcomp)},
             "contacts": {"xml": CONTACTS, "model": fields(cm), "qpos": floats(cd.qpos),
                          "nv": int(cm.nv), "contacts": contacts}}
    doc = {"oracle": marker, "cases": finite_or_string(cases)}
    with open(os.path.join(sys.argv[1], "jacobian.json"), "w", encoding="utf-8") as f:
        json.dump(doc, f, separators=(",", ":"))
        f.write("\n")
    print(f"{len(cases)} cases -> {sys.argv[1]}")


if __name__ == "__main__":
    main()
