#!/usr/bin/env python3
"""Generate the multi-joint-body golden data from the unfused MuJoCo 3.5.0 oracle.

Run it with the oracle's own interpreter (build_mujoco_oracle.sh builds it):

    <workdir>/venv/bin/python -I sim/L0/tests/scripts/gen_multijoint_reference.py \\
        sim/L0/tests/assets/golden/multijoint

It refuses any other interpreter, as gen_census_golden.py does.

Each case is a body carrying two or more joints (MuJoCo's `mj_comVel` steps the
body velocity over them, `engine_core_smooth.c:2285-2336`), and the same
joints split over bodies chained through 1e-9 kg intermediates where the case
names a split. For each it writes the XML, the model fields the dynamics read,
a state (qpos from qpos0 moved by `mj_integratePos`, so quaternions stay unit;
qvel set), and at that state after `mj_forward` and `mj_rnePostConstraint`:
qfrc_bias, qacc, the dense mass matrix, and each body's cacc and cfrc_int moved
from MuJoCo's frame (at the subtree centre of mass of the body's root) to the
body's origin xpos, where sim-core keeps them. Then qpos and qvel after steps
1, 10, 100 and 500 of mj_step from that state under Euler, RK4, implicit and
implicitfast.
"""
import json
import os
import sys

import mujoco
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from gen_census_golden import finite_or_string, oracle  # noqa: E402

INTEGRATORS = ["Euler", "RK4", "implicit", "implicitfast"]
CHECKPOINTS = [1, 10, 100, 500]

# The rotation is MuJoCo's compiled geom_quat for euler="10 20 30", written as a
# quat: sim-mjcf's inertia from geoms reads a geom's quat and not its euler until
# Rigid-loading's geom-frame entry (`geom_euler_rotates_body_inertia`).
GEOM = ('<geom type="box" size="0.12 0.05 0.03" pos="0.1 0.04 -0.02" mass="1.3" '
        'quat="0.943714364147489 0.12767944069578063 0.14487812541736914 0.2685358227515692"/>')


# A motor at a site on the body, its gear in all six directions, at a control
# of 0: it adds no force, and gives the transition's B a column.
SITE = '<site name="s" pos="0.05 -0.02 0.03"/>'
MOTOR = '<actuator><motor site="s" gear="1 0.5 -0.3 0.2 -0.4 0.1"/></actuator>'


def model_xml(joints, integrator="Euler", parent=None):
    """A body carrying `joints` (MJCF joint elements), under `parent` if given."""
    if parent is None:
        inner = f'<body name="b" pos="0 0 0.5">{joints}{GEOM}{SITE}</body>'
    else:
        inner = (f'<body name="p" pos="0 0 0.5">{parent}'
                 '<geom type="capsule" fromto="0 0 0 0.2 0 0" size="0.02" mass="0.4"/>'
                 f'<body name="b" pos="0.2 0 0">{joints}{GEOM}{SITE}</body></body>')
    return (f'<mujoco><option timestep="0.002" integrator="{integrator}"/>'
            f'<worldbody>{inner}</worldbody>{MOTOR}</mujoco>')


def split_xml(joints, integrator="Euler"):
    """Each joint on its own body, chained through 1e-9 kg intermediates; the geom on the last."""
    inner = f"<body>{joints[-1]}{GEOM}</body>"
    for j in reversed(joints[:-1]):
        inner = f'<body>{j}<inertial pos="0 0 0" mass="1e-9" diaginertia="1e-9 1e-9 1e-9"/>{inner}</body>'
    # The outermost body carries the case's pos; MuJoCo needs no name.
    inner = inner.replace("<body>", '<body pos="0 0 0.5">', 1)
    return (f'<mujoco><option timestep="0.002" integrator="{integrator}"/>'
            f'<worldbody>{inner}</worldbody></mujoco>')


H0 = '<joint type="hinge" axis="1 0 0" damping="0.05"/>'
H1 = '<joint type="hinge" axis="0 1 0" damping="0.05"/>'
H2 = '<joint type="hinge" axis="0.3 0.4 0.866" damping="0.05"/>'
# Off the body's origin.
H1_OFF = '<joint type="hinge" axis="0 1 0" pos="0.05 0.02 0" damping="0.05"/>'
# A hinge through the ball's centre would repeat one of the ball's rotations.
H0_OFF = '<joint type="hinge" axis="1 0 0" pos="0 0.05 0.03" damping="0.05"/>'
S0 = '<joint type="slide" axis="1 0 0" damping="0.05"/>'
S1 = '<joint type="slide" axis="0 0.6 0.8" damping="0.05"/>'
S2 = '<joint type="slide" axis="0 0 1" damping="0.05"/>'
# At the body's origin.
B0 = '<joint type="ball" damping="0.05"/>'

CASES = {
    "hinge_hinge": dict(joints=[H0, H1]),
    "hinge_offset_hinge": dict(joints=[H0, H1_OFF]),
    "slide_hinge": dict(joints=[S0, H1]),
    "hinge_slide": dict(joints=[H0, S1]),
    "hinge_ball": dict(joints=[H0_OFF, B0]),
    "slide_ball": dict(joints=[S0, B0]),
    "six": dict(joints=[S0, S1, S2, H0, H1, H2]),
    "hinge_hinge_under_hinge": dict(joints=[H0, H1], parent=H2),
}
SPLIT = ["hinge_hinge", "slide_hinge", "hinge_ball"]


def floats(a):
    return [float(x) for x in np.asarray(a).ravel()]


def state(m, seed):
    rng = np.random.default_rng(seed)
    qpos = m.qpos0.copy()
    mujoco.mj_integratePos(m, qpos, 0.4 * rng.standard_normal(m.nv), 1.0)
    qvel = 0.8 * rng.standard_normal(m.nv)
    return qpos, qvel


def at_origin(m, d, vec, kind):
    """MuJoCo's com-frame spatial vectors (nbody x 6, [ang; lin]) moved to each body's xpos."""
    out = []
    for b in range(m.nbody):
        c = d.subtree_com[m.body_rootid[b]]
        ang, lin = vec[b, :3], vec[b, 3:]
        off = d.xpos[b] - c
        if kind == "motion":
            out.append(floats(np.concatenate([ang, lin + np.cross(ang, off)])))
        else:
            out.append(floats(np.concatenate([ang - np.cross(off, lin), lin])))
    return out


def forward(m, qpos, qvel):
    d = mujoco.MjData(m)
    d.qpos[:] = qpos
    d.qvel[:] = qvel
    mujoco.mj_forward(m, d)
    stable(d, "forward")
    mujoco.mj_rnePostConstraint(m, d)
    M = np.zeros((m.nv, m.nv))
    mujoco.mj_fullM(m, M, d.qM)
    return {
        "qfrc_bias": floats(d.qfrc_bias),
        "qacc": floats(d.qacc),
        "qM": floats(M),
        "cacc": at_origin(m, d, d.cacc, "motion"),
        "cfrc_int": at_origin(m, d, d.cfrc_int, "force"),
    }


def stable(d, what):
    """Refuse a run MuJoCo flagged: a singular case makes no golden."""
    for w in (mujoco.mjtWarning.mjWARN_BADQACC, mujoco.mjtWarning.mjWARN_BADQVEL,
              mujoco.mjtWarning.mjWARN_BADQPOS):
        if d.warning[w].number:
            sys.exit(f"{what}: MuJoCo warned {w.name}")


def trajectory(xml, qpos, qvel, what):
    m = mujoco.MjModel.from_xml_string(xml)
    d = mujoco.MjData(m)
    d.qpos[:] = qpos
    d.qvel[:] = qvel
    out = {}
    for step in range(1, CHECKPOINTS[-1] + 1):
        mujoco.mj_step(m, d)
        if step in CHECKPOINTS:
            out[str(step)] = {"qpos": floats(d.qpos), "qvel": floats(d.qvel)}
    stable(d, what)
    return out


def model_fields(m):
    return {k: floats(getattr(m, k)) for k in (
        "body_mass", "body_inertia", "body_ipos", "body_iquat", "body_pos", "body_quat",
        "jnt_pos", "jnt_axis", "qpos0")}


def case(name, spec, seed):
    xml = {i: model_xml("".join(spec["joints"]), i, spec.get("parent")) for i in INTEGRATORS}
    m = mujoco.MjModel.from_xml_string(xml["Euler"])
    qpos, qvel = state(m, seed)
    out = {
        "xml": xml,
        "model": model_fields(m),
        "qpos": floats(qpos),
        "qvel": floats(qvel),
        "forward": forward(m, qpos, qvel),
        "traj": {i: trajectory(xml[i], qpos, qvel, f"{name} {i}") for i in INTEGRATORS},
    }
    if name in SPLIT:
        sx = split_xml(spec["joints"])
        sm = mujoco.MjModel.from_xml_string(sx)
        assert sm.nq == m.nq and sm.nv == m.nv
        sd = mujoco.MjData(sm)
        sd.qpos[:] = qpos
        sd.qvel[:] = qvel
        mujoco.mj_forward(sm, sd)
        stable(sd, f"{name} split")
        out["split"] = {"xml": sx, "qacc": floats(sd.qacc)}
    return out


def write(path, doc):
    with open(path, "w", encoding="utf-8") as f:
        json.dump(doc, f, separators=(",", ":"))
        f.write("\n")


def main():
    if len(sys.argv) != 2:
        sys.exit(__doc__)
    marker = oracle()
    cases = {name: case(name, spec, seed) for seed, (name, spec) in enumerate(CASES.items())}
    doc = {"oracle": marker, "cases": finite_or_string(cases)}
    write(os.path.join(sys.argv[1], "multijoint.json"), doc)
    print(f"{len(cases)} cases -> {sys.argv[1]}")


if __name__ == "__main__":
    main()
