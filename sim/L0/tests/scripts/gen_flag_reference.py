#!/usr/bin/env python3
"""Generate the 3.5.0 flag golden data from the unfused MuJoCo 3.5.0 oracle.

Run it with the oracle's own interpreter (build_mujoco_oracle.sh builds it):

    <workdir>/venv/bin/python -I sim/L0/tests/scripts/gen_flag_reference.py \\
        sim/L0/tests/assets/golden/flags

It refuses any other interpreter, as gen_census_golden.py does.

It writes flags_3_5_0.json beside the 3.4.0 goldens (gen_flag_golden.py),
which run every flag on one scene, flag_golden_test.xml. This file gives each
flag below a scene the flag changes: flag_golden_test.xml with the options
named, or a scene of its own. For each case, from mj_resetData with every
ctrl set to `ctrl`, NSTEPS mj_step calls without the flag (`baseline`) and
NSTEPS with it (`flagged`), recording after each step qpos, qvel, qacc,
sensordata, energy, solver_fwdinv, nisland, `niter` (solver_niter summed
over the first min(nisland, 20) islands, or solver_niter[0] when nisland is
0) and each warning's count.

It refuses a case whose flagged run is within REFUSE_BELOW of its baseline,
the distance being the largest |baseline - flagged| / max(1, |flagged|)
over the steps and the float channels: a test at TOLERANCE could then not
tell the flag ignored. The cases:

- autoreset (disable): a slide driven by a motor of gear 1e12, so the first
  forward's qacc is 1e12, past mjMAXVAL; MuJoCo resets the data (ctrl to 0)
  and runs the forward again, and with the flag continues from the bad
  state (`mj_checkAcc`, engine_forward.c:92-112). The run must raise the
  bad-qacc warning.
- sensor (disable): the sensors are not computed, so sensordata stays 0;
  the scene gains a velocity sensor (jointvel) and an acceleration one
  (accelerometer), so each stage's gate is reached.
- energy (enable): the energy is computed.
- fwdinv (enable): CG at 2 iterations, so the forward's constraint force is
  not the one its qacc implies and the comparison's statistics are not 0.
- override (enable): the option's o_margin, o_solref, o_solimp and
  o_friction take the contacts' place; the scene gains a static sphere
  0.015 below the arm's sphere, which only o_margin's 0.02 makes a contact
  of (in the broadphase and the narrowphase), and a static sphere 0.015
  below the forearm's that collides only through a `<pair>`, whose
  parameters o_* replace.
- island (disable) and warmstart (disable): CG at 2 iterations, so the
  island split and the warmstart change the force the solve stops at.
- warmstart_pgs (disable): the warmstart flag under PGS at 2 iterations;
  MuJoCo starts PGS from zero forces under the flag (engine_forward.c:
  682-686), and the warm forces it would otherwise keep move where the
  solve stops.
- sleep (enable): the free ball starts asleep (sleep="init").
- invdiscrete (enable): the fwdinv case's scene with the fwdinv flag on in
  both runs; the flag makes the comparison's inverse the discrete one
  (`mj_discreteAcc`, engine_inverse.c:76-153), and the scene's damped dofs
  make Euler's discrete acceleration differ from the continuous one, so
  solver_fwdinv moves.
- multiccd (enable): a cylinder standing 1 mm into a box; mjc_Convex lists
  one contact at its cap without the flag, and with it four more passes
  with the geoms turned 1e-3 rad find the cap's other contacts
  (engine_collision_convex.c:917-1000).

Not here:
- midphase: no rigid-body scene was found where it moves MuJoCo's
  trajectory (flag_golden_test.xml, and two multi-geom bodies on
  multi-geom static bodies over 50 steps, one under PGS at 3 iterations:
  bit-equal); MuJoCo sorts the midphase's contacts into the all-to-all
  order (engine_collision_driver.c:235, :360-371), but not a flex's
  self-collision's (:446-472), and sim-core's does not read the flag
  (ledger L158).
- nativeccd: with it MuJoCo collides convex pairs with libccd, which
  sim-core does not port; sim-core refuses a model with the flag
  (`Model::try_make_data`, registry D-NATIVECCD).
"""
import json
import os
import sys

import mujoco

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from gen_census_golden import finite_or_string, oracle  # noqa: E402

NSTEPS = 10
TOLERANCE = 1e-8
REFUSE_BELOW = 1e-5
FLOAT_CHANNELS = ("qpos", "qvel", "qacc", "sensordata", "energy", "fwdinv")

SLIDE = """<mujoco model="autoreset">
  <option timestep="0.002"/>
  <worldbody>
    <body name="s" pos="0 0 1">
      <joint name="z" type="slide" axis="0 0 1" damping="0.1"/>
      <geom type="sphere" size="0.05" mass="1"/>
    </body>
  </worldbody>
  <actuator>
    <motor joint="z" gear="1e12"/>
  </actuator>
</mujoco>"""


CYLINDER = """<mujoco model="multiccd">
  <option timestep="0.002"/>
  <worldbody>
    <geom name="slab" type="box" size="0.5 0.4 0.1" pos="0 0 -0.1"/>
    <body name="c" pos="0.02 -0.01 0.049">
      <freejoint/>
      <geom type="cylinder" size="0.05 0.05" mass="0.5"/>
    </body>
  </worldbody>
</mujoco>"""


def canonical(path):
    with open(path, encoding="utf-8") as f:
        return f.read()


def with_options(xml, attrs):
    old = '<option gravity="0 0 -9.81" timestep="0.002"/>'
    new = f'<option gravity="0 0 -9.81" timestep="0.002" {attrs}/>'
    out = xml.replace(old, new, 1)
    assert out != xml, "flag_golden_test.xml's <option> line changed"
    return out


def with_ball_asleep(xml):
    old = '<body name="ball" pos="1 0 0.049">'
    out = xml.replace(old, '<body name="ball" pos="1 0 0.049" sleep="init">', 1)
    assert out != xml, "flag_golden_test.xml's ball changed"
    return out


def with_sensors(xml):
    old = '<jointpos name="hinge1_pos" joint="hinge1"/>'
    new = (old + '\n    <jointvel name="hinge1_vel" joint="hinge1"/>'
           '\n    <accelerometer name="tip_acc" site="tip"/>')
    out = xml.replace(old, new, 1)
    assert out != xml, "flag_golden_test.xml's sensor changed"
    return out


def with_near_misses(xml):
    # Static spheres 0.015 below the arm's sphere (world pos 0.12 0 1.5,
    # radius 0.05) and the forearm's (0.15 0 1.5, radius 0.04); the second
    # collides only through its pair.
    old = '    <!-- Free-floating ball slightly penetrating floor'
    new = ('    <geom name="near" type="sphere" size="0.05" pos="0.12 0 1.385"/>\n'
           '    <geom name="paired" type="sphere" size="0.04" pos="0.15 0 1.405"'
           ' contype="0" conaffinity="0"/>\n' + old)
    out = xml.replace(old, new, 1)
    old = '  <!-- Fixed tendon coupling hinge1 and hinge2'
    new = ('  <contact>\n    <pair geom1="forearm_geom" geom2="paired"/>\n  </contact>\n\n' + old)
    out2 = out.replace(old, new, 1)
    assert out != xml and out2 != out, "flag_golden_test.xml's layout changed"
    return out2


def with_fwdinv(xml):
    old = '<option gravity="0 0 -9.81" timestep="0.002" solver="CG" iterations="2"/>'
    new = ('<option gravity="0 0 -9.81" timestep="0.002" solver="CG" iterations="2">\n'
           '    <flag fwdinv="enable"/>\n  </option>')
    out = xml.replace(old, new, 1)
    assert out != xml, "the CG option line changed"
    return out


def cases(canon):
    """Each case's (scene, kind, flag, ctrl), by name."""
    cg2 = with_options(canon, 'solver="CG" iterations="2"')
    pgs2 = with_options(canon, 'solver="PGS" iterations="2"')
    override = with_options(with_near_misses(canon), 'o_margin="0.02" o_solref="0.05 0.8" '
                            'o_solimp="0.8 0.9 0.01 0.5 2" o_friction="0.4 0.4 0.01 0.001 0.001"')
    return {
        "autoreset": (SLIDE, "disable", "autoreset", 1.0),
        "sensor": (with_sensors(canon), "disable", "sensor", 2.0),
        "energy": (canon, "enable", "energy", 2.0),
        "fwdinv": (cg2, "enable", "fwdinv", 2.0),
        "override": (override, "enable", "override", 2.0),
        "island": (cg2, "disable", "island", 2.0),
        "warmstart": (cg2, "disable", "warmstart", 2.0),
        "warmstart_pgs": (pgs2, "disable", "warmstart", 2.0),
        "sleep": (with_ball_asleep(canon), "enable", "sleep", 2.0),
        "invdiscrete": (with_fwdinv(cg2), "enable", "invdiscrete", 2.0),
        "multiccd": (CYLINDER, "enable", "multiccd", 0.0),
    }


def flag_bit(kind, name):
    if kind == "disable":
        return int(getattr(mujoco.mjtDisableBit, "mjDSBL_" + name.upper()))
    return int(getattr(mujoco.mjtEnableBit, "mjENBL_" + name.upper()))


def run(xml, kind, name, flagged, ctrl):
    m = mujoco.MjModel.from_xml_string(xml)
    if flagged and kind == "disable":
        m.opt.disableflags |= flag_bit(kind, name)
    elif flagged:
        m.opt.enableflags |= flag_bit(kind, name)
    d = mujoco.MjData(m)
    d.ctrl[:] = ctrl
    steps = []
    for _ in range(NSTEPS):
        mujoco.mj_step(m, d)
        islands = min(int(d.nisland), 20)
        steps.append({
            "qpos": d.qpos.tolist(), "qvel": d.qvel.tolist(), "qacc": d.qacc.tolist(),
            "sensordata": d.sensordata.tolist(), "energy": d.energy.tolist(),
            "fwdinv": d.solver_fwdinv.tolist(), "nisland": int(d.nisland),
            "niter": int(sum(d.solver_niter[:islands])) if islands else int(d.solver_niter[0]),
            "warnings": [int(w.number) for w in d.warning],
        })
    return steps


def distance(baseline, flagged):
    worst = 0.0
    for a, b in zip(baseline, flagged):
        for ch in FLOAT_CHANNELS:
            for x, y in zip(a[ch], b[ch]):
                worst = max(worst, abs(x - y) / max(1.0, abs(y)))
    return worst


def main():
    if len(sys.argv) != 2:
        sys.exit(__doc__)
    marker = oracle()
    warned = []
    mujoco.set_mju_user_warning(warned.append)
    out = {}
    canon = canonical(os.path.join(sys.argv[1], "flag_golden_test.xml"))
    for name, (xml, kind, flag, ctrl) in cases(canon).items():
        baseline = run(xml, kind, flag, False, ctrl)
        flagged = run(xml, kind, flag, True, ctrl)
        gap = distance(baseline, flagged)
        if gap <= REFUSE_BELOW:
            sys.exit(f"{name}: the flag moves the trajectory by {gap:.3e}, "
                     f"not above {REFUSE_BELOW:.0e}: a test could not tell it ignored")
        out[name] = {"xml": xml, "kind": kind, "flag": flag, "ctrl": ctrl, "distance": gap,
                     "baseline": baseline, "flagged": flagged}
    bad_qacc = int(mujoco.mjtWarning.mjWARN_BADQACC)
    if out["autoreset"]["baseline"][0]["warnings"][bad_qacc] != 1:
        sys.exit("autoreset: the first step raised no bad-qacc warning")
    doc = {"oracle": marker, "nsteps": NSTEPS, "tolerance": TOLERANCE,
           "refuse_below": REFUSE_BELOW, "cases": finite_or_string(out)}
    with open(os.path.join(sys.argv[1], "flags_3_5_0.json"), "w", encoding="utf-8") as f:
        json.dump(doc, f, separators=(",", ":"))
        f.write("\n")
    print(f"{len(out)} cases -> {sys.argv[1]} ({len(warned)} MuJoCo warnings, as the autoreset case raises)")


if __name__ == "__main__":
    main()
