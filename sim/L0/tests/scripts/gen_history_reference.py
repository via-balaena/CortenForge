#!/usr/bin/env python3
"""Generate the history-buffer (delay) golden data from the unfused MuJoCo 3.5.0 oracle.

Run it with the oracle's own interpreter (build_mujoco_oracle.sh builds it):

    <workdir>/venv/bin/python -I sim/L0/tests/scripts/gen_history_reference.py \\
        sim/L0/tests/assets/golden/history

It refuses any other interpreter, as gen_census_golden.py does: the PyPI wheel
fuses multiply-adds. History buffers are new in MuJoCo 3.5.0, so this golden
has no 3.4.0 counterpart.

Each case is a model (its XML), a driver and a control sequence; the file holds
MuJoCo's history buffer after make_data (and after the keyframe reset, if any)
and, after each call, the time before it, actuator_force, qpos, qvel,
sensordata and history. The models:

- act: a 1-dof slide with 12 motors: no buffer; a buffer and no delay; delays
  of 0.3 to 3.7 timesteps; zoh, linear and cubic; 2 to 6 samples, one with a
  delay longer than its buffer;
- sens: a damped hinge with 14 sensors: jointpos, jointvel, framepos,
  framequat, actuatorfrc and accelerometer with a buffer only, delayed,
  interval, interval with phase, interval with delay, and cubic with cutoff;
- acc: an accelerometer with and without a one-step delay, and a jointpos.

Drivers: mj_step under Euler, RK4, implicitfast and implicit; mj_forward;
mj_step1 + mj_step2; a reset part-way; keyframes at times 0.5, -0.015, -0.02
and -0.0333 (the negative times take the buffer's out-of-order, exact-match
and older-than-oldest insert branches).

The control is sin(0.9 k) + 0.05 k at step k on every actuator, so a delay
shows in the force.

It writes history.json (the cases above) and history_api.json: MuJoCo's
mj_readCtrl and mj_readSensor after 12 Euler steps of the act and sens
models, at 55 times from -0.05 to 0.1498 with each interpolation (-1 is the
model's), and the buffer mj_initCtrlHistory leaves on actuator 8 then (its
cursor is not at its last slot); the buffers mj_initCtrlHistory (actuators 2 and 7) and
mj_initSensorHistory (sensor 6, phase 0.123) leave, and the forces of the
4 steps after; and which calls MuJoCo refuses.
"""
import json
import math
import os
import sys

import mujoco
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from gen_census_golden import finite_or_string, oracle  # noqa: E402

DT = 0.01
NSTEP = 20

ACT_XML = """<mujoco>
  <option timestep="{dt}" gravity="0 0 0" integrator="{integ}"/>
  <worldbody>
    <body name="b">
      <joint name="j" type="slide" axis="1 0 0"/>
      <geom type="sphere" size="0.1" mass="1"/>
    </body>
  </worldbody>
  <actuator>
{acts}
  </actuator>
</mujoco>
"""

# (nsample, delay in timesteps, interp)
ACT_CONFIGS = [
    (0, 0.0, None),
    (3, 0.0, "zoh"),
    (3, 1.0, "zoh"),
    (3, 1.0, "linear"),
    (4, 1.5, "zoh"),
    (4, 1.5, "linear"),
    (4, 1.5, "cubic"),
    (5, 2.5, "cubic"),
    (5, 2.5, "linear"),
    (2, 0.3, "linear"),
    (2, 3.0, "zoh"),
    (6, 3.7, "cubic"),
]

SENS_XML = """<mujoco>
  <option timestep="{dt}" gravity="0 0 0" integrator="{integ}"/>
  <worldbody>
    <body name="b">
      <joint name="j" type="hinge" axis="0 0 1" damping="0.05"/>
      <geom type="capsule" fromto="0 0 0 0.3 0 0" size="0.03" mass="0.5"/>
      <site name="s" pos="0.3 0 0"/>
    </body>
  </worldbody>
  <actuator>
    <motor name="m" joint="j" gear="0.2"/>
  </actuator>
  <sensor>
{sens}
  </sensor>
</mujoco>
"""

# (type, target, nsample, delay in timesteps, interp, interval, cutoff)
SENS_CONFIGS = [
    ("jointpos", 'joint="j"', 0, 0.0, None, None, None),
    ("jointpos", 'joint="j"', 3, 0.0, None, None, None),
    ("jointpos", 'joint="j"', 3, 1.0, "zoh", None, None),
    ("jointpos", 'joint="j"', 4, 1.5, "linear", None, None),
    ("jointpos", 'joint="j"', 5, 2.5, "cubic", None, None),
    ("jointvel", 'joint="j"', 4, 1.5, "linear", None, None),
    ("framepos", 'objtype="site" objname="s"', 4, 1.5, "linear", None, None),
    ("framequat", 'objtype="site" objname="s"', 3, 1.0, "zoh", None, None),
    ("actuatorfrc", 'actuator="m"', 4, 2.0, "zoh", None, None),
    ("accelerometer", 'site="s"', 4, 1.5, "linear", None, None),
    ("jointpos", 'joint="j"', 3, 0.0, "zoh", "0.025", None),
    ("jointpos", 'joint="j"', 3, 0.0, "linear", "0.03 -0.01", None),
    ("jointvel", 'joint="j"', 4, 1.0, "zoh", "0.02", None),
    ("jointvel", 'joint="j"', 5, 2.5, "cubic", None, "0.02"),
]

ACC_CONFIGS = [
    ("accelerometer", 'site="s"', 0, 0.0, None, None, None),
    ("accelerometer", 'site="s"', 3, 1.0, "zoh", None, None),
    ("jointpos", 'joint="j"', 0, 0.0, None, None, None),
]


def act_xml(integ):
    lines = []
    for i, (ns, dl, ip) in enumerate(ACT_CONFIGS):
        attrs = f'name="m{i}" joint="j" gear="0.01"'
        if ns:
            attrs += f' nsample="{ns}"'
        if dl:
            attrs += f' delay="{dl * DT!r}"'
        if ip:
            attrs += f' interp="{ip}"'
        lines.append(f"    <motor {attrs}/>")
    return ACT_XML.format(dt=DT, integ=integ, acts="\n".join(lines))


def sens_xml(integ, configs=SENS_CONFIGS):
    lines = []
    for i, (ty, tgt, ns, dl, ip, iv, co) in enumerate(configs):
        attrs = f'name="s{i}" {tgt}'
        if ns:
            attrs += f' nsample="{ns}"'
        if dl:
            attrs += f' delay="{dl * DT!r}"'
        if ip:
            attrs += f' interp="{ip}"'
        if iv:
            attrs += f' interval="{iv}"'
        if co:
            attrs += f' cutoff="{co}"'
        lines.append(f"    <{ty} {attrs}/>")
    return SENS_XML.format(dt=DT, integ=integ, sens="\n".join(lines))


def with_key(xml, attrs):
    return xml.replace("</mujoco>", f'  <keyframe><key name="k" {attrs}/></keyframe>\n</mujoco>')


def floats(a):
    return [float(x) for x in a]


def run_case(name, xml, driver="step", nstep=NSTEP, key=None, reset_at=None):
    m = mujoco.MjModel.from_xml_string(xml)
    d = mujoco.MjData(m)
    if key is not None:
        mujoco.mj_resetDataKeyframe(m, d, key)
    init_history = floats(d.history)
    ctrls = [[math.sin(0.9 * k) + 0.05 * k] * m.nu for k in range(nstep)]
    records = []
    for k in range(nstep):
        if reset_at is not None and k == reset_at:
            mujoco.mj_resetData(m, d)
        d.ctrl[:] = ctrls[k]
        t = float(d.time)
        if driver == "step":
            mujoco.mj_step(m, d)
        elif driver == "forward":
            mujoco.mj_forward(m, d)
        elif driver == "step12":
            mujoco.mj_step1(m, d)
            mujoco.mj_step2(m, d)
        else:
            raise ValueError(driver)
        records.append({
            "time": t,
            "actuator_force": floats(d.actuator_force),
            "qpos": floats(d.qpos),
            "qvel": floats(d.qvel),
            "sensordata": floats(d.sensordata),
            "history": floats(d.history),
        })
    return {
        "name": name,
        "xml": xml,
        "driver": driver,
        "key": key,
        "reset_at": reset_at,
        "ctrl": ctrls,
        "nhistory": int(m.nhistory),
        "actuator_historyadr": [int(x) for x in m.actuator_historyadr],
        "sensor_historyadr": [int(x) for x in m.sensor_historyadr],
        "init_history": init_history,
        "records": records,
    }


def cases():
    out = []
    for integ in ["Euler", "RK4", "implicitfast"]:
        out.append(run_case(f"act_{integ}", act_xml(integ)))
        out.append(run_case(f"sens_{integ}", sens_xml(integ)))
    out.append(run_case("act_forward_only", act_xml("Euler"), driver="forward", nstep=5))
    out.append(run_case("sens_forward_only", sens_xml("Euler"), driver="forward", nstep=5))
    out.append(run_case("act_step12_Euler", act_xml("Euler"), driver="step12"))
    out.append(run_case("act_step12_RK4", act_xml("RK4"), driver="step12"))
    out.append(run_case("sens_step12_Euler", sens_xml("Euler"), driver="step12"))
    out.append(run_case("sens_step12_RK4", sens_xml("RK4"), driver="step12"))
    out.append(run_case("act_reset_mid", act_xml("Euler"), reset_at=7))
    out.append(run_case("act_key_t0.5", with_key(act_xml("Euler"), 'time="0.5" qpos="0.2" qvel="0.1"'), key=0))
    out.append(run_case("sens_key_t0.5", with_key(sens_xml("Euler"), 'time="0.5" qpos="0.2" qvel="0.1"'), key=0))
    for t in ["-0.015", "-0.02", "-0.0333"]:
        out.append(run_case(f"act_key_t{t}", with_key(act_xml("Euler"), f'time="{t}"'), key=0, nstep=8))
    for t in ["-0.015", "-0.02"]:
        out.append(run_case(f"sens_key_t{t}", with_key(sens_xml("Euler"), f'time="{t}"'), key=0, nstep=8))
    for integ in ["Euler", "implicitfast", "implicit", "RK4"]:
        out.append(run_case(f"acc_{integ}", sens_xml(integ, ACC_CONFIGS)))
    return out


def drive(m, d, nstep):
    for k in range(nstep):
        d.ctrl[:] = [math.sin(0.9 * k) + 0.05 * k] * m.nu
        mujoco.mj_step(m, d)


def refusal(call):
    try:
        call()
    except Exception as e:  # noqa: BLE001 — MuJoCo's mjERROR, recorded as its message
        return str(e)
    return None


def api():
    out = {}
    times = [round(-0.05 + 0.0037 * i, 10) for i in range(55)]
    for kind, xml in [("act", act_xml("Euler")), ("sens", sens_xml("Euler"))]:
        m = mujoco.MjModel.from_xml_string(xml)
        d = mujoco.MjData(m)
        drive(m, d, 12)
        reads = []
        for i in range(m.nu if kind == "act" else m.nsensor):
            for interp in (-1, 0, 1, 2):
                for t in times:
                    if kind == "act":
                        value = [float(mujoco.mj_readCtrl(m, d, i, t, interp))]
                    else:
                        res = np.zeros(m.sensor_dim[i])
                        value = floats(np.asarray(mujoco.mj_readSensor(m, d, i, t, res, interp)).ravel())
                    reads.append([i, interp, t, value])
        out[kind] = {"xml": xml, "history": floats(d.history), "reads": reads}
        if kind == "act":
            # an init after steps, where the cursor is not at the last slot
            mujoco.mj_initCtrlHistory(
                m, d, 8, np.array([-0.03, -0.02, -0.01, 0.0, 0.05]), np.array([1.0, 2.0, 3.0, 4.0, 5.0]))
            out[kind]["history_after_init"] = floats(d.history)
    m = mujoco.MjModel.from_xml_string(act_xml("Euler"))
    d = mujoco.MjData(m)
    mujoco.mj_initCtrlHistory(m, d, 2, np.array([-0.03, -0.02, -0.01]), np.array([1.0, 2.0, 3.0]))
    mujoco.mj_initCtrlHistory(m, d, 7, None, np.array([0.5, -0.5, 0.25, 4.0, 1.0]))
    act_history = floats(d.history)
    forces = []
    for _ in range(4):
        d.ctrl[:] = 0.0
        mujoco.mj_step(m, d)
        forces.append(floats(d.actuator_force))
    refused = {
        "no_buffer": refusal(lambda: mujoco.mj_initCtrlHistory(m, d, 0, None, np.zeros(0))),
        "not_increasing": refusal(lambda: mujoco.mj_initCtrlHistory(
            m, d, 2, np.array([0.0, 0.0, 1.0]), np.zeros(3))),
        "bad_actuator": refusal(lambda: mujoco.mj_readCtrl(m, d, 99, 0.0, -1)),
    }
    ms = mujoco.MjModel.from_xml_string(sens_xml("Euler"))
    ds = mujoco.MjData(ms)
    mujoco.mj_initSensorHistory(ms, ds, 6, None, np.arange(12, dtype=float).reshape(4, 3), 0.123)
    out["init"] = {"act_history": act_history, "act_forces": forces, "refused": refused,
                   "sens_history": floats(ds.history)}
    return out


def write(path, doc):
    with open(path, "w", encoding="utf-8") as f:
        json.dump(doc, f, separators=(",", ":"))
        f.write("\n")


def main():
    if len(sys.argv) != 2:
        sys.exit(__doc__)
    marker = oracle()
    doc = {"oracle": marker, "dt": DT, "cases": finite_or_string(cases())}
    write(os.path.join(sys.argv[1], "history.json"), doc)
    write(os.path.join(sys.argv[1], "history_api.json"),
          {"oracle": marker, "dt": DT, "api": finite_or_string(api())})
    print(f"{len(doc['cases'])} cases and the API cases -> {sys.argv[1]}")


if __name__ == "__main__":
    main()
