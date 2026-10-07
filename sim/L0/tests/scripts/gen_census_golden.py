#!/usr/bin/env python3
"""Generate the parity-census golden data from the unfused MuJoCo 3.5.0 oracle.

Run it with the oracle's own interpreter (build_mujoco_oracle.sh builds it):

    <workdir>/venv/bin/python -I sim/L0/tests/scripts/gen_census_golden.py \\
        sim/L0/tests/assets/census/docs sim/L0/tests/assets/census/golden [ids-file]

It refuses an interpreter whose mujoco package libraries do not hash to the
marker build_mujoco_oracle.sh writes beside the venv: the PyPI wheel fuses
multiply-adds, so its numbers are not the numbers sim-core's plain arithmetic
should reach. It also refuses to write into a golden directory whose meta.json
names another oracle; moving the golden to a new oracle is a deliberate act
(delete meta.json first).

For every <docs>/<id>.xml (or every id in ids-file) it writes <golden>/<id>.json:

    {"doc", "status": "ok" | "refused", "model"?, "e1"?, "e2"?}
    eK = {"dump": {"0", "1", "100"}, "traj_steps", "traj": {"q", "v", "a", "t"}, "warn_end"}

"refused" carries no message: MuJoCo's message varies between runs for some
docs (which of two missing mesh files it names), and the golden must be
byte-stable. Non-finite numbers are the strings "inf", "-inf" and "nan",
because JSON has none. Without an ids-file it also writes <golden>/meta.json:
the oracle, the platform, the versions and the rules below, with no timestamp.

The rules are the census's, and layer_e_census.rs runs the same ones on our side:
- initial state: qpos = qpos0; qvel[i] = 0.1 (1 + i mod 3) under e1 and
  0.1 (1 + i mod 5) (-1)^i under e2; ctrl = lo + 0.625 (hi - lo) for a limited
  actuator, else 0; act = 0;
- 100 steps of mj_step; mj_forward dumps on a copy at steps 0, 1 and 100;
  qpos, qvel, act and time at 19 checkpoints (steps 1-10, 20, 30, ..., 100).
"""
import copy
import fnmatch
import hashlib
import json
import math
import os
import platform
import sys

import numpy as np
import mujoco

NSTEP = 100
DUMPS = (0, 1, 100)
CHECKPOINTS = list(range(1, 11)) + list(range(20, NSTEP + 1, 10))
FULL_QM_MAX_NV = 120
EXCITATIONS = {
    'e1': lambda i: 0.1 * (1 + i % 3),
    'e2': lambda i: 0.1 * (1 + i % 5) * (1 if i % 2 == 0 else -1),
}

# MuJoCo enum values under the names sim-core's Debug output uses; SENSOR keeps
# MuJoCo's mjSENS_ names, and compare.rs maps sim-core's names onto them.
INTEGRATOR = {0: 'Euler', 1: 'RungeKutta4', 2: 'Implicit', 3: 'ImplicitFast'}
SOLVER = {0: 'PGS', 1: 'CG', 2: 'Newton'}
JOINT = {0: 'Free', 1: 'Ball', 2: 'Slide', 3: 'Hinge'}
GEOM = {0: 'Plane', 1: 'Hfield', 2: 'Sphere', 3: 'Capsule', 4: 'Ellipsoid', 5: 'Cylinder',
        6: 'Box', 7: 'Mesh', 8: 'Sdf'}
TRANSMISSION = {0: 'Joint', 1: 'JointInParent', 2: 'SliderCrank', 3: 'Tendon', 4: 'Site', 5: 'Body'}
DYNAMICS = {0: 'None', 1: 'Integrator', 2: 'Filter', 3: 'FilterExact', 4: 'Muscle', 5: 'User'}
GAIN = {0: 'Fixed', 1: 'Affine', 2: 'Muscle', 3: 'User'}
BIAS = {0: 'None', 1: 'Affine', 2: 'Muscle', 3: 'User'}
EQUALITY = {0: 'Connect', 1: 'Weld', 2: 'Joint', 3: 'Tendon', 4: 'Flex', 5: 'FlexVert', 6: 'Distance'}
CONSTRAINT = {0: 'Equality', 1: 'FrictionLoss', 2: 'FrictionLoss', 3: 'LimitJoint', 4: 'LimitTendon',
              5: 'ContactFrictionless', 6: 'ContactPyramidal', 7: 'ContactElliptic'}
SENSOR = {int(getattr(mujoco.mjtSensor, n)): n[len('mjSENS_'):]
          for n in dir(mujoco.mjtSensor) if n.startswith('mjSENS_')}


def floats(a):
    return [float(x) for x in np.asarray(a, dtype=float).ravel()]


def rows(a, width):
    return [floats(r) for r in np.asarray(a, dtype=float).reshape(-1, width)]


def ints(a):
    return [int(x) for x in np.asarray(a).ravel()]


def model_json(m):
    op = m.opt
    inf = float('inf')
    body_I = []
    for b in range(m.nbody):
        r = np.zeros(9)
        mujoco.mju_quat2Mat(r, m.body_iquat[b])
        r = r.reshape(3, 3)
        body_I.append(floats(r @ np.diag(m.body_inertia[b]) @ r.T))
    return {
        'nq': m.nq, 'nv': m.nv, 'nu': m.nu, 'na': m.na, 'nbody': m.nbody, 'ngeom': m.ngeom,
        'njnt': m.njnt, 'neq': m.neq, 'nsite': m.nsite, 'ntendon': m.ntendon,
        'nsensor': m.nsensor, 'nsensordata': m.nsensordata, 'nflex': m.nflex,
        'nflexvert': m.nflexvert, 'nflexedge': m.nflexedge, 'nmocap': m.nmocap,
        'timestep': float(op.timestep), 'gravity': floats(op.gravity),
        'integrator': INTEGRATOR[int(op.integrator)], 'solver': SOLVER[int(op.solver)],
        'iterations': int(op.iterations), 'tolerance': float(op.tolerance), 'cone': int(op.cone),
        'impratio': float(op.impratio), 'disableflags': int(op.disableflags),
        'enableflags': int(op.enableflags), 'ls_iterations': int(op.ls_iterations),
        'ls_tolerance': float(op.ls_tolerance), 'noslip_iterations': int(op.noslip_iterations),
        'noslip_tolerance': float(op.noslip_tolerance), 'density': float(op.density),
        'viscosity': float(op.viscosity), 'wind': floats(op.wind), 'magnetic': floats(op.magnetic),
        'o_margin': float(op.o_margin), 'o_solref': floats(op.o_solref),
        'o_solimp': floats(op.o_solimp), 'o_friction': floats(op.o_friction),
        'ccd_iterations': int(op.ccd_iterations), 'ccd_tolerance': float(op.ccd_tolerance),
        'sleep_tolerance': float(op.sleep_tolerance), 'meaninertia': float(m.stat.meaninertia),
        'qpos0': floats(m.qpos0), 'qpos_spring': floats(m.qpos_spring),
        'body_parent': ints(m.body_parentid), 'body_pos': rows(m.body_pos, 3),
        'body_quat': rows(m.body_quat, 4), 'body_ipos': rows(m.body_ipos, 3),
        'body_iquat': rows(m.body_iquat, 4), 'body_mass': floats(m.body_mass),
        'body_inertia': rows(m.body_inertia, 3), 'body_gravcomp': floats(m.body_gravcomp),
        'body_mocapid': ints(m.body_mocapid), 'body_I': body_I,
        'jnt_type': [JOINT[int(t)] for t in m.jnt_type], 'jnt_body': ints(m.jnt_bodyid),
        'jnt_qposadr': ints(m.jnt_qposadr), 'jnt_dofadr': ints(m.jnt_dofadr),
        'jnt_pos': rows(m.jnt_pos, 3), 'jnt_axis': rows(m.jnt_axis, 3),
        'jnt_limited': ints(m.jnt_limited), 'jnt_range': rows(m.jnt_range, 2),
        'jnt_stiffness': floats(m.jnt_stiffness), 'jnt_margin': floats(m.jnt_margin),
        'jnt_solref': rows(m.jnt_solref, 2), 'jnt_solimp': rows(m.jnt_solimp, 5),
        'jnt_actgravcomp': ints(m.jnt_actgravcomp),
        'dof_body': ints(m.dof_bodyid), 'dof_damping': floats(m.dof_damping),
        'dof_armature': floats(m.dof_armature), 'dof_frictionloss': floats(m.dof_frictionloss),
        'dof_solref': rows(m.dof_solref, 2), 'dof_solimp': rows(m.dof_solimp, 5),
        'geom_type': [GEOM[int(t)] for t in m.geom_type], 'geom_body': ints(m.geom_bodyid),
        'geom_pos': rows(m.geom_pos, 3), 'geom_quat': rows(m.geom_quat, 4),
        'geom_size': rows(m.geom_size, 3), 'geom_friction': rows(m.geom_friction, 3),
        'geom_condim': ints(m.geom_condim), 'geom_contype': ints(m.geom_contype),
        'geom_conaffinity': ints(m.geom_conaffinity), 'geom_margin': floats(m.geom_margin),
        'geom_gap': floats(m.geom_gap), 'geom_priority': ints(m.geom_priority),
        'geom_solmix': floats(m.geom_solmix), 'geom_solref': rows(m.geom_solref, 2),
        'geom_solimp': rows(m.geom_solimp, 5),
        'actuator_trntype': [TRANSMISSION.get(int(t), str(int(t))) for t in m.actuator_trntype],
        'actuator_dyntype': [DYNAMICS[int(t)] for t in m.actuator_dyntype],
        'actuator_gaintype': [GAIN[int(t)] for t in m.actuator_gaintype],
        'actuator_biastype': [BIAS[int(t)] for t in m.actuator_biastype],
        'actuator_trnid': [int(r[0]) for r in m.actuator_trnid],
        'actuator_gear': rows(m.actuator_gear, 6),
        'actuator_ctrlrange': [floats(m.actuator_ctrlrange[i]) if m.actuator_ctrllimited[i]
                               else [-inf, inf] for i in range(m.nu)],
        'actuator_forcerange': [floats(m.actuator_forcerange[i]) if m.actuator_forcelimited[i]
                                else [-inf, inf] for i in range(m.nu)],
        'actuator_actlimited': ints(m.actuator_actlimited),
        'actuator_actrange': rows(m.actuator_actrange, 2),
        'actuator_actearly': ints(m.actuator_actearly),
        'actuator_dynprm': rows(m.actuator_dynprm, 10),
        'actuator_gainprm': [floats(r[:9]) for r in m.actuator_gainprm],
        'actuator_biasprm': [floats(r[:9]) for r in m.actuator_biasprm],
        'actuator_lengthrange': rows(m.actuator_lengthrange, 2),
        'actuator_acc0': floats(m.actuator_acc0), 'actuator_actnum': ints(m.actuator_actnum),
        'actuator_delay': floats(m.actuator_delay),
        'actuator_nsample': [int(r[0]) for r in m.actuator_history],
        'tendon_limited': ints(m.tendon_limited), 'tendon_range': rows(m.tendon_range, 2),
        'tendon_stiffness': floats(m.tendon_stiffness), 'tendon_damping': floats(m.tendon_damping),
        'tendon_lengthspring': rows(m.tendon_lengthspring, 2),
        'tendon_frictionloss': floats(m.tendon_frictionloss),
        'tendon_margin': floats(m.tendon_margin), 'tendon_num': ints(m.tendon_num),
        'tendon_length0': floats(m.tendon_length0),
        'tendon_solref_lim': rows(m.tendon_solref_lim, 2),
        'tendon_solimp_lim': rows(m.tendon_solimp_lim, 5),
        'eq_type': [EQUALITY[int(t)] for t in m.eq_type], 'eq_obj1id': ints(m.eq_obj1id),
        'eq_obj2id': ints(m.eq_obj2id),
        'eq_data': rows(m.eq_data, m.eq_data.shape[1] if m.neq else 11),
        'eq_solref': rows(m.eq_solref, 2), 'eq_solimp': rows(m.eq_solimp, 5),
        'eq_active': ints(m.eq_active0),
        'sensor_type': [SENSOR.get(int(t), str(int(t))) for t in m.sensor_type],
        'sensor_dim': ints(m.sensor_dim), 'sensor_adr': ints(m.sensor_adr),
        'sensor_objid': ints(m.sensor_objid), 'sensor_cutoff': floats(m.sensor_cutoff),
        'sensor_noise': floats(m.sensor_noise), 'sensor_delay': floats(m.sensor_delay),
        'sensor_nsample': [int(r[0]) for r in m.sensor_history],
        'flex_dim': ints(m.flex_dim), 'flex_vertnum': ints(m.flex_vertnum),
        'flex_edgenum': ints(m.flex_edgenum), 'flex_elemnum': ints(m.flex_elemnum),
        'flexvert_bodyid': ints(m.flex_vertbodyid),
    }


def data_json(m, d):
    out = {'time': float(d.time), 'qpos': floats(d.qpos), 'qvel': floats(d.qvel),
           'act': floats(d.act), 'xpos': rows(d.xpos, 3), 'xquat': rows(d.xquat, 4)}
    if m.nv <= FULL_QM_MAX_NV:
        full = np.zeros((m.nv, m.nv))
        mujoco.mj_fullM(m, full, d.qM)
        out['qM_full'], out['qM'] = 1, floats(full)
    else:
        out['qM_full'], out['qM'] = 0, [float(d.qM[m.dof_Madr[i]]) for i in range(m.nv)]
    contacts = []
    c = d.contact
    for i in range(d.ncon):
        geom, flex, vert = ints(c.geom[i]), ints(c.flex[i]), ints(c.vert[i])
        flexvert = [int(m.flex_vertadr[flex[j]] + vert[j]) if flex[j] >= 0 and vert[j] >= 0 else -1
                    for j in range(2)]
        frame = floats(c.frame[i])
        contacts.append([geom[0], geom[1], flexvert[0], flexvert[1], float(c.dist[i]),
                         floats(c.pos[i]), frame[0:3], frame[3:6], frame[6:9]])
    counts = {}
    for t in d.efc_type[:d.nefc]:
        kind = CONSTRAINT[int(t)]
        counts[kind] = counts.get(kind, 0) + 1
    out.update(ncon=int(d.ncon), con=contacts, nefc=int(d.nefc), efc_counts=counts)
    for f in ('qfrc_bias', 'qfrc_passive', 'qfrc_actuator', 'qacc_smooth', 'qfrc_constraint',
              'qacc', 'sensordata'):
        out[f] = floats(getattr(d, f))
    out['warn'] = [int(w.number) for w in d.warning]
    return out


def forward_dump(m, d):
    c = copy.copy(d)
    mujoco.mj_forward(m, c)
    return data_json(m, c)


def run_excitation(m, qvel_of):
    d = mujoco.MjData(m)
    d.qpos[:] = m.qpos0
    for i in range(m.nv):
        d.qvel[i] = qvel_of(i)
    for i in range(m.nu):
        lo, hi = m.actuator_ctrlrange[i]
        d.ctrl[i] = lo + 0.625 * (hi - lo) if m.actuator_ctrllimited[i] else 0.0
    dumps = {'0': forward_dump(m, d)}
    traj = {'q': [], 'v': [], 'a': [], 't': []}
    for k in range(1, NSTEP + 1):
        mujoco.mj_step(m, d)
        if k in CHECKPOINTS:
            traj['q'].append(floats(d.qpos))
            traj['v'].append(floats(d.qvel))
            traj['a'].append(floats(d.act))
            traj['t'].append(float(d.time))
        if k in DUMPS:
            dumps[str(k)] = forward_dump(m, d)
    return {'dump': dumps, 'traj_steps': CHECKPOINTS, 'traj': traj,
            'warn_end': [int(w.number) for w in d.warning]}


def finite_or_string(o):
    if isinstance(o, float) and not math.isfinite(o):
        return 'nan' if o != o else ('inf' if o > 0 else '-inf')
    if isinstance(o, list):
        return [finite_or_string(x) for x in o]
    if isinstance(o, dict):
        return {k: finite_or_string(v) for k, v in o.items()}
    return o


def golden(path):
    doc = os.path.basename(path)[:-len('.xml')]
    with open(path, encoding='utf-8') as f:
        text = f.read()
    try:
        m = mujoco.MjModel.from_xml_string(text)
    except Exception:  # noqa: BLE001 — any refusal is the same verdict: status only
        return {'doc': doc, 'status': 'refused'}
    out = {'doc': doc, 'status': 'ok', 'model': model_json(m)}
    for name, qvel_of in EXCITATIONS.items():
        out[name] = run_excitation(m, qvel_of)
    return finite_or_string(out)


def libraries_sha256(pkg):
    """build_mujoco_oracle.sh's hash: sha256 over "<path in pkg>\\t<sha256>\\n" per library."""
    lines = []
    for root, _, files in os.walk(pkg):
        for name in files:
            path = os.path.join(root, name)
            if os.path.islink(path) or not any(
                    fnmatch.fnmatch(name, g) for g in ('*.dylib', '*.so', '*.so.*')):
                continue
            with open(path, 'rb') as f:
                lines.append((os.path.relpath(path, pkg), hashlib.sha256(f.read()).hexdigest()))
    text = ''.join(f'{rel}\t{digest}\n' for rel, digest in sorted(lines))
    return hashlib.sha256(text.encode()).hexdigest()


def oracle():
    """The marker build_mujoco_oracle.sh writes beside the venv, checked against this run."""
    marker = os.path.join(os.path.dirname(sys.prefix), 'oracle.json')
    if not os.path.exists(marker):
        sys.exit(f'{sys.executable} is not the unfused oracle (no {marker}); '
                 'build it with build_mujoco_oracle.sh')
    with open(marker) as f:
        o = json.load(f)
    here = f'{platform.system()}-{platform.machine()}'
    if o['mujoco'] != mujoco.__version__ or o['platform'] != here:
        sys.exit(f'oracle marker {o} does not match mujoco {mujoco.__version__} on {here}')
    loaded = libraries_sha256(os.path.dirname(mujoco.__file__))
    if o.get('libraries_sha256') != loaded:
        sys.exit(f'the mujoco this interpreter loads ({loaded}) is not the build the marker '
                 f'{marker} records ({o.get("libraries_sha256")})')
    return o


def main():
    if len(sys.argv) not in (3, 4):
        sys.exit(__doc__)
    docs, out = sys.argv[1], sys.argv[2]
    built = oracle()
    meta_path = os.path.join(out, 'meta.json')
    if os.path.exists(meta_path):
        with open(meta_path) as f:
            blessed = json.load(f)['oracle']
        if blessed != built:
            sys.exit(f'{out} was blessed by the oracle {blessed}; this one is {built}')
    os.makedirs(out, exist_ok=True)
    if len(sys.argv) == 4:
        with open(sys.argv[3]) as f:
            ids = [line.strip() for line in f if line.strip()]
    else:
        ids = sorted(f[:-len('.xml')] for f in os.listdir(docs) if f.endswith('.xml'))
    for doc in ids:
        record = golden(os.path.join(docs, doc + '.xml'))
        with open(os.path.join(out, doc + '.json'), 'w') as f:
            f.write(json.dumps(record, separators=(',', ':')) + '\n')
    if len(sys.argv) == 3:
        meta = {
            'oracle': built, 'numpy': np.__version__, 'python': platform.python_version(),
            'nstep': NSTEP, 'dumps': list(DUMPS), 'traj_steps': CHECKPOINTS,
            'excitations': {'e1': 'qvel_i = 0.1 (1 + i mod 3)',
                            'e2': 'qvel_i = 0.1 (1 + i mod 5) (-1)^i'},
            'ctrl': 'lo + 0.625 (hi - lo) if ctrllimited else 0', 'qpos': 'qpos0', 'act': '0',
        }
        with open(meta_path, 'w') as f:
            f.write(json.dumps(meta, indent=1, sort_keys=True) + '\n')
    print(f'{len(ids)} docs -> {out}')


if __name__ == '__main__':
    main()
