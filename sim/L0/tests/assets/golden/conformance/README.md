# Conformance Reference Data

Per-stage and trajectory reference data generated from **MuJoCo 3.4.0** for
Phase 12 conformance tests (§45 Layers B and C).

## Structure

```
conformance/
├── models/                    8 canonical MJCF models
│   ├── pendulum.xml           (a) nv=1, FK/CRBA/RNE/passive/integration
│   ├── double_pendulum.xml    (b) nv=2, FK chain/CRBA off-diagonal/Coriolis
│   ├── contact_scenario.xml   (c) nv=6, collision/constraint/solver
│   ├── actuated_system.xml    (d) nv=1 nu=2 na=1, actuator/activation dynamics
│   ├── tendon_model.xml       (e) nv=2, tendon length/velocity/passive force
│   ├── sensor_model.xml       (f) nv=2 nsensor=8, sensordata evaluation
│   ├── equality_model.xml     (g) nv=3, equality constraint assembly/solver
│   └── composite_model.xml    (h) nv=4, full pipeline interaction (Layer C)
├── reference/                 99 .npy files + 1 .json metadata
│   ├── reference_metadata.json
│   ├── {model}_fk_xpos.npy           FK body positions (nbody×3)
│   ├── {model}_fk_xquat.npy          FK body orientations (nbody×4)
│   ├── {model}_fk_xipos.npy          FK inertial frame positions (nbody×3)
│   ├── {model}_crba_qM.npy           Dense mass matrix (nv×nv)
│   ├── {model}_rne_qfrc_bias.npy     Coriolis+gravity bias (nv)
│   ├── {model}_passive_qfrc_passive.npy  Passive forces (nv)
│   ├── {model}_actuator_*.npy        Actuator forces (when nu>0)
│   ├── {model}_sensor_sensordata.npy Sensor data (when nsensor>0)
│   ├── {model}_tendon_*.npy          Tendon length/velocity (when ntendon>0)
│   ├── {model}_contact_*.npy         Contact pos/normal/depth/geom pairs
│   ├── {model}_constraint_efc_*.npy  Constraint Jacobian/bias/force
│   ├── {model}_trajectory_qpos.npy   Trajectory positions (N×nq)
│   ├── {model}_trajectory_qvel.npy   Trajectory velocities (N×nv)
│   └── {model}_trajectory_qacc.npy   Trajectory accelerations (N×nv)
└── README.md
```

## How to regenerate

```bash
uv pip install mujoco==3.4.0 numpy
uv run sim/L0/tests/scripts/gen_conformance_reference.py
```

The script is idempotent — running it twice produces bit-identical output.

## MuJoCo version pinning

Reference data **must** be generated with `mujoco==3.4.0`. The script asserts
the version at startup and aborts on mismatch. Do not regenerate with a
different version without updating all downstream conformance tests.

## Actuator control values

Models with actuators require specific `ctrl` values set before `forward()`:

| Model | ctrl values |
|-------|-------------|
| actuated_system | `[1.0, 0.5]` (motor + position servo) |
| composite_model | `[1.0]` (motor) |

These values are documented in `reference_metadata.json` and must match in
downstream Rust tests.

## File format

All `.npy` files use NumPy v1.0 format (little-endian float64, C-contiguous).
Contact geom pairs use int32. No `.npz` archives — individual files only, for
zero-dependency parsing in Rust via `parse_npy()`.

## File inventory

- 99 `.npy` reference data files
- 1 `reference_metadata.json` (model properties, shapes, checksums)
- Total size: ~436 KB
