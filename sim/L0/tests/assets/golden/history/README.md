# History-buffer (delay) golden

`history.json` holds MuJoCo 3.5.0's actuator and sensor history buffers,
forces, states and sensor values for the cases
`integration/history.rs` runs: actuators and sensors with buffers, delays,
intervals and each interpolation, under Euler, RK4, implicitfast and
implicit, with `mj_step`, `mj_forward` and `mj_step1` + `mj_step2`, a reset
part-way and keyframes at positive and negative times. History buffers are
new in MuJoCo 3.5.0, so this golden has no 3.4.0 counterpart.

It comes from the unfused oracle, MuJoCo 3.5.0 built without fused
multiply-adds (`scripts/build_mujoco_oracle.sh`), which the generator
requires, as `gen_census_golden.py` does:

```sh
<workdir>/venv/bin/python -I sim/L0/tests/scripts/gen_history_reference.py \
    sim/L0/tests/assets/golden/history/history.json
```

The file is byte-stable across runs; its `oracle` entry is the oracle's
marker. The generator's docstring describes the models and drivers.
