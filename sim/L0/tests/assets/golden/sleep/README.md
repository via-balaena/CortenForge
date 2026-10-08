# Sleep golden

`sleep.json` holds MuJoCo 3.5.0's kinematic-tree tables, resolved automatic
sleep policies, body sleep states, `qfrc_gravcomp`, `dof_length`, the sleep
state after each step of short runs, and the messages MuJoCo refuses three
models with; `sleep_traces.json` holds, after the reset and after each step
of longer runs, the sleep state, contact, row and island counts, the
callbacks that fired, and the velocities, accelerations and sensors near
each change of a tree's sleep state. `integration/sleep_parity.rs` checks
both.

They come from the unfused oracle, MuJoCo 3.5.0 built without fused
multiply-adds (`scripts/build_mujoco_oracle.sh`), which the generator
requires, as `gen_census_golden.py` does:

```sh
<workdir>/venv/bin/python -I sim/L0/tests/scripts/gen_sleep_reference.py \
    sim/L0/tests/assets/golden/sleep
```

Both files are byte-stable across runs; their `oracle` entry is the oracle's
marker. The generator's docstring describes the models and runs.
