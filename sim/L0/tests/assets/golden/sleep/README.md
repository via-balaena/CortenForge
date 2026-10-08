# Sleep golden

`sleep.json` holds MuJoCo 3.5.0's kinematic-tree tables, resolved automatic
sleep policies, body sleep states and `qfrc_gravcomp` for the models
`integration/sleep_parity.rs` checks.

It comes from the unfused oracle, MuJoCo 3.5.0 built without fused
multiply-adds (`scripts/build_mujoco_oracle.sh`), which the generator
requires, as `gen_census_golden.py` does:

```sh
<workdir>/venv/bin/python -I sim/L0/tests/scripts/gen_sleep_reference.py \
    sim/L0/tests/assets/golden/sleep
```

The file is byte-stable across runs; its `oracle` entry is the oracle's
marker. The generator's docstring describes the models.
