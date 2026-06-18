#!/usr/bin/env python3
"""Generate golden forward-dynamics reference values from MuJoCo.

This is the PINNED oracle for the forward-conformance harness: for each MJCF
fixture and each (qpos, qvel) sample, it records MuJoCo's forward-dynamics
outputs (body world poses, mass matrix, bias force, acceleration). The Rust test
`forward_conformance.rs` loads the SAME MJCF via `mjcf::load_model`, runs
sim-core's forward, and asserts it matches these numbers — externally anchoring
the port to its reference.

The golden values come from MuJoCo, NOT from sim-core, so they only change when a
fixture/sample changes or MuJoCo's version bumps — never when sim-core changes.
That defeats the usual golden-snapshot rot (you cannot make a real divergence
pass by regenerating; regen reproduces the same numbers).

Run (pinned, reproducible):
    uv run --with 'mujoco==3.9.0' sim/L0/mjcf/tests/conformance/gen_golden.py

The nightly drift guard runs the same script against the LATEST mujoco and diffs
the result against the checked-in golden to detect upstream behavior drift.

Compared quantities are convention-independent: world poses (`xpos`, `xmat`) and
generalized quantities (`qM`, `qfrc_bias`, `qacc`). `cvel` is deliberately NOT
compared — MuJoCo references it at the subtree COM and sim-core at the body
origin, so a raw diff would be a false mismatch.
"""

import json
import pathlib

import mujoco
import numpy as np

HERE = pathlib.Path(__file__).parent
FIXTURES = HERE / "fixtures"
GOLDEN = HERE / "golden"

# (qpos, qvel) samples per fixture (by stem). Kept here as the single source of
# truth for the operating points; the Rust test reads them back from the golden
# JSON, so there is no duplication across languages.
SAMPLES = {
    "single_hinge": [
        ([0.3], [0.8]),
        ([-0.5], [-0.4]),
    ],
    "two_link_hinge": [
        ([0.3, -0.35], [0.8, -0.6]),
        ([-0.5, 0.7], [-0.4, 0.9]),
        ([1.1, -0.9], [0.0, 0.0]),  # zero-velocity: isolates gravity + mass
    ],
}


def gen_fixture(stem: str) -> dict:
    xml = (FIXTURES / f"{stem}.xml").read_text()
    model = mujoco.MjModel.from_xml_string(xml)
    data = mujoco.MjData(model)
    nv = model.nv

    samples = []
    for qpos, qvel in SAMPLES[stem]:
        assert len(qpos) == model.nq, f"{stem}: qpos len {len(qpos)} != nq {model.nq}"
        assert len(qvel) == nv, f"{stem}: qvel len {len(qvel)} != nv {nv}"
        data.qpos[:] = qpos
        data.qvel[:] = qvel
        data.ctrl[:] = 0.0
        mujoco.mj_forward(model, data)

        dense_m = np.zeros((nv, nv), dtype=np.float64)
        mujoco.mj_fullM(model, dense_m, data.qM)

        samples.append(
            {
                "qpos": list(qpos),
                "qvel": list(qvel),
                # nbody x 3 (world body included at index 0)
                "xpos": data.xpos.tolist(),
                # nbody x 9 (row-major 3x3 rotation), avoids quaternion double-cover
                "xmat": data.xmat.tolist(),
                "qM": dense_m.tolist(),
                "qfrc_bias": data.qfrc_bias.tolist(),
                "qacc": data.qacc.tolist(),
            }
        )

    return {
        "fixture": stem,
        "mujoco_version": mujoco.__version__,
        "nq": int(model.nq),
        "nv": int(model.nv),
        "nbody": int(model.nbody),
        "samples": samples,
    }


def main() -> None:
    GOLDEN.mkdir(exist_ok=True)
    for xml_path in sorted(FIXTURES.glob("*.xml")):
        stem = xml_path.stem
        if stem not in SAMPLES:
            raise SystemExit(f"fixture {stem} has no SAMPLES entry")
        out = gen_fixture(stem)
        dest = GOLDEN / f"{stem}.json"
        dest.write_text(json.dumps(out, indent=2) + "\n")
        print(f"wrote {dest.relative_to(HERE)}  ({len(out['samples'])} samples)")


if __name__ == "__main__":
    main()
