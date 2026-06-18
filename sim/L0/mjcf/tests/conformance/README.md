# Forward-dynamics conformance harness (vs MuJoCo)

sim-core is a port of MuJoCo. This harness certifies that its **forward
dynamics** match MuJoCo's across a matrix of joint types × topologies, using
MuJoCo as an external golden-value oracle. It is the forward-side complement to
the transition-derivative harness (`sim-core` `transition_matrix_harness`), which
validates *derivatives* against finite differences of the same forward — and so
structurally cannot catch a bug in the forward itself. Only an external oracle
can; that is what this harness is for.

## Layout

- `fixtures/*.xml` — MJCF models (the joint × topology matrix). The **single
  source of truth**: both engines parse the identical file, so there is no
  model-translation drift.
- `gen_golden.py` — loads each fixture in MuJoCo, runs `mj_forward` at fixed
  `(qpos, qvel)` samples, and writes the reference outputs to `golden/*.json`.
- `golden/*.json` — checked-in MuJoCo reference values (`xpos`, `xmat`, `qM`,
  `qfrc_bias`, `qacc`) per sample. The `(qpos, qvel)` samples live here too, so
  the Rust test reads them back — single source, no cross-language duplication.
- `../forward_conformance.rs` — the Rust test: loads each fixture via
  `mjcf::load_model`, runs sim-core's forward, asserts agreement.

Compared quantities are **convention-independent**: world poses (`xpos`, `xmat`)
and generalized quantities (`qM`, `qfrc_bias`, `qacc`). `cvel` is deliberately
**not** compared — MuJoCo references it at the subtree COM, sim-core at the body
origin, so a raw diff would be a false mismatch.

## Why golden values (not live MuJoCo in CI)

The golden values are **MuJoCo's** output, not sim-core's, so they change only
when a fixture/sample or MuJoCo's version changes — never when sim-core changes.
That defeats the usual golden-snapshot rot: a real divergence cannot be
"re-blessed" away, because regenerating reproduces the same numbers and the test
still fails until sim-core is fixed. PR CI stays hermetic (no MuJoCo, no Python,
cross-OS clean) and pinned to a known-good reference.

## Regenerating golden (pinned)

```sh
uv run --with 'mujoco==3.9.0' sim/L0/mjcf/tests/conformance/gen_golden.py
```

Only needed when fixtures/samples change or when deliberately bumping the pin.
Regeneration must be a reviewed change to the checked-in `golden/*.json`.

## Nightly drift guard (planned)

A scheduled CI job installs the **latest** MuJoCo, regenerates golden, and runs
the conformance suite against the fresh values — failing (alert only, never
auto-committing) if MuJoCo's behavior has drifted from the checked-in pin. This
is what catches staleness: pinned-vs-pinned regeneration is deterministic and
cannot drift, so detecting "a newer MuJoCo diverged from our frozen numbers"
requires floating to latest. Re-pinning + regeneration is always a deliberate,
reviewed step.

## Known divergences

Documented, regression-guarded gaps where sim-core does not yet match MuJoCo live
in `known_coriolis_divergence` in the test, each with a root cause and the
follow-on stone that closes it (mirroring the transition harness's
`known_limit`). Everything else is held machine-exact.
