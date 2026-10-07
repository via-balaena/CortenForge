# Verification

A6 §4 is the protocol; the rules it enforces:

- **Before the first commit**: baseline on `main` — corpus harness ×10 (the determinism mask, 71 docs), repo and submodule `.xml` (streaming-fingerprint harness only; a `{:#?}` fingerprint of `fourier_n1` is 2.39 GB of text), MuJoCo 3.5.0 oracle, `cargo xtask licensed-gates --run` (main carried 4 red gates when last run, 2026-09-21: a different set is a new baseline, not a regression), the 24 ignored golden-flag tests' residuals, `--features mjb`, `--no-default-features`, the full validator fleet.
- **Per MJCF commit**: the five buckets (unchanged · ok→ok changed must be on the commit's pre-registered expected-change list · ok→err must map to the commit's rule · err→ok must be MuJoCo-ok · err→err message change), and NEW NONDETERMINISTIC empty from M3 on. Compare MuJoCo by status, not message (its message varies for 2 docs).
- **Per commit, every rule**: its must-fail test made to fail at the parent, written down before the code.
- **Runtime-generated MJCF** (157 `format!` templates, cf-design, cf-mjcf-emit, sim-urdf, therm-env, cf-codesign, cf-fsu-model, cf-osim): capture in a detached scratch worktree with an uncommitted dump patch; the push guard `grep -c CF-SCRATCH-MJCF-DUMP` must print 0 on the PR diff, after being made to print ≥ 1 on the worktree.
- **Oracle**: every new refusal is MuJoCo-refused or on the divergences list; no MuJoCo-ok doc newly fails unless listed.
- **Before push**: grade every touched crate and its downstream (`RAYON_NUM_THREADS=1`), clippy exactly as the hook runs it, doc-theft, licensed gates at the head, the review rounds.
