# Phase 0 — Track A Bundle Scope (SD-4 + sim-mjcf validate() wiring)

**Status:** Scope memo — first PR of the six-phase foundation-up sequence. Pre-code recon complete; ready to code.
**Date:** 2026-04-24, post PR #215 (`c7c79d23`) — SD-5 chassis→mjcf audit MERGED.
**Branch:** `feature/phase-0-track-a-bundle`, off main `c7c79d23`.
**Follows:** `project_gameplan.md` (memory) — Phase 0 of the six-phase foundation-up sequence.
**Target:** ~30–80 LOC across `.github/workflows/quality-gate.yml` + `sim/L0/mjcf/src/builder/mod.rs` + one new test in `sim/L0/mjcf/src/lib.rs`. One PR.

Phase 0 is pure plumbing — no domain risk, no new physics. It unblocks regression detection on `sim-soft` (and 5 other omitted L0 crates) for every subsequent C1 phase, and turns `sim-mjcf::validate` from defense-in-depth dead code into an active ingress gate. Both changes are bundled in one PR because each is small and they share the "infra cleanup" framing.

**Why this matters under the sharpened scope (2026-04-24):** CortenForge's unifying primitive is *soft-body payloads on rigid (or rigid+soft hybrid) mechanisms*. `sim-mjcf` is the rigid backbone of every hybrid mechanism (products + fab machines, recursively); strengthening its ingress validation is foundational. `sim-soft` is the soft-payload primitive set; CI regression detection on it gates every C1 chunk that follows. Phase 0 hardens both halves of that bridge before C1 starts extending either.

## 0. Baseline — what's currently in place

**CI test matrix** (`.github/workflows/quality-gate.yml:128-134`): hardcoded `cargo test --all-features -p <list>` covering `mesh-*` (10 crates) + `sim-types`/`sim-simd`/`sim-core`/`sim-mjcf`/`sim-urdf`/`sim-conformance-tests`/`sim-gpu` + `cf-*` (3 crates). **Omitted L0 crates:** `sim-soft`, `sim-ml-chassis`, `sim-rl`, `sim-thermostat`, `sim-opt`, `sim-therm-env`. The `grade` job's `cargo xtask grade-all` covers per-crate clippy `--all-features` (so build graphs are exercised) but per-crate `cargo test` inside `grade.rs` runs **default features only** — so `gpu-probe` tests do not execute in any CI surface today.

**sim-mjcf validate()** (`sim/L0/mjcf/src/validation.rs:156`): `pub fn validate(model: &MjcfModel) -> Result<ValidationResult>` has zero production callers. `model_from_mjcf` (`builder/mod.rs:224`) calls only `validate_tendons` (line 292). `load_model` (line 359) and `load_model_from_file` (line 387) inherit that gap. The `ValidationResult` returned on success is a topology struct (`sorted_bodies`, `body_parent`, `body_children`, `joint_names`, `joint_to_body`, `actuator_names`) — no `errors`/`warnings` field; everything is fail-on-`Err`. Wire-in discards `Ok(ValidationResult)` and propagates `Err` only.

**What validate() rejects** (sweep verified pre-code):
- Duplicate body / joint / actuator names
- `inertial.mass <= 0.0` or non-finite (only when `body.inertial` is `Some(_)`)
- `inertial.diaginertia` negative or non-finite
- `geom.size` or `geom.fromto` non-finite (`check_geom_finite`, line 378 — does NOT check geom mass)
- Actuator references undefined joint / site / refsite
- Actuator with multiple transmission targets (joint + tendon + site + body)
- `validate_option` violations: timestep, iteration counts, tolerance, impedance ratio, density/viscosity (line 41)

## 1. The three load-bearing invariants

| # | Invariant | What green looks like |
|---|---|---|
| I-A | sim-soft regressions fire in CI on every PR | `cargo test --all-features -p sim-soft` runs in the test matrix on all three OS entries; a deliberately-broken sim-soft test fails the PR gate. Same coverage extends to the other 5 newly-added L0 crates. |
| I-B | gpu-probe path runs in CI, not just locally | `--all-features` picks up `gpu-probe` for sim-soft; probe's skip-clean branch (skeleton scope §1 I-6) handles headless ubuntu / windows runners; macos most likely to actually acquire an adapter. |
| I-C | validate() actively gates MJCF ingress | Every call site that builds a `Model` from MJCF runs the full `validate` checks before any model construction. A fixture with a duplicate body name, non-finite mass, or non-finite geom dimensions fails to load — does not silently produce a corrupt model. |

## 2. The two changes — concrete diffs

### Change 1: SD-4 — extend CI test matrix

`.github/workflows/quality-gate.yml`, replace lines 128-134's hardcoded crate list with (12-space indent on `-p` lines):

```diff
             cargo test --all-features \
               -p mesh-types -p mesh-io -p mesh-repair -p mesh-sdf \
               -p mesh-offset -p mesh-shell -p mesh-measure \
               -p mesh-printability -p mesh-lattice -p mesh \
               -p sim-types -p sim-simd -p sim-core -p sim-mjcf \
               -p sim-urdf -p sim-conformance-tests -p sim-gpu \
+              -p sim-soft -p sim-ml-chassis -p sim-rl \
+              -p sim-thermostat -p sim-opt -p sim-therm-env \
               -p cf-geometry -p cf-spatial -p cf-design
```

Six new `-p` flags. No other workflow change. `--all-features` is already in place — picks up `gpu-probe` automatically for sim-soft. **One known compile-cost item:** sim-ml-chassis has `bevy = ["dep:bevy_ecs"]` opt-in feature; `--all-features` will compile bevy_ecs into the test binary. This is a build-time cost only, not a runtime concern, and not a Bevy-free violation (Bevy-free is a grade-job concern, separate from the test job).

### Change 2: validate() wiring into `model_from_mjcf`

`sim/L0/mjcf/src/builder/mod.rs:228-229`, **insert immediately after the clone, before any expansion pass:**

```diff
 pub fn model_from_mjcf(
     mjcf: &MjcfModel,
     base_path: Option<&Path>,
 ) -> std::result::Result<Model, ModelConversionError> {
     // Apply compiler pre-processing passes on a mutable clone
     let mut mjcf = mjcf.clone();

+    // Active ingress validation (Phase 0): full validate() promoted from
+    // defense-in-depth dead code. Fail-closed on any finding. Runs against
+    // user input as-parsed, before frame/composite expansion.
+    crate::validation::validate(&mjcf).map_err(|e| ModelConversionError {
+        message: format!("Model validation failed: {e}"),
+    })?;
+
     // Validate childclass references before frame expansion dissolves frames.
```

That's the entire code change for I-C — one call, fail-closed, `Ok(ValidationResult)` discarded. `load_model` and `load_model_from_file` inherit it because both call `model_from_mjcf` internally.

### Change 3: new end-to-end test

`sim/L0/mjcf/src/lib.rs`, add a test next to existing `test_validation_errors` at line ~282:

```rust
#[test]
fn test_validate_wired_into_model_from_mjcf() {
    // Ingress gate test: duplicate body name should fail at load_model,
    // not produce a corrupt Model. Uses the parsed-XML path end-to-end.
    let mjcf = r#"
        <mujoco model="dup">
            <worldbody>
                <body name="a"><geom type="sphere" size="0.1" mass="1.0"/></body>
                <body name="a"><geom type="sphere" size="0.1" mass="1.0"/></body>
            </worldbody>
        </mujoco>
    "#;
    let result = load_model(mjcf);
    assert!(matches!(
        result,
        Err(crate::error::MjcfError::Unsupported(msg)) if msg.contains("validation failed")
    ));
}
```

One representative test exercising the wire-in path. The per-validator behavior (mass, geom-finite, refs, etc.) is already covered in `validation.rs`'s test mod; this one only confirms wire-in.

## 3. Decision: validate() wiring policy (locked)

**Option 1: wire unconditionally into `model_from_mjcf`, fail-closed.** All checks in `validate` are silent-corruption-on-pass — none has warn-only semantics. `ValidationResult` is a topology summary, not a warnings carrier, so option 3 (lenient default) would leave validate mostly-dead and option 2 (split block/warn) collapses to option 1 once you audit the checks. Pre-code fixture sweep (§6) found zero workspace fixtures that trip the new gate, so option 1 ships clean.

**Placement: option (a), right after the clone (line 229), before any expansion pass.** Pre-code analysis of `expand_frames` / `expand_composites` / `apply_discardvisual` / `apply_fusestatic` confirmed none generates state that validate would reject — composite body-name auto-generation uses unique-prefix naming, frame expansion preserves bodies/inertials, discardvisual only removes geoms, fusestatic merges inertials cleanly. Validating user input as-parsed gives clearer error messages than validating post-mutation state.

## 4. Test harness — see §6 for verification mechanics

## 5. Green checklist

- [ ] `cargo xtask grade sim-soft` A under `integration-only` profile (Coverage + Bevy-free N/A) — establishes baseline.
- [ ] `cargo xtask grade sim-mjcf` A across all 7 — must hold after wire-in.
- [ ] CI matrix on commit 1: all 6 newly-added crates run cleanly on ubuntu / macos / windows.
- [ ] CI matrix on commit 1: gpu-probe test on macos shows pass OR clean skip; ubuntu/windows show clean skip.
- [ ] CI matrix on commit 2: zero fixture rejections from validate wire-in (sweep predicts zero).
- [ ] If commit 2 shows fixture rejections: categorize per §6 R-1 and fix forward in same PR.
- [ ] Workspace `cargo test` not run locally (per CLAUDE.md "NEVER run cargo test full workspace" rule); CI is the verification surface.
- [ ] Pre-squash tag — Phase 0 has no chapter hash anchors and ≤3 commits; **skip the pre-squash tag** unless commit count grows beyond 5.
- [ ] Platform-agility invariant: Phase 0 modifies sim-mjcf for foundational hardening (active gate vs dead code), not domain extension — clear.

## 6. Stress-test rounds (most executed pre-code via recon)

| # | Round | Status | Result / Hook |
|---|---|---|---|
| R-1 | Workspace fixture-corpus blast radius | **Done pre-code** | Explore-agent sweep + targeted greps: ~2,100 callers across workspace (1,353 in sim-tests, ~200 in examples, rest distributed); zero `<inertial mass="0">` patterns; zero tests asserting on validate-domain `MjcfError` variants directly. Predicted fixture failure count: **zero** (high confidence). CI on commit 2 is the live confirmation. |
| R-2 | sim-therm-env generator output | **Done pre-code** | Generator hardcodes `mass="1"` (sim/L0/therm-env/src/builder.rs:42); options/timestep set from validated builder params. Cannot produce validate-rejected output. |
| R-3 | gpu-probe headless behavior on Linux/Windows CI | Live in CI on commit 1 | Trust GitHub Actions ubuntu/windows runners; observe `gpu_probe` test result. Skip-clean per skeleton scope §1 I-6. |
| R-4 | Test matrix runtime impact | Live in CI on commit 1 | Adding 6 crates to `cargo test --all-features`; sim-ml-chassis pulls bevy_ecs as compile cost. If windows wall time slips beyond current ~20 min by more than 5 min, action: split test job into fast + slow matrix entries with separate timeouts. |
| R-5 | sim-mjcf grade after wire-in | Live locally pre-push commit 2 | `cargo xtask grade sim-mjcf` must hold A across all 7 with the wire-in applied. |
| R-6 | Cross-PR ordering | N/A | Phase 1 won't be in flight; Phase 0 ships first. |

## 7. What Phase 0 does NOT do

- Does NOT add new validators to `validate` — only wires the existing set.
- Does NOT split `validate` into block/warn variants. Option 1 = unconditional fail-closed, no policy parameter.
- Does NOT touch `sim-urdf` (analogous validator wiring question — separate audit scope per the validate-wiring deferral memo §Related).
- Does NOT add the `load_model_with_validation(xml, policy)` API variant.
- Does NOT pick up the deferred sim-opt rematch verdict capture (per gameplan).
- Does NOT add post-expansion validate() (composite-induced state is internally controlled, not user input — different layer of concern).
- Does NOT touch any soft-body code. Phase 2 starts that.

## 8. Sequencing within the PR

1. Branch off main `c7c79d23` as `feature/phase-0-track-a-bundle`.
2. **Commit 1: yaml patch only** (Change 1). Push, draft PR. Observe CI matrix on all three OS — proves I-A and I-B in real conditions and gives R-3 + R-4 results.
3. **Commit 2: validate() wire-in (Change 2) + new test (Change 3).** Push. CI runs sim-mjcf and the 5 other sim-* crates with wired validate; any fixture rejection surfaces as a test failure, naturally categorized by which test broke.
4. If R-1 prediction holds (zero rejections): mark PR ready for review.
5. If commit 2 surfaces unexpected rejections: categorize per §6 R-1 (a/b/c/d), fix forward in additional commits in same PR. If category (b) (overly-strict-validator) appears, fall back to option 3 (`load_model_with_validation` variant) in a Phase 0.5 follow-up; ship Phase 0 with SD-4 only.
6. Squash to one commit at merge.

## 9. Open decisions

- **R-4 outcome dependent:** if windows wall time slips > 5 min, split test job. Default: accept the slip; revisit only if R-4 shows it.
- **sim-urdf parallel:** sim-urdf has analogous validator wiring question per the deferral memo. Phase 0 explicitly excludes it. Recommend separate Phase 0.5 (sim-urdf wiring) only if user wants symmetry across both crates; otherwise leave as standing follow-up.
