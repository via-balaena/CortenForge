# 2026-04-09 (part 5) — Item 3 closed: existing `qfrc_applied` write patterns

> Extracted from `MASTER_PLAN.md` §7 part 05 during the 2026-04-09 doc-tree refactor.

- **Trigger**: Continuation of recon item 3, the *rest* of it. Part 4
  closed the `cb_passive` sub-question and revised the Phase 1 design;
  this entry closes the remainder by reading the three files cataloging
  how RL controllers and example code currently inject forces:
  `sim/L0/ml-bridge/src/space.rs`, `sim/L0/tests/integration/split_step.rs`,
  `sim/L1/bevy/examples/coupled_pendulums.rs`. The thermostat itself
  doesn't go through `qfrc_applied` anymore, so this is forward-looking
  recon for Phase 7 composition with RL — *not* Phase 1 design input.
- **Pattern 1 — ml-bridge `ActionSpace::apply`**
  (`ml-bridge/src/space.rs:599-768`, call site `env.rs:133-146`):
  - **Cadence**: action injection happens *before* `data.step()`, with
    plain `step()`. Not split-step. The `SimEnv::step()` body is literally
    `act_space.apply(action, &mut self.data, &self.model); for _ in 0..sub_steps { self.data.step(...)?; … }`.
  - **Semantics**: pure overwrite. `Injector::QfrcApplied` does
    `data.qfrc_applied[i] = f64::from(val)` (line 644). Same shape for
    `XfrcApplied` (line 651), `Ctrl` (line 639, with clamp), `MocapPos`,
    `MocapQuat`. There is no `+=` anywhere in the injector path.
  - **Range coverage**: each injector overwrites only the indices in its
    declared `Range<usize>`. **Indices outside the declared range are not
    cleared.** Safe today because `Data::reset()` zeros `qfrc_applied` and
    `SimEnv::reset()` calls `Data::reset()`, but the injector contract is
    "I own my range," not "I own the field."
  - **Five disjoint slots**: ctrl, qfrc_applied, xfrc_applied, mocap_pos,
    mocap_quat. ml-bridge composes multiple injectors *only* via field
    disjointness — never two writers to the same slot.
- **Pattern 2 — `split_step.rs` integration test**
  (`sim/L0/tests/integration/split_step.rs:87-121`):
  - The *only* in-tree consumer of the documented split-step force-injection
    cadence. Writes `data.qfrc_applied[0] = 5.0` between `step1()` and
    `step2()` and asserts the trajectory diverges from a baseline.
  - It is a **contract test** for the split-step API, not a usage
    cookbook. It does not represent the dominant idiom in the codebase.
- **Pattern 3 — `coupled_pendulums.rs` Bevy example**
  (`sim/L1/bevy/examples/coupled_pendulums.rs:359-394`):
  - **Cadence**: identical to ml-bridge — write before plain `step()`,
    no split-step. Inside the substep loop:
    `data.qfrc_applied[0] = coupling; data.qfrc_applied[1] = -coupling;
    data.step(&model)?;`
  - **Semantics**: overwrite with state-dependent value
    (`coupling = -KAPPA * (theta1 - theta2)`).
  - **Reset path is manual**: the restart handler does
    `qpos.fill(0.0); qvel.fill(0.0); qfrc_applied.fill(0.0); time = 0.0`
    explicitly, *bypassing* `Data::reset()`. The author *knows*
    `qfrc_applied` is persistent across steps and clears it explicitly.
    Confirms the cultural awareness of MuJoCo "user owns lifecycle."
- **Synthesis — three observations that matter for the thermostat:**
  1. **Both real-world writers (ml-bridge and coupled_pendulums) use
     plain `step()`, not split-step.** The `step1`/`step2` API is
     documented + tested but the codebase's actual writers don't reach
     for it; they write to `qfrc_applied` *before* `step()` and rely on
     overwrite the next iteration. **This validates the cb_passive
     design choice from part 4** — the thermostat will use plain
     `step()` too, joining the dominant convention. The split-step API
     remains a tool in the toolbox but is not the idiom.
  2. **Both real-world writers use `=`, not `+=`.** Each owns the index
     it cares about and overwrites it each step. Multiple writers to the
     *same* index would clobber each other; the codebase has no
     established protocol for that case because it has never needed one.
     **Field disjointness is the de facto composition idiom.**
  3. **The thermostat (`qfrc_passive`) and ml-bridge `ActionSpace`
     (`qfrc_applied`/`xfrc_applied`/`ctrl`) live in different fields.
     They cannot clobber each other.** Composition with RL is automatic
     and trivial — no API design needed in Phase 1, and Phase 7 inherits
     it for free. Document this in the Phase 1 spec under "Composition
     with other writers" so future readers understand it was *designed
     in*, not stumbled into.
- **Two surprises (neither blocks Phase 1, both flagged for the user):**
  1. **Doc/code drift on `step1()`'s doc-comment**. Part 2's recon noted
     that the `step1()` doc-comment "literally calls out [the
     step1/step2 hook] as the RL force-injection point." That doc is
     accurate as a *capability* statement but does not match the *actual
     ml-bridge code path*, which writes before plain `step()`. Not a
     bug, not a design issue, but the doc reads as if split-step is the
     RL idiom when in fact it isn't. Worth a doc fixup eventually; out
     of scope for this branch.
  2. **The thermostat will be the workspace's first `cb_passive`
     consumer.** `callbacks.rs:37-41` documents the API; nothing in
     `sim/`, `examples/`, or any test currently uses it. (Verified by
     the absence of any hits in the three target files and by the fact
     that Phase 1 design searches in part 4 found only the type
     definition + initialization sites, not call sites.) **Implication
     for the spec**: tests need to establish the precedent for
     callback-RNG reproducibility patterns. A small dedicated test —
     "the callback fires once per `step()`, RNG advances by exactly the
     expected count" — should be in the Phase 1 validation suite, not
     just the equipartition test.
- **Latent footgun noted (NOT Phase 1 relevant)**: The
  `Injector::QfrcApplied` contract is "I overwrite my range, I do not
  clear unrelated indices." Safe today via `Data::reset()`. If a user
  ever combines an `ActionSpace` with the escape-hatch `data_mut()` and
  bypasses reset, stale `qfrc_applied` values outside the action range
  would persist across episodes. Not introduced by anything thermostat-
  related; flagging for ml-bridge maintainers as a docs improvement
  opportunity. Out of scope for this branch.
- **Implications for the Phase 1 spec** (concrete, will appear in the
  spec when it gets drafted):
  - **Composition section**: state explicitly that the thermostat lives
    in `qfrc_passive` via `cb_passive`, that ml-bridge writers live in
    `qfrc_applied`/`xfrc_applied`/`ctrl`, and that field disjointness
    makes composition automatic. Cite the three patterns in this entry.
  - **Test section**: include a callback-firing-count test in addition
    to the equipartition test. Establishes the `cb_passive` test
    precedent for the workspace.
  - **Cadence section**: the canonical user pattern is plain `step()`.
    No split-step required. State this explicitly so a beginner reading
    the spec doesn't reach for `step1()`/`step2()` thinking it's
    necessary.
- **Item 3 RESOLVED.** Both halves are now closed: the `cb_passive`
  sub-question (part 4, design-changing) and the existing-pattern survey
  (this entry, design-confirming). No revisions to the Phase 1 design
  result from this entry — only documentation hooks for the spec.
- **Next action**: Phase-1-blocking recon item 4 — PRNG conventions in
  CortenForge. Question: which PRNG crate is the workspace standard
  (`rand` + `rand_chacha`? something else? a custom in-tree generator?),
  what seeding patterns are already used in tests/examples, and is
  there a documented thread-safe-RNG idiom. Answer determines the
  concrete type behind `Mutex<PRNG>` in the `LangevinThermostat` struct.

