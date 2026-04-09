# 2026-04-09 (part 7) — Item 5: `Data::qfrc_passive` public mutability

> Extracted from `MASTER_PLAN.md` §7 part 07 during the 2026-04-09 doc-tree refactor.

- **Trigger**: Recon item 5, the smallest of the Phase-1-blocking items.
  Listed in part 3 as "probably fine since other passive fields are
  pub, but verify before drafting the spec." Single-question check.
- **Question**: Is `Data::qfrc_passive` declared `pub` so the
  `cb_passive` callback (which receives `&mut Data`) can write into it
  directly via `data.qfrc_passive[i] += ...`, or is it crate-private
  with no public mutator?
- **Finding**: **Yes — `pub qfrc_passive: DVector<f64>`** at
  `sim/L0/core/src/types/data.rs:158`. Direct field-access mutation is
  supported and is the only intended access path; there is no
  setter/mutator method shadowing it.
- **Bonus context — the entire "Forces in Generalized Coordinates"
  block is uniformly public** (`data.rs:152-172`):
  - `pub qfrc_applied`     (line 154)
  - `pub qfrc_bias`        (line 156)
  - `pub qfrc_passive`     (line 158)
  - `pub qfrc_spring`      (line 161)
  - `pub qfrc_damper`      (line 164)
  - `pub qfrc_fluid`       (line 167)
  - `pub qfrc_gravcomp`    (line 170)
  - `pub qfrc_constraint`  (line 172)

  The codebase convention for `Data` is "fields are public, callers
  read and write directly." This is consistent with the MuJoCo origin
  (mjData fields are all public C struct members) and with the
  `Injector::QfrcApplied` pattern in ml-bridge (item 3 part 5), which
  writes `data.qfrc_applied[i] = ...` directly. The thermostat writing
  `data.qfrc_passive[i] += ...` matches this convention exactly.
- **Sub-observation — the spring/damper/fluid/gravcomp split**: All
  the *component* passive fields (`qfrc_spring`, `qfrc_damper`,
  `qfrc_fluid`, `qfrc_gravcomp`) are also `pub`. They are aggregated
  into `qfrc_passive` at `forward/passive.rs:681` *before* `cb_passive`
  fires (`passive.rs:719`, per part 4). The thermostat writes into
  the *aggregated* `qfrc_passive` field after the engine has finished
  populating it from springs/dampers/fluid/gravcomp — never into a
  component field. Writing into a component field would be a category
  error (component fields have specific physical meanings) and would
  also be wrong on the timing (the aggregation has already happened).
  Part 4's design choice is reaffirmed; no surprise here.
- **No surprises**. Item 5 was expected to be a quick yes-or-no check
  and it was. The only thing worth flagging is that the spec should
  state explicitly *which* field the thermostat writes into
  (`qfrc_passive`, the aggregated post-spring/damper field) and *why*
  (cb_passive fires after aggregation — the aggregated field is the
  only correct write target).
- **Implication for the Phase 1 spec**: One sentence in the API
  section ("Writes via `data.qfrc_passive[i] += ...`; field is
  declared `pub` at `data.rs:158`; no setter required.") and one
  sentence in the design rationale ("Writes the *aggregated*
  `qfrc_passive` field, not the component spring/damper/fluid/gravcomp
  fields, because `cb_passive` fires after aggregation
  (`passive.rs:681,719`).") That's it.
- **Item 5 RESOLVED.** No design changes; a documentation hook for the
  spec is the only output.
- **Next action**: Phase-1-blocking recon item 6 — `model.timestep`
  variability across steps. Question: is `model.timestep` ever changed
  during a simulation, or is it set once at model init and held
  constant? The thermostat reads `h = model.timestep` inside
  `cb_passive` to scale the noise as `(2γkT/h)^{1/2}`. If `h` can
  change between steps, the thermostat is correct. If it can't, the
  `(2γkT/h)^{1/2}` factor could be precomputed once and cached at
  install time, eliminating a sqrt per step per DOF. Performance is
  negligible at Phase 1's nv=1, but the answer also affects the
  `LangevinThermostat` constructor signature (does it take `h`, or
  read it from the model?).

