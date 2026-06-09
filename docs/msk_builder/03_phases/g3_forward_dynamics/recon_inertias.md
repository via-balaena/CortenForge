# G3 — Real Anthropometric Segment Inertias (sub-arc)

The coupled-knee forward-dynamics loop is CLOSED + validated vs OpenSim (PR1+PR2a
#290, PR2b #292, merged 2026-06-09). But the emit ships **placeholder** segment
inertias (`MAIN_INERTIAL` mass 1 / diag 0.1, `TINY` mass 0.01), so the
forward-dynamics gate (`tools/cf-mjcf-emit/tests/forward_dynamics_gate.rs`) has to
**inject** gait2392's real inertias to validate the solver. The shipped twin can't
move correctly under its own muscle force until it carries real inertias.

This sub-arc threads real inertia through the pipeline so the **emit writes real
`<inertial>`** and the gate stops injecting.

## The pipeline gap

- IR (`cf-msk-lib/src/ir.rs`) `Body` has NO mass/CoM/inertia fields.
- cf-osim (`cf-osim/src/osim.rs`) doesn't parse `<mass>`/`<mass_center>`/`<inertia_*>`.
- emit (`cf-mjcf-emit/src/lib.rs`) writes the placeholder constants.
- `realize` (`cf-msk-lib/src/lib.rs`) has no inertia to transform.

## Both #1 risks RETIRED by spikes (throwaway, `/tmp/g3_inertia_*.py` + `/tmp/g3_foot_composite.py`)

### Risk A — the scale-morph inertia transform vs OpenSim ScaleTool (PR2's risk)

Drove OpenSim `Model::scale` directly (Python) with known anisotropic factors on
femur/tibia and read back mass/CoM/inertia. **OpenSim's convention is now known
machine-exactly** (fit residual 3.5e-32, max rel err 2e-15). For a body whose long
axis is body-frame **y** (the gait2392 limb convention), with factors
`(sx, sy, sz)` = `(transverse, axial, transverse)` and `vol = sx·sy·sz`:

- **mass** → `vol · mass`  (volume scaling; `scaleMass=True`, i.e.
  `preserveMassDistribution=False`)
- **CoM** → component-wise `com · (sx, sy, sz)`
- **inertia** has TWO branches:
  - **Isotropic** (`sx==sy==sz`, includes identity): pure per-component dilation
    `I_ii_new = vol · s² · I_ii` — **preserves the true anisotropy** `Ixx≠Izz`.
  - **Anisotropic** (any factor differs, even 1.0001): OpenSim **forces
    axisymmetry** about the long axis (`Ixx_new == Izz_new`), discarding the real
    `Izz`, via:
    ```
    Iyy_new = vol · (sx·sz) · Iyy
    Ixx_new = Izz_new = vol · ( sy²·(Ixx − Iyy/2) + (sx·sz)·(Iyy/2) )
    ```
    (Decomposition: `Iyy ∝ m·r²` = transverse radius part; `Ixx − Iyy/2 ∝ m·L²` =
    axial-length part. Radius² scales geometrically `sx·sz`; length² scales `sy²`.)

**DECISION (head-engineer): MATCH OpenSim ScaleTool exactly, both branches.**
Rationale: the entire A3 validation strategy is "OpenSim ScaleTool IS the oracle for
a dialed body" (no real subject exists). The geometry morph already matches
ScaleTool; matching the inertia keeps ONE convention and — crucially — keeps the
inertia morph **gradeable** against real OpenSim (a tight 1e-5 gate, rounding-limited
by OpenSim's 6-decimal `getMoments` — the formula itself matches ScaleTool's
convention machine-exactly), which a physically-"more-correct" transform could not be. The axisymmetric-forcing quirk is
≤5% for the near-axisymmetric leg segments (femur Ixx/Izz differ 5%, tibia 1.4%) and
the canonical/gate body (identity scale) is unaffected (raw values either way). The
isotropic/anisotropic discontinuity is OpenSim's; we replicate it (branch on
`sx==sy==sz`). `BodyParams::uniform` exercises the isotropic branch; `AnthroSource`
(girth≠length) exercises the anisotropic branch.

talus/off-axis inertia: NOT scaled in v1 (the talus has no `BodyParams` dial — it
falls to `pelvis`=identity — and its composite inertia is off-axis, so the diagonal
formula doesn't apply). The foot isn't dialed; its mass still rides the (scaled)
tibia length via `location_in_parent`. Documented approximation.

### Risk B — the foot composite (PR1's risk)

The twin's `talus_r` is the **lumped foot** (the IR has no calcn/toes; OpenSim's
reduced model LOCKS subtalar/mtp, so the foot is rigid = talus+calcn+toes composite
about the talus frame — exactly what the gate injects today). Replicated
`gen_forward_dynamics.py::foot_composite` from a **pure `.osim` parse** (no OpenSim):
parse talus/calcn/toes mass+CoM+inertia + subtalar/mtp `location_in_parent`, compose
about the talus frame via parallel-axis. At the default pose subtalar/mtp are at
coord 0 with `orientation_in_parent=0`, so the three foot frames are **parallel** —
no rotation in the composition. **Matches the JSON `inertias["talus_r"]` to ~1e-8**
(mass exact; the JSON talus block is full-precision, so the ~1e-8 residual is an f64
composition-order gap vs OpenSim's own composite, NOT rounding). Subtalar/mtp
translation axes are all `Constant 0`, so `location_in_parent` is the whole offset.

Snapshot determinism: full-precision `{}` formatting is kept (unlike the polycoef,
which was snapped to `{:.12}` because it is a *lossy* fit — the inertia is not a fit,
so rounding it would discard real precision). femur/tibia `<inertial>` values are
exact `.osim` parses (no arithmetic). The talus composite is computed, but only from
plain scalar-f64 `+ − × ÷`, `dot`, and outer products — no `mul_add`/FMA and no
transcendentals — so it is bit-identical across platforms by IEEE-754 (verified: no
`mul_add` in `foot_composite`). **Caveat (pre-existing, not introduced here): the MSK
crates `cf-osim`/`cf-msk-lib`/`cf-mjcf-emit` run in NO CI job** (absent from the
`cargo test -p …` list and skipped by `grade-all --skip-coverage`), so the
`knee_ref.xml` byte-check and every test in this arc are LOCAL-only gates — keep
running them locally before each PR.

Emit `fullinertia` (sim-mjcf supports `<inertial pos mass fullinertia>` and
eigendecomposes on load — `mjcf/src/types.rs:1130`), so **NO eigendecomposition
needed in Rust** — just compose the symmetric tensor. femur/tibia have zero products
→ `fullinertia="Ixx Iyy Izz 0 0 0"`.

## PR slicing (head-engineer draft)

**PR1 — real inertia for the CANONICAL body; gate stops injecting (the headline).**
- IR `Body` gains `inertia: Option<Inertia>` where
  `Inertia { mass: f64, com: Vector3, tensor: [f64; 6] /* Ixx,Iyy,Izz,Ixy,Ixz,Iyz about com */ }`.
  (`Option`, mirroring `Muscle::force` — a synthetic body may omit it.)
- cf-osim: parse `<mass>/<mass_center>/<inertia_*>` for femur_r/tibia_r (diagonal,
  products 0) + pelvis; compose the talus foot composite (parallel-axis, parse
  calcn/toes/subtalar/mtp).
- emit: write a real `<inertial pos mass fullinertia>` from `Body.inertia` for each
  body that gets a free DOF (femur via hip, tibia via knee, talus via ankle);
  wrapper slide bodies + patella stay massless (`TINY`) — they are the tibia's
  massless coupled DOFs, exactly the validated config the gate injected into. Root
  pelvis: no `<inertial>` (welded; MuJoCo root needs none). Retire `MAIN_INERTIAL`.
- regen snapshot `tests/assets/knee_ref.xml` (now carries real `<inertial>`).
- `forward_dynamics_gate.rs`: DELETE the inertia-injection loop; keep gravity-off;
  assert the gate still passes (the emitted full-precision inertia ≈ the injected
  6-decimal JSON → ~2.6% match, marginally tighter than the old injected 3.2%).
  **THE WIN.**
- reconcile `coupled_knee_equality.rs` (the forward-dynamics manifold-hold sweeps run
  on the now-real inertia — the near-rigid PR2b impedance was tuned WITH injected
  real inertia, so expect the ~0.09 mm hold to survive the loose 1.5 mm bound; verify,
  don't assume) and `muscle_driven_dynamics.rs` (G2 static torque gate is
  inertia-independent — joint torque = moment arm × force; verify it still passes).

**PR2 — scale-morph inertia; ScaleTool oracle. ✅ IMPLEMENTED (branch
`feat/msk-g3-inertia-scale-morph`, uncommitted).**
- `realize`: scales EVERY body's `Inertia` by its own segment scale via
  `scale_inertia` (mass ∝ vol, com ∝ factors, inertia per the two-branch formula).
  Since `SegmentScale` always has `sx==sz` (transverse on both x,z), the branch is
  simply isotropic (`axial==transverse` → ×s⁵ on all 6 components, products-safe) vs
  anisotropic (`axial≠transverse` → the forced-axisymmetric diagonal rule, asserts
  products-0). femur/tibia get their own dial; pelvis/talus fall to the
  never-anisotropic pelvis scale (so the off-axis lumped-foot composite only ever
  hits the products-safe isotropic branch — moot at IDENTITY default).
- Oracle: EXTENDED `gen_scaled_moment_arms.py` (NOT a new file) with a per-config
  `inertias` block — a SEPARATE `Model::scale(..., preserveMassDistribution=False)`
  call (mass ∝ volume) reading femur_r/tibia_r mass/com/moments/products. Geometry/
  moment arms are identical either preserve flag, so the existing moment-arm gate is
  byte-unchanged. Regenerated `scaled_moment_arms_opensim.json`.
- validation gate `morph_inertia_matches_real_opensim_scaletool` (in
  `opensim_cross_check.rs`, reusing `body_params_from_factors`): realize at each grid
  factor, compare femur/tibia inertia to OpenSim. **Worst Δ 5.4e-7 across all 13
  configs**, gate 1e-5. **★ FINDING: OpenSim's `getInertia().getMoments()` returns the
  inertia 6-DECIMAL-ROUNDED** (femur Ixx 0.170221 vs the .osim's 0.170220714339411),
  so the oracle is only 6-decimal-precise; `realize` scales the FULL-precision .osim
  parse (more accurate), so it diverges from the rounded oracle by ~1e-7..1e-6 (the
  rounding propagated through scaling) — NOT a formula error (which would be
  percent-scale). The formula itself is machine-exact vs ScaleTool's convention
  (the 1e-9 spike match used OpenSim's own rounded original as input). Plus 4 formula-
  level unit tests in cf-msk-lib (identity no-op / uniform ×s⁵ / anisotropic
  axisymmetry / realize end-to-end) that pin the morph WITHOUT the OpenSim oracle.

Each PR: n+1 cold-read + pre-PR local ultra-review; don't push/open PRs without
user go-ahead.

## Key values (gait2392, full precision in `.osim`)

- femur_r: mass 8.98403823 kg, com `(0, −0.195031, 0)`, diag `(0.170220714,
  0.044620964, 0.179500858)`, products 0.
- tibia_r: mass 3.58100090 kg, com `(0, −0.184557, 0)`, diag `(0.047569366,
  0.004813567, 0.048230052)`, products 0.
- talus_r foot composite: mass 1.51314794 kg, com ≈ `(0.0613947, −0.0185949,
  0.0059278)`, full tensor `Ixx 0.0029441, Iyy 0.0082383, Izz 0.0084487, Ixy
  0.0006724, Ixz 0.0003127, Iyz −6.893e-5` (about com, talus frame).
- foot bodies: talus 0.09658802 kg, calcn 1.20735027 kg, toes 0.20920965 kg;
  subtalar_r loc `(−0.05432871, −0.04673138, 0.00882271)`, mtp_r loc `(0.18364276,
  −0.00205417, 0.00110925)`.

## Handoff anchors

- main clean + synced at `8206e89a` (#292). This recon is the first artifact of the
  inertia sub-arc.
- gate injection to remove: `forward_dynamics_gate.rs:67-81`.
- emit placeholders: `cf-mjcf-emit/src/lib.rs:63-64` (`TINY`/`MAIN_INERTIAL`),
  emitted at `:401/:458/:521`.
- cf-osim body construction: `osim.rs:43-91`; inertia parse helpers go beside
  `parse_coordinates`/`joint_location`.
- realize morph: `cf-msk-lib/src/lib.rs:208` (clones template; new inertia field
  rides through identity unchanged — PR1 needs no realize change).
- OpenSim venv `/tmp/osim-venv` (4.6). Spikes (throwaway) `/tmp/g3_inertia_*.py`,
  `/tmp/g3_foot_composite.py`, `/tmp/g3_branch.py` — findings captured above.
