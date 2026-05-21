# cf-scan-prep v1.0 recon — gap list

**Date:** 2026-05-15
**Session role:** recon (step 2 of the three-session pattern; bookmark
shipped at `3db1f929`, implementation comes after this).
**Branch:** `dev` at `3db1f929`, clean tree, 5 shipped slices + bookmark
ahead of `main` (`007db543`).
**Method:** read `docs/SCAN_PREP_DESIGN.md` cold against
`tools/cf-scan-prep/src/main.rs` (4892 LOC); then re-read
`tools/cf-cast-cli/src/{scan,prep,design_ref,lib}.rs` and
`tools/cf-device-design/src/main.rs` for the downstream-consumer
contract.

## How to read this list

Each gap is one section. Severity, effort, and the design intuition are
the three fields the implementation session will use to set ordering.
**This recon does NOT set the implementation order** — that's the next
session's call. The grouping below is alphabetical-within-severity.

- **Severity** — `workshop-blocking` (cleaned scan misbehaves
  downstream), `quality-of-life` (workflow friction, downstream still
  works), `cosmetic` (spec promised; impact ≤ visual polish).
- **Effort** — `sub-commit` (single self-contained patch, < 200 LOC),
  `multi-commit` (a slice with sub-leaves), `multi-session` (needs
  spec-amendment design call first).
- **Anchors** are `file:line` where relevant; spec citations point at
  `docs/SCAN_PREP_DESIGN.md` (abbreviated `SPEC §…`).

The known-known gaps from the bookmark (centerline trim, centerline
adjust) appear here in the full list with the same fields as the others.

---

## Workshop-blocking gaps

These produce a `.cleaned.stl + .prep.toml` pair that misbehaves
downstream — either silently (the user clicks Save and the file on disk
disagrees with the panel state) or loudly (the file loads but cf-cast
SDF / cf-device-design fails).

### WB-1 — Centerline trim not implemented

- **Severity:** workshop-blocking
- **Effort:** multi-commit (~5-7 sub-leaves; was CL.1a-f in PR #246)
- **Anchors:** `tools/cf-scan-prep/src/main.rs:969-1031`
  (`compute_centerline_polyline` — fixed `n_slices=30`, no end-trim);
  `CapState::centerline_polyline` written from this raw output and
  saved verbatim into `.prep.toml`.

The cross-section-centroids algorithm walks the WHOLE scan depth range
(min projection → max projection along the cap-loop normal). The
endpoint slabs contain the noisy entry/exit of the scan — for the
iter-1 sock fixture this means polyline points that hang off the end
of the device, then drive cf-device-design's radial-direction math to
something nonsensical at those Z extremes.

**Design intuition:** perpendicular-plane clip at user-supplied
`t_start_m` / `t_end_m` along the polyline arc length. Sliders in the
Cap panel sub-section (or its own panel) with live overlay (red
endpoint markers + ghost segments showing the trimmed tail). The trim
output is what gets baked into `.prep.toml`'s `[centerline].points_m`;
the raw centerline stays in memory for visual reference. Bookmark
already names this as the lead recon starting point.

---

### WB-2 — Cleaned STL has disconnected components / degenerate triangles

- **Severity:** workshop-blocking
- **Effort:** multi-commit (needs recon-in-recon to find the
  exact failure mode, then targeted fix)
- **Anchors:** `tools/cf-device-design/src/main.rs:410-425`
  (the smoking gun — cf-device-design switched from
  `simplify_decoder` to `simplify_sloppy_decoder` because the iter-1
  cleaned scan retains its full 3.34M face count through topology-
  preserving simplification); `tools/cf-scan-prep/src/main.rs:1225-1317`
  (`build_cleaned_mesh` — appends cap faces via ear-clip without
  re-checking for degenerate / near-collinear vertices, and reuses
  loop vertex indices as-is so any non-planar wobble in the boundary
  loop manifests as off-plane cap triangles).

The cleaned STL emitted by cf-scan-prep is not actually "clean enough"
for ordinary topology-preserving simplification downstream.
cf-device-design admits this in a code comment: "the scan has
disconnected components / degenerate triangles that block the regular
collapse algorithm. The sloppy variant ignores topology and reliably
hits ~1500 faces." Sloppy simplify ignores both connectivity and
manifoldness, so cf-device-design works, but cf-cast's SDF builder
(`mesh_sdf::SignedDistanceField::new`) makes stricter assumptions —
and the user has reported downstream pain consistent with this class
of bug.

**Design intuition:** two-pronged.
1. Add a post-build mesh-cleanup pass to `build_cleaned_mesh`:
   `weld_vertices(SIMPLIFY_WELD_EPSILON_M)` + degenerate-triangle
   strip (zero-area faces, collinear vertices) + smallest-component
   strip (drop islands < 1% of total face count). All three primitives
   already live in `mesh-repair`.
2. Cap ear-clipping reuses original loop vertex indices instead of
   sampling new ones on the fit plane — for low-R² loops this puts cap
   faces off the plane. Fix by projecting loop vertices onto the fit
   plane before appending (small geometric move; ear-clip already
   builds the 2D coords).

Both fixes belong in the same slice because the symptom is one mesh
hygiene problem with two roots.

---

### WB-3 — Clip-floor toggle has no effect at save

- **Severity:** workshop-blocking
- **Effort:** sub-commit
- **Anchors:** `tools/cf-scan-prep/src/main.rs:711-818`
  (`clip_mesh_against_world_z` — fully implemented, fully unit-tested,
  marked `#[allow(dead_code)]`); `main.rs:1208-1212`
  (`build_cleaned_mesh` docstring: "Skips clip baking for now");
  `main.rs:1469-1475` (`PrepClipBlock { baked: false }` hard-coded).
  Spec §Panel specifications §5 + §Output format both say the clip
  bakes into the cleaned STL.

The Clip panel renders the disc visualization + the drops-X% readout,
and the user can toggle it on with a Z slider. None of that affects
the file on disk. The cleaned STL emerges unchanged, `.prep.toml`
records `clip.enabled = true, clip.baked = false`, downstream tools
have no way to know they're getting an un-trimmed scan. This is the
9.8 failure-mode again (slider implies an effect that doesn't happen)
in a different panel.

**Design intuition:** wire `clip_mesh_against_world_z` into
`build_cleaned_mesh` when `clip.enabled`. Algorithm is already unit-
tested. Order matters: clip BEFORE cap ear-clip (clipped boundary is
what cap detection sees) — but the cap loops were detected at scan
time on a pre-clip mesh, so a clip change after scan should trigger
the same staleness invalidation as Reorient/Recenter. The
`mark_cap_stale_on_transform_change` system already takes the right
shape; add `clip: Res<ClipState>` to its parameter list. Update
`PrepClipBlock.baked` to track reality.

Alternative: declare clip purely advisory (matches the current
comment "v2 cf-cast handles trimming via the centerline-derived mold
geometry") and **remove the toggle / slider** so users don't expect
an effect. Pick one — current state is the worst of both worlds.

---

### WB-4 — Centerline algorithm degrades on closed / branching scans

- **Severity:** workshop-blocking (for non-prosthetic-appendage
  geometries) / quality-of-life (for the iter-1 fixture class)
- **Effort:** multi-commit
- **Anchors:** `tools/cf-scan-prep/src/main.rs:990-1031`
  (`compute_centerline_polyline` — uses first cap loop's normal as
  spine; assumes single boundary + tubular topology);
  `main.rs:2574-2581` (handle_cap_actions — closed mesh ⇒ empty
  centerline ⇒ empty `[centerline]` block ⇒ cf-cast-cli refuses to
  build the cast at `lib.rs:137-140`).

If the user prep'd a closed scan (single-shell already-watertight, or
a multi-cap scan where cap 0 is the WRONG end), the centerline either
disappears entirely or points the wrong way. cf-cast-cli's downstream
contract — "centerline polyline in `<stem>.prep.toml` is empty — cf-
scan-prep emits the [centerline] block only when a centerline is
present; ensure the Cap panel was applied + centerline polyline was
computed before saving" — surfaces the failure but the user has no
in-tool recovery path.

**Design intuition:**
- Let the user pick which cap loop indexes the spine
  (`spine_loop_idx: Option<usize>` on `CapState`; default 0; UI is
  a one-line dropdown in the Cap panel).
- For closed meshes with no boundary loops, fall back to PCA's
  principal axis as the spine (we already have
  `compute_pca_orientation`; same math, slightly different output
  shape).
- Slabs adaptive to spine arc length: target `n_slices` ≈ depth /
  `target_slab_thickness_m` (slider; default 2 mm matching the
  cf-cast SDF cell). Hard-coded 30 is too few for long body parts
  and too many for short ones.

Closed-mesh fallback is sub-commit; user-pickable spine + slab-density
slider are each sub-commits; pulling them together is multi-commit
because they share the Cap panel surface.

---

## Quality-of-life gaps

These don't break downstream — the cleaned scan still loads + the
mold still computes — but they materially affect whether the user can
finish a prep session without external tools or guesswork.

### QoL-1 — Cleaned AABB panel missing

- **Severity:** quality-of-life
- **Effort:** sub-commit
- **Anchors:** spec §Panel specifications §8; no `render_cleaned_aabb_section`
  in the as-built (`grep "^fn render_" main.rs` shows 7 sections, spec
  says 9).

Spec promised a read-only post-transform AABB readout. As-built, the
user has the *raw* AABB in the Scan Info panel and nothing for the
post-Reorient/Recenter footprint. Without it the user can't tell if
the cleaned scan fits the print volume until they save + re-open.

**Design intuition:** new `render_cleaned_aabb_section` that walks
the 8 corners of `OverlayLengths::raw_aabb_m` through the Reorient
quaternion + Recenter translation (the math already exists in
`rotated_aabb_around_centroid_physics_mm` — extend it to apply
Recenter too, or just compose at the call site). Display W/D/H in
mm. Tie WB-3 into the same panel for free: if clip is enabled +
baked, the readout reflects the clipped extents.

---

### QoL-2 — Build-volume CLI flag + warnings missing

- **Severity:** quality-of-life
- **Effort:** sub-commit
- **Anchors:** spec §Architectural decisions §Print-volume warnings
  + §Panel specifications §8; no `build_volume` / `--build-volume-mm`
  anywhere in `main.rs`.

Spec promised `--build-volume-mm W,D,H` (default `250×210×210 mm` for
the Prusa MK3S) and a red panel warning when the cleaned AABB
exceeds it. Useful for the user's actual printer (the workshop
printer's volume drives mold-piece slicing). As-built: no flag, no
warning. Pairs naturally with QoL-1 (the build-volume warning lives
in the Cleaned AABB panel).

**Design intuition:** new `clap` arg on `Cli`, parse as
`(f64, f64, f64)`, store as `Resource`. Per-axis warning in
QoL-1's panel: `"⚠ exceeds build volume on Z axis (260.3 > 210.0 mm)"`.
Advisory only — save still proceeds.

---

### QoL-3 — Mesh-repair load diagnostic not surfaced

- **Severity:** quality-of-life
- **Effort:** sub-commit
- **Anchors:** spec §Validation rules §"At load:" — the
  `"6 issues: 4 holes, 2 non-manifold edges"` status-bar diagnostic.
  `init_status_for_load` (`main.rs:2227-2240`) only surfaces the
  face-count auto-suggest banner, not the mesh-repair issue count.

The user has no signal at load time that the scan has non-manifold
edges, disconnected shells, or holes other than the ones the Cap
panel will surface. WB-2 is partly downstream of this: if the user
knew at load that the scan had 12 disconnected shells, they'd
external-trim before importing.

**Design intuition:** `init_status_for_load` runs `mesh-repair`'s
diagnostic (it's a one-shot call on the loaded mesh; same surface as
`detect_boundary_loops`) and concatenates the issue summary into the
auto-suggest banner. Persistent until user runs a panel action.

---

### QoL-4 — Centerline algorithm has no density / smoothing knobs

- **Severity:** quality-of-life
- **Effort:** sub-commit
- **Anchors:** `main.rs:2578` (`compute_centerline_polyline(&scan.0,
  first_loop.plane_normal, 30)` — hard-coded N=30).

Different scans want different polyline density. For a 130 mm sock
fixture, 30 segments ≈ 4 mm pitch, fine. For a 500 mm arm scan, 30
segments ≈ 17 mm pitch — too coarse for cf-device-design's radial
math at the curve. No way to dial this without recompiling.

**Design intuition:** slab-density slider in the Cap panel (or
folded into WB-4's broader centerline panel rework). Default
`target_slab_thickness_m` = 2 mm matching the cf-cast SDF cell.
Compute `n_slices = max(2, ceil(spine_length_m / target_slab))`.
Smoothing pass (3-tap moving average over polyline points) is
trivial to bolt on; banked unless workshop iter-1 shows the raw
centroids are too jaggedy.

---

### QoL-5 — Keyboard shortcuts missing

- **Severity:** quality-of-life (Ctrl+S) / cosmetic (everything else)
- **Effort:** sub-commit
- **Anchors:** spec §Keyboard shortcuts; `exit_on_esc`
  (`main.rs:2846-2850`) is the only key handler — Ctrl+S, R, C, F, A,
  1/2/3 are all unimplemented.

Ctrl+S is the only one workflow-relevant: clicking through the egui
panel for Save costs ~1 second per save iteration. The rest (1/2/3
for snap rotations, A for apply caps, R for reset) are nice-to-have.

**Design intuition:** one Bevy system reading
`Res<ButtonInput<KeyCode>>`, dispatching to
`SavePendingAction::save_now = true` on Ctrl+S and
`SimplifyAction::Reset` / etc. on the rest. Reuses the existing
pending-action plumbing — no new wiring.

---

### QoL-6 — Centerline adjust (manual point nudging) missing

- **Severity:** quality-of-life
- **Effort:** multi-commit
- **Anchors:** bookmark "Known gaps as recon starting points";
  spec doesn't explicitly include this, but the user flagged it
  during PR #246 ladder design.

When the auto-generated centerline doesn't quite trace the spine —
common at the dome end where cross-section centroids skew toward the
fat side of the cup — there's no way to nudge it. Workshop iter-1
will surface specific cases; for now it's known-missing.

**Design intuition:** per-point sliders in the Cap panel (one row per
centerline segment, X/Y/Z nudges in mm). Alternative: 3D drag-handles
in the viewport (banked in the spec's deferred items). User flagged
this as lower priority than WB-1. The lift is non-trivial — making the
trim from WB-1 ship first is probably worth more than this for iter-1.

---

## Cosmetic gaps

Spec promised these; nothing downstream cares. Listed for completeness
so the implementation session knows what to leave deferred vs. fold in
opportunistically.

### Cos-1 — Mouth extension panel missing

- **Severity:** cosmetic (under v2 cf-cast)
- **Effort:** multi-commit
- **Anchors:** spec §Panel specifications §7; no `MouthState` or
  `render_mouth_section` anywhere.

The spec design predates the v2 curve-following cf-cast pivot. v2
cf-cast derives the mold bounding region from the centerline +
ribbon, NOT from a mouth-extension hint in `.prep.toml`. Neither
cf-cast-cli nor cf-device-design reads `mouth_extension` from the
TOML. Implementing this panel would write a field nothing consumes.

**Design intuition:** **defer indefinitely**. If a future workshop
iteration needs a "headroom past the +z extent" knob, revisit then.
Worth noting in the spec slice ship log that this is v1.0-scope-cut.

---

### Cos-2 — `[Reset all]` button missing

- **Severity:** cosmetic
- **Effort:** sub-commit
- **Anchors:** spec §Panel specifications §9 (Save panel); no
  reset-all in `render_save_section` (`main.rs:3324-3383`).

Spec promised a button that resets Reorient/Recenter/Clip/Cap state
without touching simplification. As-built: user clicks `Reset
rotation` + `Reset translation` + un-checks `Clip enabled` + re-Scans
the cap manually. Tedious but possible.

**Design intuition:** one button in the Save panel; one handler that
writes `ReorientState::default()` / `RecenterState::default()` /
`ClipState::default()` / `CapState::default()`. ~10 LOC.

---

### Cos-3 — `[Apply caps]` preview button missing

- **Severity:** cosmetic
- **Effort:** sub-commit
- **Anchors:** spec §Panel specifications §6 (Apply caps); as-built
  bakes caps automatically at save time inside `build_cleaned_mesh`.

Spec called for a button that previews the cap triangulation in the
viewport (translucent fill on the proposed cap plane). As-built: no
preview, caps just appear in the saved STL. The user-facing behavior
is equivalent for confident users; less reassuring for the first run
on a new fixture.

**Design intuition:** new gizmo-line overlay in `draw_cap_overlays`
showing the ear-clip output edges for any loop with `include = true`.
Sub-commit; "Apply caps" button can stay implicit at save time.

---

### Cos-4 — Per-loop manual plane override missing

- **Severity:** cosmetic (matters only when auto-fit R² is poor)
- **Effort:** multi-commit
- **Anchors:** spec §Panel specifications §6 sub-section "Per-loop
  manual override"; not implemented.

Spec calls for a collapsible sub-panel per loop with axis dropdown +
offset slider, allowing user override of the auto-fit plane. For the
iter-1 sock fixture the auto-fit gets R² > 0.94 (the canonical
example value in the TOML schema); manual override is dead code until
a low-R² fixture shows up.

**Design intuition:** banked behind workshop evidence. Implement when
a real scan shows R² < ~0.85 and the resulting cap shape is visibly
wrong.

---

### Cos-5 — `.prep.toml` schema drift from spec

- **Severity:** cosmetic
- **Effort:** sub-commit
- **Anchors:** spec §Output format vs. `build_prep_toml_string`
  (`main.rs:1416-1509`); examples:
  - Spec: `[transform.rotation] / quaternion = [w,x,y,z]`. As-built:
    `[reorient] / quaternion = [...] / roll_deg / pitch_deg / yaw_deg`.
  - Spec: `[transform.translation] / m = [x,y,z]`. As-built:
    `[recenter] / translation_m / translation_mm`.
  - Spec: `[simplify] / target_face_count / achieved_face_count / …`.
    As-built: no `[simplify]` block at all (the 9.8 save-time
    simplification is invisible in provenance).
  - Spec: `[output.aabb_m]`. As-built: no AABB in the output block.

Downstream consumers (cf-cast-cli `prep.rs`, cf-device-design
`parse_centerline`) both only read `[centerline].points_m` and
tolerate unknown fields, so the schema drift is invisible to them.
A future audit / migration would benefit from spec-true names + the
missing `[simplify]` block (so the file says what was actually
applied).

**Design intuition:** either (a) update the spec to match the
as-built and add a `[simplify]` block, or (b) rename the as-built
blocks to match spec. Pick once at the start of the implementation
session — doesn't affect downstream behavior either way.

---

### Cos-6 — Status TTL polish (cosmetic Save-message lingering)

- **Severity:** cosmetic
- **Effort:** sub-commit
- **Anchors:** `handle_save_action` (`main.rs:3516`) sets
  `auto_clear_at_secs = Some(now + 6.0)`; spec §Save panel said
  `"Saved at HH:MM:SS"` — the as-built includes the full triangle
  count + simplify suffix but no HH:MM:SS clock readout. Minor.

Banked; not worth a slice on its own.

---

## Out-of-scope items found during recon (not gaps)

These came up in the cold-read and are explicitly NOT in this gap
list, but worth surfacing so the implementation session doesn't
accidentally re-open them:

- **Auto-orient (PCA)** — shipped beyond spec at `main.rs:3044-3051`.
  Useful + working; do not remove.
- **Snap boundary → floor** — shipped beyond spec at
  `main.rs:3244-3268`. One-click alignment that the user
  appreciated. Do not remove.
- **Save-time face budget (slice 9.8)** — shipped on dev at
  `6ebc60db`. Confirmed-working; the broader diagnosis (cf-scan-prep
  promises behavior it doesn't deliver) emerged AROUND it, not
  against it.
- **Live progress logging (slice 9.7)** — orthogonal to v1.0
  completion. Helps workshop iter-1 visibility; no recon action.
- **mesh-io load_stl >162-face BufReader-boundary bug** — fixed on
  dev `922ee1b4` 2026-05-12. Resolved.

---

## First-pass severity totals

- workshop-blocking: **4** (WB-1 centerline trim, WB-2 cleaned STL
  hygiene, WB-3 clip-not-baked, WB-4 centerline algo robustness)
- quality-of-life: **6** (Cleaned AABB, build volume, mesh-repair
  diagnostic, centerline density, keyboard shortcuts, centerline
  adjust)
- cosmetic: **6** (Mouth ext., Reset all, Apply preview, manual plane
  override, schema drift, status TTL polish)

## Handoff notes for the implementation session

1. **Read this file first**, then `docs/PR_246_BOOKMARK.md`, then
   pick an ordering. The recon does not set order.
2. The 4 workshop-blocking gaps are not independent. WB-1 trim and
   WB-4 algo robustness share the centerline-polyline surface. WB-2
   and WB-3 share `build_cleaned_mesh`. Two of the four can share a
   single slice in each case.
3. **Decide schema posture (Cos-5) once, before any TOML-touching
   slice ships.** If we're renaming blocks, doing it early in the arc
   means later slices don't have to plan migrations.
4. The 5 already-shipped slices on dev (`6e4d3973`, `989d7301`,
   `cd466309`, `de567792`, `6ebc60db`) are downstream-of-cf-scan-prep
   polish. They don't conflict with anything in this gap list — but
   the PR shape decision (one big "v1.0 completion + the 5 polish
   slices" PR vs. two PRs) is still open.
5. The bookmark explicitly forbade touching the 5 shipped commits,
   opening PR #246, pushing dev, or rebasing during this recon. None
   of those happened.
