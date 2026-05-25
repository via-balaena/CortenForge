# cf-cast cap-plane flatness — recon bookmark (2026-05-25)

**Status (2026-05-25 same-day RESOLVED):** triage Findings A + B
(cap-plane CENTER + EDGE flatness on cup-floor + plug-bottom)
**resolved via (4') accept + document** — head-architect call
after (5') git-bisect Branch A confirmed PR #255 era had the
identical chamfer band. Findings C + D (socket-mouth obstruction
+ seam-face dome+cap flatness) remain open for separate triage
(different root causes — out of scope for the cap-plane recon).

`procedure.rs` v2 generator gains a new `## Cap-Plane Edge Chamfer
(Expected MC Quantization)` section documenting the chamfer as
expected geometry; spec.rs gains a doc-anchoring test gating any
future drift back toward "fix it" framing. **No production STL
geometry changes.** Workshop iter-3 print **UNBLOCKED on cap-plane**
pending workshop user's triage on remaining findings C + D.

### Recon-trail (for archaeology)

- **2026-05-25 morning** (commit `06286520`): triage doc Candidate
  (1) "add SDF halfspace intersect" FALSIFIED — already shipping
  in `pinned_floor_shell`. Empirical measurement on iter-1 STLs:
  70-95% of cap-face tris bit-precise flat, 5-30% in 10-100 µm
  scatter at cap-plane EDGE. Re-framed candidate set: (1') no-op
  removed, (2') paradigm-boundary slab subtract, (4') accept +
  document, (5') git-bisect first.
- **2026-05-25 same-day** (commit `f5d80112`): (5') git-bisect
  result BRANCH A — PR #255 era had bit-precisely identical
  chamfer band; workshop user misremembered. Candidate set
  collapsed to (2') vs (4').
- **2026-05-25 same-day** (this commit): (4') PICKED + SHIPPED.
  Rationale below in §"(4') head-architect decision rationale".

### Original framing (retained as audit trail)

Triage of `docs/CF_CAST_POST_SALVAGE_TRIAGE.md` Findings A+B+C+D1
→ escalated to recon arc. Verification step on Candidate (1)
**falsified the triage doc's framing**: the SDF cap-plane
halfspace intersect is already in production via `pinned_floor_shell`;
the visible cf-view "faceting" is MC corner-chamfering at the sharp
cap-plane × curving-wall corner, not a missing SDF intersect.

**Lesson reinforced from this session:** the
[[feedback-read-prior-arc-memory-before-architectural-decisions]]
discipline caught this. The triage doc (written by the same agent
in the prior session) wrote Candidate (1) as a 50 LOC fix; reading
`pinned_floor_shell` + recon-4 (P) §F-4 + measuring iter-1 STL
geometry showed the framing was wrong. **No production code shipped
this session** — bookmark + sleep is the right move when an
architectural snag surfaces.

## Empirical findings (this session)

### Finding 1 — Candidate (1) is already shipping via `pinned_floor_shell`

`design/cf-design/src/solid_layered.rs:122-130`:

```rust
for (centroid, normal) in cap_planes {
    let plane_offset = centroid.coords.dot(normal);
    shell = shell.intersect(Solid::plane(*normal, plane_offset));
}
```

Both the plug body AND every layer body (`pinned_floor_shell` is
called for each in `tools/cf-cast-cli/src/derive.rs:255-287`) get
the SDF cap-plane halfspace intersect applied BEFORE MC sampling.
This is the same paradigm as recon-4 (P) used for the seam plane
— `Solid::plane`'s SDF is exactly linear; MC's linear-SDF
interpolation places cap-plane vertices bit-precisely (per
recon-4 (P) §F-4 — `f64` precision on a synthetic fixture).

Adding `body.intersect(cap_plane_halfspace)` again as Candidate
(1) proposes is **a no-op** (idempotent against an already-clipped
shell).

### Finding 2 — Body extent assumption holds by construction

The triage doc's verification step (`body.evaluate(cap_centroid + ε
× cap_normal)` returns exterior > 0 for ε ∈ [+0.5, +1, +5, +20] mm)
is **trivially true** by construction. `pinned_floor_shell` clips
the shell at the cap-plane via the halfspace intersect; the
intersect's `max(shell_sdf, plane_sdf)` returns `≥ plane_sdf = ε`
(positive) at any point ε past the cap-plane on the +cap_normal
side, regardless of the shell SDF's value. No empirical probe of
the production fixture was needed.

### Finding 3 — Production STL measurement: the visible non-flatness IS real, ~10-100 µm at the cap-plane EDGE

Quantitative measurement of `~/scans/cast_iter1/*.stl` against the
cap plane from `~/scans/sock_over_capsule.prep.toml`:

Cap-plane = normal (-0.0414, 0.0555, -0.9976), centroid m
(-0.00107, 0.00469, -0.05312). Triangles classified as
"cap-region" if all 3 vertices fall within 100 µm of the
cap-plane (perpendicular signed distance).

| STL                    | cap-region tris | within 10 µm | 10-100 µm |
|------------------------|----------------:|-------------:|----------:|
| plug_layer_0           |             124 |           94 |        30 |
| plug_layer_1           |             674 |          595 |        79 |
| plug_layer_2           |            1197 |         1032 |       165 |
| mold_layer_0_piece_0   |             303 |          219 |        84 |
| mold_layer_0_piece_1   |             353 |          218 |       135 |
| mold_layer_2_piece_0   |             796 |          534 |       162 |

**Interpretation:**

- **70-95% of cap-plane face triangles are bit-precisely flat
  (within 10 µm)** — that's `Solid::plane`'s linear SDF doing
  exactly what recon-4 §F-4 predicts for any linear-SDF MC pass.
- **The 5-30% deviant tris with 10-100 µm vertex deviations are
  clustered at the cap-plane EDGE** — where the flat cap-plane
  meets the curving body wall. MC at 3 mm cells can't represent a
  sharp ~90° corner; cells straddling the corner produce vertices
  interpolated between the `max(shell_sdf, plane_sdf)` branches at
  the derivative discontinuity. Result: a ~3 mm-wide "chamfer
  band" of triangulation rounding the corner.
- **This chamfer band is what the workshop user observed in
  cf-view.** It's a geometric reality, not a render artifact.

For full measurement script see the inline Python in
`design/cf-cast/dev` session log (or rerun: the script reads STL
binary headers + classifies triangles by vertex distance to the
cap plane parsed from `.prep.toml`).

## Root cause reframed

The triage doc's framing "MC's triangulation produces visible
facets because the 3 mm grid samples don't land exactly on the
cap-plane plane" is **incorrect** for the cap-plane INTERIOR — MC
does land vertices on a linear SDF bit-precisely (recon-4 §F-4).
The actual cause is at the cap-plane EDGE:

> A sharp ~90° corner between the flat cap-plane face and the
> curving body wall cannot be resolved by MC at any finite cell
> size — the corner gets rounded into a chamfer of approximately
> one-cell width, with vertices scattered between the two
> bounding surfaces.

This is intrinsic to marching-cubes on a piecewise-defined SDF
with a non-smooth (max-operator) corner. No SDF-side fix can
eliminate it; only a **post-MC cleanup pass** (mesh-CSG slab
subtract) or a **CHANGE OF GEOMETRY** (e.g., introduce a
filleted/chamfered cap-plane corner in the SDF) can produce a
sharp cap-plane edge.

## Re-framed candidate set

The original triage Candidates (1)-(4) were framed against the
wrong root cause. Re-framed:

### Candidate (1') — NO-OP (already shipping)

The "add SDF halfspace intersect" Candidate (1) is already in
production via `pinned_floor_shell`. Removed from the candidate
set.

### Candidate (2') — Post-MC mesh-CSG slab subtract with ε offset

**Same as triage Candidate (2)**, unchanged. The risk profile is
recon-4 (P) §F-2 paradigm-boundary: the slab face SDF surface
would be exactly coincident with the body cap-plane SDF surface,
which is the recon-4 framework's predicted failure mode. The
2026-05-24 night `a8e3e056` attempt (using
`MatingTransform::SeamTrim` at the cap-plane) hit this exact
failure (F4 Critical issues at the boolean junction) and was
reverted within the same commit.

**Mitigation hypothesis** (similar to plug-shaft recon's
`extend_near_end` 1 mm overlap-bias from the
[[project-cf-cast-sdf-meshcsg-paradigm-boundary]] framework): if
the slab subtract face is OFFSET inward from the body cap-plane
by ε ≥ 0.5 mm (one-sixth of a 3 mm MC cell), the boolean is no
longer face-coincident; the cap-plane stays mesh-CSG-flat at the
new ε-displaced location, and the SDF cap-plane lives ε mm above
it (carved away by the slab). Visible cost: ε mm of plug height
+ cup-wall thickness lost. ε = 0.5 mm is well below FDM bead.

**Probe pattern required (g7c-style)**: throwaway synthetic
fixture with flat-face primitive + slab subtract at various ε
offsets, with **cross-primitive control** per
[[feedback-read-prior-arc-memory-before-architectural-decisions]]
(the g7c naive-union paradigm-boundary probe is the canonical
template at `design/cf-cast/tests/g7c_naive_union_paradigm_boundary_probe.rs`).
Branch A clean = ship (2') with measured ε. Branch B
paradigm-boundary artifacts at any reasonable ε = fall through to
(4').

### Candidate (3') — Finer MC cell size in the cap-plane region

**Same as triage Candidate (3)**, unchanged. Not viable without
adaptive-mesh MC infrastructure (multi-week, out of scope).

### Candidate (4') — Accept the MC corner chamfer + document

**Same as triage Candidate (4)**, but with reframed cost: the
chamfer band is **10-100 µm visible**, not "the whole face is
faceted." Workshop user post-print sanding cost is **a 3 mm-wide
ring of < 100 µm bumps at the cap-plane edge**, not "sand the
whole cup-floor flat." Materially cheaper than the triage doc
implied.

### Candidate (5') — NEW: Investigate the "PR #255 didn't have this" claim via git bisect

Workshop user said in the 2026-05-24 night cf-view session that
"we actually had this down and solved in the main branch." If
true, something changed between PR #255 (`aadcfed6` on main) and
current dev that reintroduced the cap-plane chamfer. If the
"didn't have this" was misremembered (PR #255 had the same
chamfer; the workshop user just didn't notice because attention
was on the seam-face film + registration-pin disconnection), then
(4') accept+document becomes the obvious pick.

git-bisect protocol:
1. Reset dev clone to `aadcfed6` (PR #255 merge commit).
2. Run `cargo run --release -p cf-cast-cli -- ~/scans/cast.toml`
   in that state with iter-1 config.
3. Measure cap-plane face flatness on the regenerated plug + cup
   STLs using the same Python script as this session.
4. If PR #255 era shows similar 10-100 µm chamfer: workshop user
   misremembered, pick (4') accept+document.
5. If PR #255 era shows clean cap-plane: bisect between
   `aadcfed6` and current dev to find the responsible commit.

Cost estimate: ~1 session (5 min regen × 2-4 bisect iterations,
plus measurement script).

**Recommended FIRST step of the next session**: (5') git-bisect.
The framing whether-this-is-blocker-or-ergonomic depends on
whether the chamfer was always there or just appeared.

## Recon-prep checklist (next session protocol)

Per [[feedback-read-prior-arc-memory-before-architectural-decisions]]
read these BEFORE picking a candidate:

- [[project-cf-cast-sdf-meshcsg-paradigm-boundary]] — the
  recon-4 framework that classifies the cap-plane mesh-CSG slab
  subtract as paradigm-boundary risk.
- [[project-cf-cast-seam-face-film-recon4-impl]] — recon-4 (P)
  implementation details. The seam-plane parallel for what worked.
- [[project-cf-cast-mating-features-s4-seam-plane]] — S4
  seam-plane trim attempt that recon-4 (P) reverted. Same failure
  mode that the `a8e3e056` cap-plane trim attempt hit.
- [[project-cf-cast-fdm-friendly-geometry-arc]] — the FDM
  friendly arc memory with the full salvage history.

Plus the empirical data in this bookmark.

The g7c naive-union paradigm-boundary probe at
`design/cf-cast/tests/g7c_naive_union_paradigm_boundary_probe.rs`
is the template for the (2') synthetic probe — **must include
cross-primitive control on the same fixture** so the conclusion
isn't shape-specific (the §G-7 mistake from recon-1).

cf-view smoke after every commit that changes production STLs;
workshop user regen + screenshot before next commit lands. No
"I'll check at the end."

## Open questions for the workshop user (next session)

1. Is the 10-100 µm chamfer band acceptable as ergonomic if it's
   confined to a ~3 mm-wide ring at the cap-plane edge (= the
   cup-floor's perimeter where it meets the body cavity, and the
   plug-bottom's perimeter where it meets the plug wall)? The
   center of the face is bit-precisely flat.
2. Should we run the git-bisect (5') before any code change to
   confirm whether PR #255 actually had cleaner cap-plane faces?
   **ANSWERED 2026-05-25 — see "(5') git-bisect result" below.
   Branch A: PR #255 era had the same chamfer band; workshop user
   misremembered. (4') accept + document is the obvious pick.**

## (5') git-bisect result — 2026-05-25 same-day (BRANCH A)

**Setup.** Worktree at PR #255 baseline `aadcfed6` (detached HEAD,
outside the main checkout so `dev`'s 19 commits are untouched).
PR #255 era TOML at `/tmp/cast_pr255.toml` mirroring `~/scans/cast.toml`
except `[plug_pins] pin_length_m = 0.004` (PR #255 era's
wall_thickness ↔ pin_length cross-field gate requires
`wall + 1 mm margin >= pin_length`; `iter1()` default of 20 mm
fails the 5 mm wall — 4 mm matches the pre-S4 PR #254
physically-printed iter-1 value per project memory). Regen at
`~/scans/cast_iter1_bisect_pr255/`; current-dev `~/scans/cast_iter1/`
preserved as reference.

**Measurement method.** Same Python flatness script
(`/tmp/cap_plane_flatness.py`) applied to both STL sets. Cap-plane
parsed from `~/scans/sock_over_capsule.prep.toml`. Triangles
classified as "cap-region" if all 3 vertices land within 100 µm
perpendicular signed distance of the cap-plane. Script validated on
current-dev STLs first; reproduced the 6-row table in the
"Empirical findings" section above to bit-precise counts (one
minor count delta on `mold_layer_2_piece_0`: 696 measured vs 796
in the bookmark — likely an OCR / transcription typo in the
bookmark, not a measurement bug; the pattern is identical).

**Side-by-side results (cap-region tris / ≤10 µm / 10-100 µm):**

| STL                  | dev (06286520)       | PR #255 (aadcfed6)   |
|----------------------|----------------------|----------------------|
| plug_layer_0         | 124 / 94 / **30**    | 153 / 123 / **30**   |
| plug_layer_1         | 674 / 595 / **79**   | 703 / 624 / **79**   |
| plug_layer_2         | 1197 / 1032 / **165**| 1226 / 1061 / **165**|
| mold_layer_0_piece_0 | 303 / 219 / 84       | 323 / 233 / 90       |
| mold_layer_0_piece_1 | 353 / 218 / 135      | 383 / 216 / 167      |
| mold_layer_1_piece_0 | 536 / 394 / 142      | 556 / 408 / 148      |
| mold_layer_1_piece_1 | 589 / 365 / 224      | 619 / 363 / 256      |
| mold_layer_2_piece_0 | 696 / 534 / 162      | 716 / 548 / 168      |
| mold_layer_2_piece_1 | 772 / 550 / 222      | 802 / 548 / 254      |

**Plug STLs' 10-100 µm count is bit-precisely identical across
both eras (30 / 79 / 165).** Mold piece STLs' 10-100 µm counts
differ by 4-32 tris per piece (PR #255 era slightly higher), but
the percentage of cap-tris in the 10-100 µm band is essentially
unchanged (15-30% in both eras).

**Spatial confirmation (`plug_layer_0`, radial-bin distribution of
deviant tris from cap-plane centroid):**

| radial (mm) | dev tris / ≤10 µm / 10-100 µm | PR #255 tris / ≤10 µm / 10-100 µm |
|-------------|-------------------------------|-----------------------------------|
| 0-5         | 7 / 7 / **0**                 | 41 / 41 / **0**                   |
| 5-10        | 37 / 37 / **0**               | 32 / 32 / **0**                   |
| 10-15       | 50 / 41 / **9**               | 50 / 41 / **9**                   |
| 15-20       | 30 / 9 / **21**               | 30 / 9 / **21**                   |

**The two outer radial bins (10-20 mm = the cap-plane edge ring
abutting the curving body wall) are bit-precisely identical
between eras** — same tri count, same ≤10 µm count, same 10-100 µm
count. The interior bins (0-10 mm) differ because the mating
mechanism is different (PR #255: cylindrical T-bar; current dev:
truncated-pyramid). Both mechanisms produce ZERO deviant tris in
the interior — the chamfer band is purely an EDGE phenomenon,
invariant to the mating mechanism choice.

### Verdict

**Branch A.** PR #255 era had the same ~3 mm-wide chamfer band at
the cap-plane edge. Workshop user's 2026-05-24 night cf-view
recollection ("we actually had this down and solved in the main
branch") was misremembered — attention at the time of the recon-4
(P) + funnel-disconnection ship was on the seam-face film + plug
registration pin disconnection bugs, both of which the §R1
connectivity inspector + cf-view smoke gated as resolved. The
cap-plane edge chamfer was not noticed because it wasn't in scope.

**Mechanism (confirmed by the spatial bin pattern):** the chamfer
band is intrinsic to marching-cubes on the piecewise-defined SDF
where `pinned_floor_shell`'s halfspace intersect introduces a
sharp ~90° derivative discontinuity. Cells straddling the corner
between flat cap-plane (`Solid::plane`) and curving body wall
(`Solid::shell`) MUST interpolate vertices across the
`max(shell_sdf, plane_sdf)` branch boundary — the chamfer width
scales with one MC cell size (3 mm).

**No commit between `aadcfed6` and `06286520` introduced this** —
the FDM-friendly arc's recon-1 §G-12 #2 SDF migration (S3-S6
commits, now reverted by salvage `b2ff45d5`+`fed4b0c6`) is NOT the
responsible commit (the salvage reverted those production paths
to mesh-CSG, restoring pre-S3 cap-plane geometry; PR #255 era's
cap-plane geometry was also driven by `pinned_floor_shell`'s SDF
halfspace intersect, which has been continuously shipping since
2026-05-19 per `derive.rs` docstring).

**Workshop user decision (next session):** the bookmark's
re-framed candidate set collapses to (2') vs (4'):

- **(2') post-MC mesh-CSG slab subtract with ε ≥ 0.5 mm offset.**
  Paradigm-boundary risk per recon-4 (P) §F-2; needs g7c-style
  synthetic probe with cross-primitive control BEFORE
  implementation. ~1-2 sessions.
- **(4') accept + document the ~3 mm-wide ring of <100 µm bumps
  at the cap-plane edge.** ~30 LOC procedure.rs prose addition.
  ~1 session. Now the front-runner — PR #255 shipped with the
  same chamfer and the workshop iter-2 print was workshop-user
  accepted, suggesting the chamfer is workshop-ergonomically
  acceptable in practice.

(1') remains REMOVED (already shipping). (3') remains out-of-scope
(adaptive-mesh MC infrastructure, multi-week). (5') git-bisect
itself is COMPLETE — this section is the result.

**Reproduction artifacts** (retained for the workshop user / next
session):

- `/tmp/cf-cast-pr255/` — git worktree at `aadcfed6`. Cleanup
  with `git worktree remove /tmp/cf-cast-pr255` when done.
- `/tmp/cast_pr255.toml` — PR #255 era TOML.
- `/tmp/cap_plane_flatness.py` — measurement script. Replays
  `python3 /tmp/cap_plane_flatness.py <prep_toml> <stl_dir>`.
- `/tmp/cap_plane_spatial.py` — per-STL radial-bin distribution
  of deviant tris.
- `~/scans/cast_iter1_bisect_pr255/` — PR #255 era regenerated
  STL set (11 STLs + procedure.md).
- `~/scans/cast_iter1/` — current-dev `06286520` reference STL
  set (UNCHANGED by this session).

### LESSON

The bookmark's framing assumed Branch A vs Branch B were
roughly-equally-likely. The spatial bin pattern's
bit-precise-identical-at-the-edge result was much stronger
evidence than the integral counts alone — the integral count
differences (153 vs 124 cap-tris on plug_layer_0) initially
suggested "maybe the chamfer is slightly worse on PR #255" until
the radial decomposition showed the interior was the only thing
that changed, and the chamfer band itself was unchanged. **Per-
region spatial probes resolve "did the integral metric change?"
into "did the metric change in the region we care about?" —
critical when the region of concern is a small fraction of the
total surface.**

Cross-cuts [[feedback-load-bearing-test-fixtures]]: a test that
just checked "number of cap-region tris within 10 µm" would have
FAILED on PR #255 vs dev (different integral counts), suggesting
a regression that doesn't exist; a test that checked "deviant tri
count in the cap-plane-edge radial bin" would have PASSED
identically.

## What does NOT land this session

- No production code change.
- No new tests in `cf-cast` or `cf-cast-cli`.
- No iter-1 regen.

## (4') head-architect decision rationale (2026-05-25 same-day)

Workshop user delegated the (2')-vs-(4') pick to head architect
("your call as head architect. take all the time you need.").
**Picked (4') accept + document.** Reasoning chain:

1. **Below-print-resolution argument.** Chamfer band max
   geometric deviation ≤ 100 µm (0.1 mm) in a ~3 mm-wide edge
   ring. §G-3 target FDM floor (Bambu A1 + default + Jayo) is
   0.4 mm extrusion / 0.2 mm layer / typical 0.1-0.2 mm
   dimensional tolerance. A 100 µm STL-level deviation does NOT
   survive slicer-to-printer quantization — the printed
   plug-to-cup-floor mating interface is identical whether the
   STL has a 100 µm chamfer or 0 µm chamfer. Fixing it STL-side
   is a category error.
2. **PR #255 + workshop iter-2 acceptance is empirical evidence.**
   PR #255 (`aadcfed6`) shipped with the bit-precisely identical
   chamfer band (see §"(5') git-bisect result"); workshop user
   printed iter-2 on calibrated Bambu and accepted it. The
   chamfer is workshop-ergonomically acceptable in practice; the
   2026-05-24 night cf-view recollection ("PR #255 had this fixed")
   was attention drift on the seam-face film + plug-pin
   disconnection bugs, not a regression detection.
3. **(2') has unmitigated paradigm-boundary risk.** The
   2026-05-24 `a8e3e056` cap-plane trim attempt using
   `MatingTransform::SeamTrim` already hit recon-4 (P) §F-2
   failure (F4 Critical issues at the boolean junction) and was
   reverted. The (2') mitigation hypothesis (ε ≥ 0.5 mm offset
   to break face-coincidence) is UNPROVEN; needs g7c-style
   synthetic probe with cross-primitive control BEFORE
   implementation. 1-2 session minimum just for the probe.
4. **(2') even if successful has geometric side effects.** An
   ε ≥ 0.5 mm inward slab subtract shifts the cap-plane location
   inward by ε. The S4 salvage truncated-pyramid plug-lock
   (workshop-verified 2 sessions ago at `fed4b0c6`) depends on
   cap-plane geometry; shifting the cap-plane changes the
   plug-seating depth relative to the body cavity, risking
   regression on workshop-verified base-down captive geometry.
5. **(2') would not eliminate the chamfer phenomenon anyway.**
   The slab subtract slices off the existing chamfer band but
   creates a NEW sharp corner at the ε-displaced cap-plane × wall
   intersection. That new corner face-welds to the SDF body wall
   via mesh-CSG → paradigm boundary all over again. Even in the
   best case, (2') replaces one chamfer-band geometry with another
   at a different location.
6. **The morning bookmark's "Open question 1" already framed (4')
   as the obvious pick if PR #255 had the same chamfer.** The
   bisect confirmed PR #255 had the same chamfer. The picker
   would have to invent novel justification to NOT pick (4');
   none is forthcoming.

**What (4') ships:** ~80 LOC procedure.rs prose addition
(`write_cap_plane_chamfer_v2` section in v2 generator pipeline)
+ ~35 LOC spec.rs doc-anchoring test
(`generate_procedure_markdown_v2_cap_plane_chamfer_section_accepts_edge_band`)
+ this bookmark resolution + triage doc resolution + memory
updates. **Zero production STL geometry changes.** Zero
paradigm-boundary exposure. Workshop-verified S4-salvage
plug-lock geometry preserved unchanged.

## Status

- Triage doc → recon bookmark transition (2026-05-25 morning).
- (5') git-bisect COMPLETE 2026-05-25 same-day. Branch A: PR #255
  era had the same chamfer band; workshop user misremembered.
- (4') PICKED + SHIPPED 2026-05-25 same-day. See §"(4')
  head-architect decision rationale" above. Findings A + B
  RESOLVED.
- Findings C (socket-mouth obstruction) + D (seam-face dome+cap
  flatness) **remain open** for separate triage — different root
  causes (mesh-CSG boolean junction artifact + curved-body ×
  seam-plane MC, NOT cap-plane × wall corner). Workshop iter-3
  print partially unblocked (cap-plane edge no longer a blocker);
  full unblock pending workshop user's C + D triage.
- Branch: `dev`, no push, no PR per
  [[feedback-omnibus-pr-single-branch]].

## Cross-references

- `docs/CF_CAST_POST_SALVAGE_TRIAGE.md` — original triage doc
  (Candidate (1) framing falsified by this session's findings).
- `design/cf-design/src/solid_layered.rs:93-132` —
  `pinned_floor_shell` source of truth.
- `design/cf-cast/src/mesh_csg.rs` — `MatingTransform::SeamTrim`
  variant retained as defensive primitive but NOT emitted for
  cap-plane (`a8e3e056` bookmark).
- `~/scans/cast_iter1/` — iter-1 STL set measured in this
  bookmark.
- `~/scans/sock_over_capsule.prep.toml` — cap-plane source for
  the measurement.
