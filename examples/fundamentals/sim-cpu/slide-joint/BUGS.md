# Slide Joint — Bugs Found

## BUG-1: Armature double-counting in CRBA

**Status:** FIXED
**Severity:** Physics correctness
**Found:** 2026-03-25
**Fixed:** 2026-03-25 — `sim/L0/core/src/dynamics/crba.rs` Phase 4

CRBA Phase 4 added armature twice: once from `jnt_armature[jnt_id]` and once
from `dof_armature[dof_id]`. Both arrays were populated from the same MJCF
`armature` attribute by the builder (`sim/L0/mjcf/src/builder/joint.rs` lines
126, 156). Result: effective mass = m + 2*armature.

**Fix:** Removed the `dof_armature` addition from CRBA Phase 4. `jnt_armature`
is the canonical source (matches MuJoCo semantics). 2,265 tests pass.

**Verification:** With armature=0.1 restored, period error dropped from 4.5%
to 0.04%.

## BUG-2: Visual geoms generate unwanted collisions

**Status:** Not a bug — standard MuJoCo pattern
**Severity:** Usability / footgun
**Found:** 2026-03-25

All geoms default to `contype=1, conaffinity=1` (full collision). Rail geoms
used as visual guides were colliding with physics bodies, creating friction
that killed oscillation. This is the expected MuJoCo behavior — use
`contype="0" conaffinity="0"` for visual-only geoms.

**Resolution:** Set `contype="0" conaffinity="0"` on rail geoms. This is the
correct MJCF pattern, not a workaround.
