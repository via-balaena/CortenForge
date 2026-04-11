# Layer 1 Spec — Visual Foundations

**Status:** DRAFT
**Date:** 2026-04-11
**Goal:** Every physics feature has a polished visual example.
**Prerequisite for:** Layer 2 (CI & quality gates), Layer 3 (thermo visuals at scale)

> This spec covers the first of 6 foundation layers leading to D4 (sim-to-real).
> It is temporary — archive after Layer 1 ships.

---

## Workflow

Each example domain is a mini-initiative, not a checklist item. The process
for every domain (thermo, actuators, heightfield, etc.) is:

1. **Recon** — read the engine code, the tests, the existing examples. Understand
   the feature deeply. Look for API gaps, rendering gaps, engine gaps.
2. **Spec** — write a per-domain spec (or section in this file) covering: what
   the example should show, what physics it validates, what HUD readouts matter,
   what camera angle makes it legible. Identify infrastructure needs.
3. **Build** — implement one example at a time. Baby steps. Simplest geometry
   first. If the engine is wrong, stop and fix the engine.
4. **Review** — two-pass: numbers pass (Claude checks physics, validation, HUD)
   then visuals pass (user checks camera, aesthetics, clarity, "does it look
   right?"). Only move to the next example after both passes.

This workflow has surfaced real engine bugs in every domain so far (sensors
found `cutoff=0`, mesh-collision found 6 physics bugs, SDF-physics found the
stacking blocker). The examples ARE the integration tests. Rushing them defeats
the purpose.

---

## Scope

Layer 1 has 4 tracks, ordered by priority:

| # | Track | Items | Blocking |
|---|-------|-------|----------|
| T1 | Thermo visual examples | 4 new domains | Layer 3 |
| T2 | Actuator visual review | 9 polish passes | — |
| T3 | Track 1B remaining domains | 3 clean + 1 investigate + 2 defer | Coverage 100% |
| T4 | README + SDF polish | ~25 READMEs + SDF docs | — |

---

## T1 — Thermo-Computing Visual Examples

**Why first:** Phases 1-6 + D1 + D2 are done numerically but have zero visual
proof. These are the first examples using sim-thermostat. Layer 3 builds
directly on this work.

**Integration path:** Examples depend on `sim-thermostat` directly (sim-bevy
does NOT depend on sim-thermostat). Forces install via
`PassiveStack::builder().with(...).install(&mut model)`. sim-bevy handles
visualization of the underlying sim-core Model/Data.

**All systems are 1-DOF** (particle on slide joint). Visuals should show:
- The particle moving in its potential landscape
- Real-time statistics via PhysicsHud (temperature, transition rate, KE)
- The potential function as a background curve or surface

**Recon needed before building:**
- How does `PassiveStack` interact with sim-bevy's step loop? Does the
  `cb_passive` hook fire correctly when Bevy drives stepping?
- Can existing Bevy gizmos draw a potential landscape curve, or do we need
  a new primitive?
- What's the right camera setup for a 1-DOF system? Side view? Top-down?
- Trail/trajectory drawing — does sim-bevy have this, or do we build it?

### T1a — Langevin Oscillator

Simplest thermo visual. 1-DOF damped harmonic oscillator with thermal noise.

- **Physics:** `LangevinThermostat` on a spring-mass system
- **Shows:** Stochastic wiggling superimposed on deterministic oscillation
- **HUD:** Temperature, KE, equipartition ratio (should approach 1.0)
- **Validation:** KE converges to ½k_BT (equipartition theorem)
- **Source fixture:** `tests/langevin_thermostat.rs`

### T1b — Kramers Escape (Bistable Double-Well)

First non-trivial demo. Particle trapped in one well, occasionally thermally
activated over the barrier into the other well.

- **Physics:** `DoubleWellPotential` + `LangevinThermostat`
- **Shows:** Particle position trajectory + potential landscape + transitions
- **HUD:** Temperature, transition count, Kramers rate, well residence time
- **Validation:** Transition rate matches Kramers formula
- **Source fixture:** `tests/kramers_escape_rate.rs`

### T1c — Brownian Ratchet Motor

Directed transport from asymmetric potential + noise. D1 result visualized.

- **Physics:** `RatchetPotential` + `LangevinThermostat` + actuator ctrl
- **Shows:** Ratchet potential shape, particle drift, control signal on/off
- **HUD:** Net current, control state, position, temperature
- **Validation:** Flashing ratchet produces positive net current
- **Source fixture:** `tests/d1b_brownian_ratchet_baselines.rs`

### T1d — Stochastic Resonance

The D2 headline result: noise HELPS signal detection. Suboptimal noise =
no transitions. Optimal noise = transitions sync with drive. Too much noise =
random transitions.

- **Physics:** `DoubleWellPotential` + `OscillatingField` + `LangevinThermostat`
- **Shows:** Drive signal + particle position + transition synchrony
- **HUD:** Temperature, synchrony metric, drive frequency, SNR
- **Validation:** Synchrony peaks at intermediate noise (SR curve)
- **Source fixture:** `tests/d2b_stochastic_resonance_baselines.rs`

### T1 infrastructure needs (discovered during recon)

- [ ] Potential landscape visualization (draw V(x) as a curve behind the particle)
- [ ] Trail/trajectory drawing (position history as fading line)
- [ ] Determine if these need new sim-bevy infrastructure or can use existing gizmos
- [ ] Verify `PassiveStack` + sim-bevy step loop interaction

### T1 directory structure

```
examples/fundamentals/sim-thermostat/
├── langevin-oscillator/
├── kramers-escape/
├── brownian-ratchet/
└── stochastic-resonance/
```

---

## T2 — Actuator Visual Review

**Why second:** 10 examples exist, all 40/40 PASS. Motor is polished
(camera, README, rev display). Examples 2-10 need the same treatment.

**Workflow:** Same recon → spec → build → review cycle, but "build" here is
polish (camera, README, HUD refinement) rather than new code. Still one at
a time, still two-pass review. Each example may surface actuator-specific
engine or API gaps.

### Examples to review

| # | Example | Status | Needs |
|---|---------|--------|-------|
| 1 | motor | POLISHED | — |
| 2 | cylinder | PASS | Camera, README, visual review |
| 3 | damper | PASS | Camera, README, visual review |
| 4 | gear-ratio | PASS | Camera, README, visual review |
| 5 | integrator | PASS | Camera, README, visual review |
| 6 | position-servo | PASS | Camera, README, visual review |
| 7 | velocity-servo | PASS | Camera, README, visual review |
| 8 | force-range | PASS | Camera, README, visual review |
| 9 | transmission | PASS | Camera, README, visual review |
| 10 | stress-test | PASS | README only (headless) |

---

## T3 — Track 1B Remaining Domains

**Why third:** These are the last 6 uncovered example domains from the
coverage spec. Each domain gets the full recon → spec → build → review
treatment. Some have engine or rendering gaps that recon will surface.

| Domain | Engine ready? | Rendering ready? | MJCF syntax? | Notes |
|--------|--------------|-----------------|-------------|-------|
| Heightfield terrain | Yes | Yes | Yes | Hfield loading works (PNG, Phase 7 T1) |
| Adhesion | Yes | Yes | Yes | Body transmission type |
| Collision pairs | Yes | Yes | Yes | All primitive pairs, bitmask filtering |
| Hill muscle | Yes | **Unconfirmed** | **No** | Programmatic Model only. Tendon viz may work but untested. |
| Flex bodies | Yes | **No** | Yes | sim-bevy has zero flex rendering. Rendering gap. |
| Plugins | **No** | No | No | Plugin system (§66) not implemented. |

### Clean wins (Layer 1)

Heightfield, adhesion, and collision pairs have no known gaps — engine,
rendering, and MJCF all work. But "no known gaps" means "gaps not yet found."
The recon pass for each domain will likely surface something. That's the point.

### Hill muscle — investigate during recon

Two unknowns: (1) rendering — does existing tendon/muscle viz work with
`ActuatorDynamics::HillMuscle`? (2) model construction — no MJCF syntax,
so examples use programmatic API. Neither is necessarily a blocker, but both
need investigation before committing to a visual example. The recon pass
will determine whether Hill muscle is a Layer 1 deliverable or a Layer 2
deferral.

### Defer to Layer 2+

- **Flex rendering:** `spawn_model_geoms()` in sim-bevy only handles rigid
  geoms. Flex vertices have no visual representation. This needs sim-bevy
  engineering work (vertex mesh generation + per-frame update system) before
  flex examples can be built. Not example polish — real infrastructure.
- **Plugins:** Depends on §66 (plugin system), not implemented.

---

## T4 — README + SDF Polish

**Why last:** Documentation polish. Can be woven in alongside other tracks.
But even READMEs deserve care — each one should orient a beginner, explain
what physics is being demonstrated, and note what to look for visually.

### Missing READMEs (25 total)

| Category | Count | Examples |
|----------|-------|---------|
| SDF-physics cpu | 13 | 01-sdf-grid through 16-socket (excluding 3 with EXPECTED_BEHAVIOR) |
| Integration | 4 | design-to-sim, design-to-print, sim-informed-design, full-pipeline |
| ML algorithms | 3 | PPO, SAC, TD3 in vec-env |
| Stress tests | 5 | Various headless validators |

### SDF-physics sequence

The SDF-physics examples (01 through 16) are a progressive ladder from basic
SDF grid rendering to complex mechanisms. They're the most visually impressive
examples in the repo but the least documented.

- 0/16 have READMEs (3 have EXPECTED_BEHAVIOR.md instead)
- 11-hinge-free has POLISH_TODO.md indicating incomplete redesign
- No unified README explaining the 01-16 progression

Each SDF example should get the same treatment: recon what it shows, write a
README that orients the viewer, note any gaps found. The unified progression
README ties them together.

### Deliverables

- [ ] README for each of the 16 SDF-physics CPU examples
- [ ] Unified README at `examples/sdf-physics/cpu/README.md` explaining progression
- [ ] Resolve 11-hinge-free POLISH_TODO
- [ ] READMEs for 4 integration examples
- [ ] READMEs for 3 ML algorithm examples
- [ ] READMEs for stress tests that don't have them

---

## Out of Scope (Layer 2+)

- CI pipeline and automated quality gates (Layer 2)
- Large-scale thermo visualizations — parameter sweeps, animated SR curves (Layer 3)
- cf-design bridge — SDF-as-physics-shape (Layer 4)
- Printable device geometry (Layer 5)
- Measurement and calibration (Layer 6)
- Plugin system (§66) and plugin examples
- Flex rendering and flex examples
- Any engine work discovered during recon (gets its own spec, not Layer 1 scope)

---

## Definition of Done

Layer 1 is complete when:
1. 4 thermo visual examples exist, compile, run, and have READMEs
2. All 10 actuator examples are visually reviewed and polished
3. 3 clean Track 1B domains have examples (heightfield, adhesion, collision pairs)
4. All 25 missing READMEs are written
5. SDF-physics sequence is documented with progression README
6. Hill muscle rendering investigated (outcome: visual example or documented deferral)
7. Engine/API gaps discovered during recon are filed as specs (not silently deferred)
