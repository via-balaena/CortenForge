# Compliant cavity–probe conformity

Pin down the canonical problem: a compliant elastomeric cavity — an open-ended thick-walled sleeve with a rounded rim and a closed base — is inserted over a rigid probe, loaded to prescribed engagement, and allowed to reach mechanical equilibrium. The quantities the book returns to across Parts 1–12 are the cavity's deformed configuration, the contact-pressure field on its inner surface, the axial force transmitted to the probe, and the [reward](../01-reward.md) evaluated over these fields. The [related-problems sibling](01-related-problems.md) shows which engineering problems this abstraction covers; the [why-canonical sibling](02-why-canonical.md) shows which subsystems it exercises.

## Geometry

The canonical shape at design-mode scale:

- **Cavity.** Thick-walled cylindrical sleeve of elastomer. Inner radius $r_c$ set near the probe radius $r_p$ (interference fit), wall thickness $t_c$ of roughly one-third to one-half of $r_c$, axial length $L_c$ several times $r_c$. Closed at the base, open at the mouth. The rim is rounded rather than sharp.
- **Probe.** Rigid cylinder of radius $r_p$, optionally with a tapered or rounded tip. Treated as a contact partner from [`sim-mjcf`](../../110-crate/02-coupling/00-mjcf.md), moved axially along the cavity axis.

Absolute length is intentionally not pinned at this leaf — the steady-state, body-force-free limit of the primal solve is scale-invariant under geometric similarity (see nondimensional groups below). The [material-data chapter](../04-material-data.md) and the [Phase E GPU budgets](../../110-crate/03-build-order.md#the-committed-order) pin the absolute scale by fixing tet counts at roughly centimeter-scale parts: ~30k tets in design mode, ~5k tets in experience mode.

## Loading

Two load modes, both canonical-problem-valid and both exercised in Part 10's optimization loop.

**Prescribed axial engagement.** The probe is driven axially from outside the cavity to a target engagement depth $\delta$, then held. The reward is evaluated at the held steady state. Default mode for the conformity studies in [Part 10 Ch 00 forward](../../100-optimization/00-forward.md) and [Part 12 Ch 06 optimization milestone](../../120-roadmap/06-optimization.md).

**Prescribed axial force.** The probe is loaded under axial-force control. This mode exercises the [effective-stiffness reward term](../01-reward/03-stiffness-bounds.md) — for a given applied force, how much displacement is consumed by the cavity's compliance and how much transmits.

Transient loading — engagement pulses, oscillatory squeeze — is downstream of the canonical problem: the [wobble failure mode](../02-what-goes-wrong/01-wobble.md) and [Part 2 Ch 07 viscoelasticity](../../20-materials/07-viscoelastic.md) exercise the time-dependent side, but canonical-problem runs default to steady-state unless explicitly named.

## Boundary conditions

- **Proximal fixed ring.** The closed base has a Dirichlet boundary — a ring of pinned vertices approximating bonding to a rigid fixture.
- **Probe–cavity contact.** IPC-enforced non-penetration ([Part 4 Ch 00](../../40-contact/00-why-ipc.md)) over the cavity's inner surface against the probe's outer surface, with smoothed Coulomb friction at coefficient $\mu_c$.
- **Probe motion.** Prescribed axial displacement or axial force, depending on load mode. Entered as a rigid-body contact partner so the IPC solver treats it as a kinematic constraint target, not a deformable body.
- **Free outer and rim surfaces.** Traction-free. The open rim is not pinned and deforms freely as the probe advances; this is the defining feature of the problem — rim behaviour under insertion is where the [rim-deformation failure mode](../02-what-goes-wrong/04-rim.md) shows up.

Self-contact is not exercised by the canonical default geometry — the cavity's inner surface contacts the probe, not itself — but the [Part 4 Ch 03 self-contact](../../40-contact/03-self-contact.md) machinery is available for variants (multi-layer sleeves, folded rims) that engage it.

## State variables

The simulator carries a compact state over the canonical problem:

- Per-vertex position $x \in \mathbb{R}^{3N}$ for the cavity's $N$-vertex tet mesh. Vertex velocity $v$ is carried in dynamic runs; absent in pure steady-state runs.
- Per-tet deformation gradient $F$ and derived measures ($C$, $J$, invariants), computed from vertex positions via the per-element shape-function gradient.
- Per-active-pair contact gap $d$ to the probe, updated each step via the CCD pass.
- Contact-pressure field on the inner surface, computed as the surface traction from the IPC barrier's per-pair normal contribution.

The rigid probe's state is a 6-DOF pose with prescribed axial trajectory; it does not enter the primal Newton solve and does not carry a strain measure.

## Nondimensional groups

The canonical problem is parameterized by a small set of dimensionless groups. For steady-state runs without body forces, absolute length drops out of the primal solve; inertia (through $\rho$) and gravity break scale invariance only when they are enabled. The optimizer searches over these ratios and the discrete Shore-grade index.

| Group | Meaning | Typical range |
|---|---|---|
| $r_p / r_c$ | Probe radius over cavity inner radius | 0.9–1.2 (loose fit to interference) |
| $t_c / r_c$ | Wall thickness over inner radius | 0.3–0.7 |
| $L_c / r_c$ | Axial length over inner radius | 2–8 |
| $\delta / r_c$ | Axial engagement depth over inner radius | 0.5–4 |
| $\mu_c$ | Coulomb friction coefficient at inner surface | silicone-on-smooth-probe default; probe-coating dependent |
| Shore-grade index | Base silicone selection | discrete, from [Part 1 Ch 04](../04-material-data.md) |

The Shore-grade index selects the small-strain modulus $E$ and with it the full [material-data row](../04-material-data/00-ecoflex/00-mechanical.md) (density, Poisson ratio, tensile strength, hyperelastic fit). The canonical default is Ecoflex 00-50 per the [Ecoflex mechanical leaf](../04-material-data/00-ecoflex/00-mechanical.md) ("Canonical-problem default when a single Ecoflex grade is chosen").

## What this formulation commits the book to

- **Thick-walled compliant sleeve over a rigid probe is the geometric shape the subsystem claims are validated against.** Every "the canonical problem exercises subsystem X" argument in Parts 2–12 refers to this geometry.
- **The probe is rigid.** `sim-soft` does not model probe compliance in the canonical default. A compliant probe is a downstream extension implicit in the [multi-material support](../../20-materials/09-spatial-fields.md), not a canonical feature.
- **Steady-state equilibrium at prescribed engagement is the default reward-evaluation regime.** Dynamic loading is an extension, not the base case; the [reward chapter](../01-reward.md) defines its objectives on steady-state fields.
- **Absolute length drops out of the steady-state primal solve.** The optimizer searches in ratio space; inertia and gravity break scale invariance only when explicitly enabled. A single canonical-problem sweep covers parts from catheter-scale to finger-gripper-scale without re-tuning the solver.

These four commitments are inherited by the siblings and by every later Part that invokes "the canonical problem."
