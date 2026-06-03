# CortenForge — Mission

*Established 2026-06-03.*

> **Close the loop between a real human body and the device that helps it — and keep that loop differentiable end to end.**

CortenForge is, at its core, a **software development kit for the mechatronics and simulation space** — a composable component library whose goal is to provide the building blocks for *everything* needed to go from the physical world to a simulated, designed, optimized, and manufactured system and back: geometry, parametric design, meshing and digital fabrication, rigid- and soft-body physics, control and reinforcement learning, and sim-to-real calibration. **The kit is the product.**

Because those components are general, the surface of what they unlock is near-limitless — robotics, soft robotics, biomechanics, generative design and digital fabrication, custom-fit products, embodied-AI research, and adjacencies we have not yet imagined. We do not constrain the kit to one application; we constrain ourselves to making the components **excellent and composable**.

To prove the components are real and that they compose end to end, our defining undertaking is to build the hardest thing we can with them: a **differentiable body-to-device co-design loop** — one differentiable representation carried across the full path **physical → digital → physical** — in the domain of **patient-specific assistive and rehabilitation robotics**, with a powered, RL-controlled **exoskeleton** as the capstone demonstration. **We have not built it yet.** It is the goal we are building toward — the proving ground that disciplines every component along the way.

There is a real person at the end of that loop. That is the point of the capstone — and an SDK puts the loop, and everything adjacent to it, in the hands of everyone who builds.

---

## The thesis

The components, composed into one system, form a spine — and the capstone is what will make that spine *whole and differentiable*. The loop we are building toward:

1. **Body in.** A real body is scanned and cleaned into a watertight digital twin.
2. **Twin built.** That twin becomes simulatable — rigid-body dynamics, muscles and tendons, and soft-tissue finite-element contact.
3. **Device designed.** A device is described parametrically — its passive morphology (implicit-surface geometry, graded multi-material, lattice compliance fields) *and* any active elements (tendons, actuators).
4. **Co-optimized.** Geometry **and** controller are optimized *through the differentiable simulator* against a clinical objective — pressure relief, restored gait, reduced metabolic cost, fall risk.
5. **Made.** The result is gated for manufacturability and physically produced — molds, prints, and human-readable fabrication procedures.
6. **Loop closed.** The real device is instrumented, measured, and the measurement feeds back to recalibrate the twin via system identification — tractable precisely *because* the simulation is differentiable.

The defining property: a gradient that flows from **"how the manufactured device performs on the real body"** all the way back to **the geometry parameters and the control policy.**

## The moat

Almost every lab on Earth has **either** a differentiable physics/RL simulator **or** a scan-to-fabrication pipeline. Having **both, sharing one differentiable substrate, under one roof** is what makes this possible — and it cannot be bought off the shelf.

## The breadth

The capstone is one path through the kit; it is not the kit's edge. The same components serve a near-limitless set of adjacent opportunities, for example:

- **Robotics & control** — rigid-body dynamics, URDF/MJCF import, RL, differentiable control for any mechanism.
- **Soft robotics** — hyperelastic FEM, contact, and fabrication for compliant actuators and grippers.
- **Biomechanics & rehabilitation research** — fast, differentiable musculoskeletal simulation.
- **Generative design & digital fabrication** — implicit-surface modeling, graded lattices, printability, mold generation.
- **3D scanning & reverse engineering** — scan repair, SDF conversion, measurement, the full mesh suite.
- **Custom-fit products** — anything shaped to a scanned body, from medical devices to consumer goods.
- **Embodied-AI research & education** — GPU-batched, differentiable environments with a worked-example corpus.

We measure success not only by the capstone, but by how readily someone reaches for a CortenForge component to build something we never planned.

---

## How we sequence: quality and ceiling, not speed

**We do not order this roadmap by how fast we can close the loop.** Demo velocity is the wrong objective. We order it by *quality* and by *ceiling* — building the deepest, ceiling-determining foundations first and to validated, high-fidelity standards. Each layer is a quality gate, not a milestone to rush past.

1. **Substrate fidelity.** Validate the rigid-body engine against ground truth; bring the soft-tissue FEM to *measured* accuracy. The twin's fidelity caps everything downstream — every gradient and every device inherits its error.
2. **The keystone — differentiable soft↔rigid coupling.** Tissue ↔ skeleton ↔ device contact, with gradients through the contact. This is the genuinely open research problem, and *because* it determines the platform's ceiling, it is built first and built correctly — never deferred for a faster demonstration.
3. **System identification, as a first-class discipline.** The twin must be *provably* tied to reality, not merely plausible.
4. **Co-design optimization + differentiable compliance.** Built only on a substrate we already trust.
5. **The capstone demo — the powered, RL-controlled exo/orthosis.** Chosen for ceiling, not for ease; it proves the whole SDK composes end to end and shows what the loop can reach. The passive prosthetic socket is the waypoint that exercises the loop before control enters.

---

## What we already have (recycle)

- **Scan → twin:** scan ingestion, repair, welding, flood-fill SDF (`cf-scan-prep`, `mesh-repair`, `mesh-sdf`).
- **Parametric design:** code-first implicit-surface kernel with parametric optimization (`cf-design`); graded multi-material layer stacks; lattice / TPMS compliance fields (`mesh-lattice`).
- **Manufacture:** mold splitting, demold, pour/vent, printability gating, auto-generated fabrication procedures (`cf-cast`, `mesh-printability`, `mesh-shell`); material and cure databases.
- **Simulate:** hyperelastic soft-body FEM with contact and pressure/stress fields, differentiable (`sim-soft`); a MuJoCo-aligned rigid-body engine in Rust — dynamics, muscles, tendons, actuators, sensors, analytic derivatives, GPU, batching (`sim/L0/*`); an RL chassis (`sim-ml-chassis`, `sim-rl`, `sim-opt`).

## What we must build (the connective tissue)

1. **Differentiable soft↔rigid coupling** — the keystone; an open research problem.
2. **Co-design optimizer** — one outer loop differentiating w.r.t. *both* design and policy parameters.
3. **System-ID / sim-to-real calibration** — fit tissue and material models from real sensor data.
4. **Person-specific musculoskeletal model builder** — scan → scaled skeleton + muscle routing.
5. **Lattice-as-design-variable + manufacturing-constrained optimization** — differentiable compliance fields; printability as a hard constraint.

---

## The capstone demonstration

The SDK's domain is patient-specific **assistive and rehabilitation robotics**. The greater good is personalized assistive technology — and because the deliverable is a *kit*, the leverage is wider than any one device: it is every clinician, researcher, and builder the SDK equips.

The ceiling-defining capstone we are building toward is a **powered, RL-controlled orthosis/exosuit** — the only endpoint that exercises the *entire* SDK at once: scan-derived twin, full rigid-body dynamics, muscles and tendons, soft-tissue contact at the human interface, a control policy trained in the differentiable simulator, and a physically manufactured device. A **passive prosthetic socket** is a deliberate waypoint along the way — same loop, lower stakes, a place to prove fidelity and sim-to-real calibration before control and actuation enter. We aim all the way.

Because the product is an SDK, the quality bar is not just "does the exo work" — it is **API clarity, composability, and worked examples**, so that the path we walk to the capstone is one others can walk to devices we never imagined.

## Honest constraints

This is a multi-year, research-grade program — not an integration exercise. Differentiable contact across soft and rigid bodies is hard. The sim-to-real gap is real. Anything that touches a patient carries clinical-validation and regulatory weight that code alone does not clear. We hold ourselves to *validated against measurement* as the line between an impressive demonstration and a trusted tool — and we are in no hurry to cross it before the quality is real.
