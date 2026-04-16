# Roadmap

## Mission

**Solve X-encoding through biology-inspired simulation: from Langevin models to hardware design rules.**

## The Root Problem

Given a thermodynamic circuit with energy function F, how do you choose the injection protocol so the system relaxes to the correct output distribution Y?

No formal theory exists. Chip engineers currently tune empirically at every scale. This is the bottleneck blocking thermodynamic computing from becoming an engineered technology.

## What a Solution Looks Like

A design rule that says: *for a circuit with coupling J, use noise temperature kT ≈ X, inject with phase lag δ ≈ Y, expect Z% fidelity improvement.* Quantitative, testable, generalizable.

## Where We Are

| Step | Status | Description |
|------|--------|-------------|
| 1. Framework | **Done** | 12 X-encoding principles across 5 Reynolds number regimes |
| 2. 1-particle simulation | **Done** | SR peak validated (kT=1.70, \|t\|=4.52), controls pass |
| 3. Multi-particle validation | **Done** | 4-cell Ising chain: P2 coupling sweep (5 J values), P4 metachronal sweep (4 J × 20 δ), P6 scale sweep (N=4,8,16) |
| 4. Boundary mapping | **Done** | P1 (topology) and P11 (bifurcation) tested and failed — stat-mech transfers, dynamical-systems doesn't |
| 5. Publication | **In progress** | mdbook with three design rules, two boundary conditions, reproducible code |
| 6. Hardware validation | Needs collaboration | Test predictions on Extropic / Normal Computing hardware |

## Three Design Rules (Validated)

1. **Noise tuning (P2):** kT ≈ 2.3 for J < 1.5, kT ≈ 4.3 for J ≥ 2.0. ΔV/kT must stay below 3.0.
2. **Injection timing (P4):** Phase lag δ ≈ π/5 for J < 2 (18–37% improvement). Synchronize for J ≥ 2.
3. **Scale-invariance (P6):** Rules hold from N=4 to N=16 without retuning.

## Two Boundary Conditions (Informative Negatives)

1. **Topology doesn't transfer (P1):** Amplitude dominates in Langevin domain. Use it freely.
2. **Bifurcation doesn't transfer (P11):** No sharp critical point. ΔV axis is forgiving.

## What's Next

### Immediate: Publish the mdbook

The three-pillar story (P2+P4+P6) with boundary conditions is complete. Prose is tightened. Code is in the repo. Ship it.

### Short-term: N-scaling experiment (~2 hours)

Peak synchrony increased with circuit size (0.041 → 0.049 → 0.063 for N=4→8→16). If real, bigger circuits are *better*. A targeted sweep at N=4–32 would confirm or refute this. This could upgrade the P6 result from "rules hold at scale" to "circuits improve at scale."

### Medium-term: Hardware collaboration

Take the design rules and predictions to Extropic and Normal Computing. "Here are three quantitative rules derived from biological navigation principles. Here's where they hold. Here's where they break. Test them on your chip."

## What's NOT on the Path

- Running all 12 biological principles before publishing
- Building CFD infrastructure for principles that need fluid physics
- Testing every algorithm variant on every setup
- Waiting for perfect results before shipping
