# M2-S4 — Mode-oracle recon (does a credible published silicone biaxial / planar-shear dataset exist?)

*Recon, 2026-06-10. The decision-shaping leaf of M2: no production code — a literature scan
that answers one question and recommends the roadmap fork. Sibling of the M2 recon
(`recon.md`).*

## The chartered question

M1 + M2-S3 validated `sim-soft` to **measured uniaxial** accuracy on Ecoflex 00-30 and 00-10
(datasheet 85 %/195 % RMS → measured ~6 % over λ≤2). The single thing that work did **not**
prove is that the model is right in any deformation mode *other than the one it was fit to* —
and a hyperelastic model fit to uniaxial tension mispredicting equibiaxial/planar is the
canonical failure of exactly this workflow. So deformation **modes (b)** is the highest
fidelity-ceiling next axis. But the M1 oracle (Marechal 2021) is **uniaxial-only**, so (b) is
gated on a *new* oracle. S4 answers: **does a credible published silicone biaxial / planar-shear
stress–stretch dataset exist at the M1 quality bar (raw curves + provenance)?**

## Finding: YES — multi-mode silicone data exists; the best material match is figure-data, not Marechal-style open raw

The literature scan (June 2026) found credible published multi-mode silicone characterization.
Ranked by fit to our need (same material family · modes incl. equibiaxial+planar · provenance):

| # | Source | Material | Modes | Provenance / access | Bar |
|---|---|---|---|---|---|
| 1 | **Liao, Hossain, Yao — "Ecoflex polymer of different Shore hardnesses" (Mechanics of Materials 144:103366, 2020)** + companion "On the stress recovery behaviour of Ecoflex" (2021) | **Ecoflex 00-10…00-50** (our exact anchor family) | **uniaxial + planar (pure-shear) + equibiaxial** tension, DIC, monotonic + cyclic | Green OA preprint (Swansea Cronfa `cronfa53571`; Glasgow eprint 197199). Data in **published figures + fitted params**, no confirmed machine-readable open raw dataset | **Strong** — needs digitization |
| 2 | **Meunier, Chagnon, Favier et al. (Polymer Testing 27:765, 2008)** | unfilled RTV141 silicone (NOT Ecoflex) | **5 modes**: tensile, **pure shear**, compression, plane-strain compression, **bulge (equibiaxial)**, full-field DIC | Widely-cited reference dataset; figure-data | Gold-standard *method*, wrong material |
| 3 | PDMS multi-elastic dataset (Mendeley `mwmzpgrzs3`, **CC BY 4.0 open**; Sci. Rep. 2023) | PDMS (Sylgard-class), not Ecoflex | tensile / compressive / **shear** (UTM) — equibiaxial/planar **not** clearly included | **Open raw** (Excel) + COMSOL files | Open but wrong material + ambiguous modes |
| 4 | Nunes 2011 (Mater. Sci. Eng. A); Moreira & Nunes | PDMS | **simple / pure shear** only (single mode) | figure-data | Single-mode |

**The provenance gap vs M1:** none of these is a Marechal-equivalent — a peer-reviewed dataset
published *expressly as machine-readable open raw data for FE reuse*. The best material match
(#1, Hossain Ecoflex, **same family**) presents its curves as figures, so using it at the M1 bar
means **digitizing the published curves** (WebPlotDigitizer) and vendoring a curated, cited
subsample — a standard, defensible practice that introduces ~1–3 % reading error, but one
provenance tier below M1's machine-readable Zenodo raw data. (The cleanest *open-raw* option, #3
PDMS, is the wrong material and lacks clear equibiaxial/planar.) So (b) is **viable**, just at a
slightly lower provenance tier — to be stated honestly in the gate, exactly as M1 scoped "a
published measurement, not our own cast."

## What (b) actually entails (why it is a phase, not an M2 leaf)

A faithful modes arc is **research-grade**, not a quick closer:

1. **New oracle pipeline** — digitize + curate + provenance the Hossain Ecoflex equibiaxial &
   planar curves (and optionally Meunier as a cross-check on a different silicone). A real
   sub-arc on its own.
2. **New FEM coupons** — an **equibiaxial** symmetry-cell coupon and a **planar-tension**
   (pure-shear) coupon. Both are now *unblocked by the M2-S1 roller BCs* (equibiaxial = drive two
   axes + roller the third symmetry plane; planar = drive one axis, **constrain a second to
   zero** via a roller, free the third) — this is precisely why roller BCs were the right M2
   spine.
3. **★ The real finding it surfaces — the cross-mode model-form question.** Our compressible
   **2-term Yeoh at ν = 0.40** was fit to *uniaxial* and reaches ~6 %. There is strong prior
   reason it will **not** simultaneously fit equibiaxial/planar at that bar: (a) equibiaxial
   loads the volumetric response far harder, so **ν = 0.40 (vs real ≈ 0.4999) likely binds** —
   this is exactly the **deferred near-incompressibility (e): Tet10 + F-bar / mixed-u-p**, which
   M1-S0 retired *for tension* but explicitly flagged "may bind in compression/biaxial"; (b) a
   single 2-term Yeoh fit across modes generally needs a richer invariant basis (I₂ dependence /
   Ogden). So (b) is the trigger that legitimately pulls (e) forward and may extend the model —
   a full recon→spike→implement phase, not a tacked-on leaf.

## Recommendation (head engineer)

1. **Declare M2 COMPLETE after S4.** M2's spine is delivered and merged: roller/per-axis
   Dirichlet BCs (#296), a genuine free-lateral coupon that re-discovers λ_t against measurement
   (#296), and the proof the measured-fidelity machinery generalizes across materials (#297,
   00-10 + 00-30). That was the chartered scope.
2. **Promote deformation modes (b) to its own phase (M3-modes).** The data exists at a usable
   (digitized) bar, roller BCs already unblock the coupons, and it is the highest-ceiling
   fidelity validation — but it is a research-grade phase (new oracle pipeline + the cross-mode
   model-form question that pulls in near-incompressibility (e) and possibly a richer model), not
   an M2 leaf. It deserves the full recon→spike→implement rigor.
3. **Surface the strategic fork for the user.** With M2 done, two credible directions compete —
   and roller BCs were deliberately the spine that keeps *both* open:
   - **(A) Go DEEPER on substrate fidelity → M3-modes.** Validate equibiaxial/planar (the
     canonical uniaxial-fit failure test) on the Hossain Ecoflex data; likely matures the
     constitutive model (near-incompressibility / I₂). Highest fidelity ceiling; keeps building
     the validated substrate before composing it.
   - **(B) Move UP to the Layer-2 keystone → differentiable soft↔rigid coupling**, which the
     M2 roller BCs now **unblock** (the per-axis interface constraint the coupling needs). This
     is the mission's named keystone and advances the stack toward the co-design loop.
   Both are legitimate; this is a mission-direction call for the user. (My lean: **A first** —
   the keystone should not inherit an unvalidated cross-mode constitutive error, the same
   "validate the substrate before you compose it" logic that put Layer-1 ahead of Layer-2 in the
   first place. But the user owns this fork.)

## Honest scope

S4 is a literature scan, not an acquired dataset. The conclusion — multi-mode silicone data
exists at a digitization-tier bar, best matched by the Hossain Ecoflex multi-Shore set — is
sufficient to decide *placement* (modes = a real M3 phase, not blocked), which is all S4 was
chartered to do. Acquiring/curating the oracle is the first sub-arc of M3-modes if (A) is chosen.

## Sources

- Liao, Hossain, Yao, *Ecoflex polymer of different Shore hardnesses: Experimental investigations
  and constitutive modelling*, Mechanics of Materials 144:103366 (2020). DOI
  `10.1016/j.mechmat.2020.103366`. OA: Swansea Cronfa `cronfa53571`.
- Liao, Hossain, Yao, Mehnert, Steinmann, *On the stress recovery behaviour of Ecoflex silicone
  rubbers* (2020/21); Glasgow eprint 197199.
- Meunier, Chagnon, Favier, Orgéas, Vacher, *Mechanical experimental characterisation and
  numerical modelling of an unfilled silicone rubber*, Polymer Testing 27(6):765–777 (2008).
- PDMS multi-elastic dataset, Mendeley Data `mwmzpgrzs3` (CC BY 4.0); Sci. Rep. 13 (2023)
  `s41598-023-45372-0`.
- Nunes, *Mechanical characterization of hyperelastic polydimethylsiloxane by simple shear test*,
  Mater. Sci. Eng. A 528:1799 (2011).
