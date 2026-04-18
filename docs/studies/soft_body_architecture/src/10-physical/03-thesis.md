# Why visual and physical are not separate goals — the thesis

The historical split between "games physics" and "science physics" is real, and for twenty years it was defensible. Games needed 60+ FPS on consumer hardware in the 2005–2015 era and adopted position-based dynamics, mass-spring networks, and penalty contact because nothing else could run in the frame budget. Science needed finite-strain hyperelasticity, implicit time integration, and non-penetration guarantees because the numbers came out as data and had to be right; so it adopted backward-Euler plus Newton on neo-Hookean energy, ran offline, and called the result valid when a convergence plot said so. The two fields built parallel toolchains — Bullet, PhysX-cloth, XPBD on one side; FEBio, PolyFEM, Abaqus on the other — with near-zero overlap in solver kernels and near-zero shared conferences.

This chapter argues that the split has dissolved, that the dissolution is a recent event — roughly 2020 onward — and that any soft-body stack built today should refuse to choose a side. The thesis has three parts. (1) The artifacts that make cheap solvers look wrong are physical errors, not cosmetic ones. (2) The machinery that makes scientific solvers correct is now fast enough to render at 60+ FPS on a single consumer GPU. (3) The integrated solver that follows is simpler than either parent, not more complex. The rest of the book is the consequences of taking those three together.

## The artifacts are physics, not cosmetics

The Ch 02 chapter next door enumerated six failure modes of existing soft-body solvers: volume loss under squeeze, wobble, popping, missing friction, poor rim deformation, and material-parameter poverty. Each one is presented in game-dev folklore as a *visual artifact* and in scientific papers as a *numerical artifact*, and the two framings describe the same thing from two sides.

**Volume loss under squeeze.** Linear FEM over-softens under large strain because the linearization is a first-order Taylor expansion around the undeformed configuration; XPBD and mass-spring networks have no volumetric constraint at all. The game-dev framing is *the gel looks like it is deflating.* The science framing is *the effective Poisson ratio has collapsed at operating strain.* The cure is the same cure — a hyperelastic constitutive law with a near-incompressibility penalty, written as energy density $\Psi(F)$ with a volumetric term that diverges as $J = \det F \to 0$ — and the visible and the measurable fix together.

**Wobble and unphysical jelly feel.** Under-damped explicit integrators on a spring network produce oscillations at the network's natural frequencies because no dissipative term bleeds them. The game-dev framing is *the jello keeps jiggling after I let go.* The science framing is *the constitutive model has no viscoelastic spectrum.* The cure is Prony-series viscoelasticity calibrated to DMA data, which makes the material both look right and obey the measured storage-and-loss modulus curve for silicone.

**Popping and snap-through.** Penalty contact stores energy in the overlap volume and releases it suddenly when bodies separate. The game-dev framing is *the character's hand clips through and then pops out.* The science framing is *the contact formulation is not energy-conservative and admits non-physical configuration transitions.* The cure is IPC — the incremental potential that represents contact as a smooth barrier with no overlap at all. The barrier $b(d)$ diverges as the gap $d \to 0^+$ and vanishes outside a tolerance $\hat d$, so the total potential energy is smooth everywhere the solver visits, and the Newton iteration stays well-posed.

**Missing friction.** Most real-time engines either ignore friction or model it with a Coulomb proxy that has no stick regime. The game-dev framing is *objects slide off the character's palm like it is greased.* The science framing is *the contact law is frictionless; the simulator is solving the wrong partial differential equation.* The cure is smoothed Coulomb friction integrated with IPC, which resolves the stick–slip transition as a continuous energy term rather than a conditional branch.

**Poor rim deformation.** Coarse tet meshes with linear shape functions cannot represent the high-curvature deformation at a contact edge — the element has no degrees of freedom to bend with. The game-dev framing is *the silicone glove looks polygonal where it wraps the finger.* The science framing is *the element order is too low for the local stress gradient.* The cure is adaptive refinement driven by contact proximity, or quadratic tetrahedra (Tet10) in the contact band — both of which change the mesh topology, the element stiffness, and the visual silhouette simultaneously.

**Material-parameter poverty.** Uniform single-stiffness soft bodies cannot match the layered, fiber-reinforced, Shore-graded structure of real silicone parts. The game-dev framing is *every soft object has the same plasticky feel.* The science framing is *the material model has one parameter and the real material has dozens.* The cure is multi-material meshes with spatial material fields — stiffness, fiber direction, viscous relaxation time all SDF-valued — which change both the mechanical response and the visual cue. A ribcage under skin should look like a ribcage under skin, not a uniform blob, and the reason it does is that the stiffness contrast propagates into the surface deformation on every frame.

Each of these six is a case where the solver's mechanical wrongness and the render's visual wrongness are the same wrongness measured two different ways. Fixing one fixes the other. The received "visual vs. physical" dichotomy has been fighting itself the whole time: a game engine optimizing for visual quality by adding post-hoc corrections — softbody thickness hacks, secondary-motion animation layers, normal-map noise on contact — was always reaching for the same fix the physics demanded, one level removed and without the benefit of the gradient.

## The 2015 cost argument has expired

The standard objection to *just use FEM for real-time* from 2010 to roughly 2020 was cost. Neo-Hookean plus IPC plus Newton on a 50k-tet scene running at 60 FPS means solving a sparse linear system of ~150k DOFs sixty times a second, with contact detection and barrier updates on top. On a single CPU core in 2015 this was not possible by a factor of ten; on an eight-core desktop it was not possible by a factor of two. Hence PBD, XPBD, and the games branch of the solver tree.

Three things changed between 2018 and 2024 that kill the cost argument.

**GPU sparse solvers for FEM matured.** [NVIDIA Warp](../00-context/02-sota/01-warp.md), [Genesis](../00-context/02-sota/02-genesis.md), and the 2020–2024 GPU-FEM research literature demonstrated that preconditioned conjugate gradient on a mesh-derived sparse structure runs at kernel-throughput-bound rates on modern GPUs. Warp's published neo-Hookean benchmarks reach tens-of-thousands-of-tets scenes at 60+ FPS on consumer GPUs, with the exact envelope depending on Newton inner-iteration count, warm-start quality, and element order — well inside the per-character budget of a game engine, comfortably inside the per-scene budget of an experience-mode preview for an engineering tool. [Part 11 Ch 03 Phase E](../110-crate/03-build-order.md#the-committed-order) scales these published numbers into `sim-soft`'s own target envelope and explicitly names its targets as derived-from-Warp rather than measured-in-`sim-soft`.

**IPC was made practical for real-time.** The original IPC paper (Li et al. 2020, discussed in [Part 4 Ch 01](../40-contact/01-ipc-internals.md)) was presented as an offline method; the 2022 follow-ups on adaptive barrier width, GPU-parallel continuous collision detection, and warm-started friction brought the per-step cost into the tens of milliseconds on a mid-tier GPU for scenes with hundreds of contact pairs. The [IPC Toolkit](../00-context/02-sota/03-ipc-toolkit.md) reference implementation is still Eigen-on-CPU, but the algorithmic components port cleanly to wgpu compute kernels; [Part 8](../80-gpu/00-wgpu-layout.md) walks through the port.

**Differentiability came for free.** Once the solver is written as an energy minimization with a smooth total potential $U(x; \theta) = \Psi_\text{elastic}(x) + \Psi_\text{contact}(x) + \Psi_\text{inertia}(x)$, the adjoint method and the implicit function theorem give $\partial x^*/\partial \theta$ — the gradient of the steady-state configuration with respect to any design parameter — at the cost of one additional linear solve per backward pass. This is the foundation of [DiffTaichi](../00-context/02-sota/04-difftaichi.md), Genesis, and the differentiable-physics branch of the ML community, and it is *only possible* if the forward solve is energy-based to begin with. Penalty contact, impulse-based methods, and PBD are not differentiable in any usable sense because their step functions are not smooth. Getting the physics right and getting the gradients are literally the same prerequisite.

The 2015 objection — *we cannot afford the physics* — has become the 2026 affordance: we can run the physics, and because we can run it, we get the rendering quality and the gradients as byproducts we were going to have to pay for separately anyway.

## The integrated solver is simpler

The usual worry when combining previously-separate concerns is that the integrated thing is more complicated than either parent. In this case the opposite holds.

A games-branch softbody pipeline typically contains: a PBD solver, a secondary-motion module that fakes viscoelasticity with layered animation curves, a contact-separation heuristic that hides penetration with offset biasing, a skinning layer that maps the low-resolution physics mesh to a high-resolution render mesh with per-vertex rigging, a normal-map generator that fakes micro-wrinkles at contact, and a pile of tuning knobs that gate each subsystem's output into the next. A science-branch solver typically contains: a hyperelastic FEM, an IPC contact solver, a Newton-on-total-potential-energy time integrator, and a post-hoc visualization layer that extracts a surface mesh from the tet mesh and applies a flat shader to it.

The integrated stack replaces both with a single pipeline:

```text
sim-soft (FEM on hyperelastic material + IPC contact + implicit integration)
   ↓  deformed mesh, per-vertex stress, per-tet temperature, contact pressure
sim-bevy (shader reading per-vertex attributes — SSS, anisotropic GGX, contact normals)
```

There is no skinning layer because the FEM mesh IS the render mesh (with subdivision, per [Part 9 Ch 03](../90-visual/03-subdivision.md), for resolution). There is no secondary-motion module because the viscoelastic material model IS the secondary motion. There is no contact-separation heuristic because the IPC barrier IS the separation. There is no normal-map generator because the contact-pressure field IS the normal-perturbation source for micro-wrinkle shading. Each visual layer reduces to a shader input already produced by the physics.

Concretely, a `sim-soft` scene description reduces to four pieces:

```rust
pub struct SoftScene {
    pub geometry: SdfField,          // from cf-design, tetrahedralized by sim-soft
    pub material: MaterialField,     // SDF-valued: stiffness, density, fiber direction, Prony terms
    pub boundary: Vec<Boundary>,     // displacements, tractions, pin constraints
    pub partners: Vec<ContactBody>,  // rigid probes from sim-mjcf, self-contact enabled
}
```

The visual configuration is a shader, not a second simulation. The number of tuning knobs across both concerns goes down compared to either parent, not up. The book's later chapters expand each of the four fields into the traits and types that back it, but the shape above is the whole story at the top level.

## What this commits the rest of the book to

The thesis has consequences for every downstream design choice. They are listed below as commitments the rest of the book is bound to honor.

- **[Part 2](../20-materials/00-trait-hierarchy.md) commits to hyperelastic constitutive laws as the default**, with linear and corotational included as baselines for comparison only. The material-poverty failure from Ch 02 and the visual flatness of linear elasticity are the same defect; [Part 2 Ch 04](../20-materials/04-hyperelastic.md) is where the fix lives.
- **[Part 4](../40-contact/00-why-ipc.md) commits to IPC as the contact solver**, not penalty, not impulse, not PBD collision projection. The popping failure from Ch 02 and the non-differentiability of non-energy-based contact are the same defect.
- **[Part 5](../50-time-integration/00-backward-euler.md) commits to implicit time integration on total potential energy**, with Newton plus line search as the inner loop. The wobble failure from Ch 02 and the lack of unconditional stability under explicit schemes are the same defect.
- **[Part 6](../60-differentiability/00-what-autograd-needs.md) commits to differentiability as a first-class property**, not a retrofit. The *gradient support* of XPBD or PBD is limited to finite differences and is not a substitute; the thesis says differentiability is a free byproduct of doing the physics right, so the book builds the byproduct.
- **[Part 9](../90-visual/05-sim-bevy.md) commits to reading the physics mesh as the render mesh**, with per-vertex attributes (stress, temperature, thickness, contact pressure) as shader inputs. There is no separate animation skeleton, no secondary-motion layer, and no normal-map bakery.

The later parts are allowed to struggle with these commitments — [Part 6 Ch 05](../60-differentiability/05-diff-meshing.md) names differentiable meshing as an open research problem, and [Part 4 Ch 05](../40-contact/05-real-time.md) names IPC's real-time regime as a moving target — but the commitments are not tradeoffs between visual and physical goals. They are one goal, stated twice. The rest of the book is the consequences of not choosing a side.
