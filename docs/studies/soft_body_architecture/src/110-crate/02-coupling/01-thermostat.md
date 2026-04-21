# With sim-thermostat

`sim-thermostat` owns the temperature field. It advances thermal diffusion on its own grid with its own native timestep, and it is the other per-step-coupled sibling `sim-soft` sees (alongside [`sim-core`](00-mjcf.md)). The physical coupling is material-layer ([Part 2 Ch 08](../../20-materials/08-thermal-coupling.md)) — temperature modulates stiffness and drives thermal expansion; viscoelastic loss and plastic rearrangement deposit heat back into the thermostat's field. This sub-leaf specifies the cross-crate architecture; [`coupling/` §05](../00-module-layout/05-coupling.md) implements the `sim-soft` side, [Part 2 Ch 08](../../20-materials/08-thermal-coupling.md) commits the constitutive-law side, and [Part 5 Ch 03 §01](../../50-time-integration/03-coupling/01-fixed-point.md) commits the time-integration side.

## What crosses the boundary

| Field | Direction | Representation | Required when |
|---|---|---|---|
| Temperature field at tet nodes / Gauss points | `sim-thermostat` → `sim-soft` | per-query scalar $T$ from a `Field<Output = f64>` view into `sim-thermostat`'s grid; sampled at the rate [`material/`](../00-module-layout/00-material.md)'s `Thermal<M>` decorator needs | thermal coupling enabled |
| Dissipative heating rate | `sim-soft` → `sim-thermostat` | per-tet heat-source scalar $\dot q = \sigma : \dot\varepsilon_\text{visc}$ (W/m³) integrated over the element volume and deposited on `sim-thermostat`'s grid | thermal coupling enabled, viscoelastic material present |
| Material-side temperature-dependent parameters | derived, not exchanged | `Thermal<M: Material>` decorator reads $T$ and emits temperature-adjusted $(\mu, \lambda)$; no additional interface call | always when decorator is live |

The coupling is bidirectional by default per [Part 2 Ch 08 Claim 2](../../20-materials/08-thermal-coupling.md) — writing dissipative heat back into the thermostat is required to keep extended-load analyses physically closed. One-way thermal (temperature in, no heat out) is opt-in only for cold-regime simulations where viscous dissipation is provably small.

## The `sim-soft` side — `coupling/` trait `ThermostatHandshake`

`sim-soft::coupling/` exports a `ThermostatHandshake` trait that the outer coupling loop calls at each handshake tick:

```rust
pub struct TemperatureView<'a> {
    pub field: &'a dyn Field<Output = f64>,   // read-only view into sim-thermostat's grid
    pub grid_clock: u64,                       // thermostat-side step counter, for cache invalidation
}

pub struct DissipationReport {
    pub per_tet_heating: Vec<f64>,             // W/m^3 per tet, integrated over Newton step
    pub time_window:     (f64, f64),           // start and end of the handshake interval
}

pub trait ThermostatHandshake {
    fn import_temperature(&mut self, view: TemperatureView);
    fn export_dissipation(&self) -> DissipationReport;
}
```

The temperature enters via a read-only `Field` view rather than a copy because the thermostat grid is typically finer than `sim-soft`'s tet mesh and copying at handshake rate would waste bandwidth. `sim-soft` samples the field lazily per-Gauss-point during the `Thermal<M>` decorator's call chain; the view's `grid_clock` is how [`material/`'s](../00-module-layout/00-material.md) per-handshake sample cache invalidates when the thermostat ticks. The `Field` trait itself is the [Part 2 Ch 09 spatial-fields](../../20-materials/09-spatial-fields.md) abstraction, shared with `MaterialField` so the machinery that reads stiffness per-element is the same machinery that reads temperature per-element.

Dissipation export is a per-tet scalar integrated over the Newton step's time window — not a point sample — because [Part 2 Ch 08 §02 dissipative heating](../../20-materials/08-thermal-coupling.md) commits to $\dot q = \sigma : \dot\varepsilon_\text{visc}$ (the Prony-series hysteresis rate), and the per-tet integrated form is what the thermostat's discretization consumes as a volumetric source. `DissipationReport` crosses the boundary once per handshake; individual Newton iterations do not write to the thermostat.

## Subcycling — `sim-thermostat` typically faster than `sim-soft`

Thermal diffusion on a fine spatial grid wants small $\Delta t$ for stability (the heat equation's CFL-like condition scales as $\Delta t \lesssim \Delta x^2 / (2\kappa)$ for explicit schemes, or is unconditionally stable but still accuracy-bounded for implicit ones). On the canonical problem's parameters, `sim-thermostat`'s native step is typically shorter than `sim-soft`'s 16 ms experience-mode step or 1 ms design-mode step. [`coupling/` §05 claim 3](../00-module-layout/05-coupling.md) owns the time-scale bridge:

- **Thermostat faster than `sim-soft`** (the common case): freeze the `sim-soft` state, tick `sim-thermostat` several times across the handshake interval, use the time-averaged temperature field as the boundary condition for the next `sim-soft` Newton solve.
- **Thermostat coarser than `sim-soft`** (the uncommon case — large-scale diffusion, coarse spatial grid): tick `sim-soft` several times on the same (held-fixed) temperature field, accumulate dissipative heating across the sub-steps, hand the integrated heat deposition to `sim-thermostat` as a single time-windowed source.

Neither `sim-soft` nor `sim-thermostat` encodes the other's clock rate. The bridge module is what owns the subcycling logic and the time-average / time-integrate bookkeeping, which is the module-layout commitment from [Ch 00 §05 claim 3](../00-module-layout/05-coupling.md) made concrete for the thermostat.

## The coupling is outside the Newton loop

Each `sim-soft` Newton solve runs against a frozen snapshot of `sim-thermostat`'s temperature field — the time-averaged field from the sub-cycled thermostat ticks, fixed for the duration of the Newton iteration. Temperature does not update inside the Newton loop; dissipative heating is accumulated across the converged step and exported once at handshake. This preserves the [`solver/` factor-on-tape contract](../00-module-layout/04-solver.md): the Hessian stays local to `sim-soft`'s DOFs, $\partial P / \partial T$ enters only through the `Thermal<M>` decorator's tangent contribution at a fixed $T$ per step, and the [IFT adjoint](../00-module-layout/07-autograd.md) continues to consume one back-substitution on the cached Cholesky factor. A design where temperature updated per Newton iteration would force the Hessian to carry cross-crate thermal-coupling terms — this is rejected on the same grounds [Part 5 Ch 03](../../50-time-integration/03-coupling.md) rejects monolithic rigid-deformable assembly for `sim-core`. Fixed-point iteration across handshakes, frozen boundary data inside Newton. One pattern, two siblings.

## The material-side seam — `Thermal<M>` decorator

The temperature-dependent stiffness and thermal-expansion behaviors live in [`material/`](../00-module-layout/00-material.md) via the `Thermal<M: Material>` decorator committed in [Part 2 Ch 00 §01 composition rules](../../20-materials/00-trait-hierarchy/01-composition.md) and implemented in [Part 2 Ch 08](../../20-materials/08-thermal-coupling.md). The decorator's outside-in composition order — `Thermal<Viscoelastic<Anisotropic<Iso>>>` — is fixed per [Ch 01 §01 composition rules](../01-traits/01-composition.md). `coupling/` does not add material machinery; it passes the temperature view through to `material/` and lets the `Thermal<M>` decorator do the temperature-parameter modulation at its canonical hot-path call site.

The practical consequence for this sub-leaf: the cross-crate surface is narrow (temperature view in, dissipation report out), and the constitutive complexity (temperature-dependent modulus, thermal expansion's multiplicative split, Prony hysteresis rate) lives inside `sim-soft::material/` and `sim-soft::element/`. No special handling in `coupling/`. No thermostat-specific branches in `solver/`. The thin-boundary discipline from [Ch 02 parent claim 1](../../110-crate/02-coupling.md) holds.

## Dissipation accounting and determinism-in-θ

Dissipative heating is a deterministic functional of the converged Newton trajectory over the handshake interval — $\dot q = \sigma : \dot\varepsilon_\text{visc}$ evaluated from the Prony-series internal variables per [Part 2 Ch 07 viscoelastic](../../20-materials/07-viscoelastic.md). No RNG, no wall-clock sensitivity, no per-call counter drift. The [`ForwardMap` γ-locked determinism-in-θ contract](../../100-optimization/00-forward.md) holds across coupled `sim-soft` ↔ `sim-thermostat` evaluations for the same reason it holds across coupled `sim-soft` ↔ `sim-core` evaluations: each simulator is deterministic given fixed inputs, the fixed-point iterate count is a deterministic function of initial partner state, and the subcycled thermostat ticks are deterministic reductions (time-averaging, time-integration) of a deterministic inner loop. The coupling does not introduce a stochastic component and would have to be explicitly renegotiated against [Part 10 Ch 00](../../100-optimization/00-forward.md) if it did.

## What the thermostat coupling does not carry

- **No temperature field storage on the `sim-soft` side.** [`coupling/` §05 "does-not-carry"](../00-module-layout/05-coupling.md) is explicit: the temperature field lives in `sim-thermostat`, not `sim-soft`. The bridge passes a view, `material/`'s `Thermal<M>` decorator reads it per-Gauss-point, and the data never gets copied into a `sim-soft`-side buffer.
- **No mechanical response to temperature inside `sim-thermostat`.** The thermostat diffuses heat; it does not solve for the deformable body's response to temperature. That is `sim-soft`'s job. The coupling is asymmetric in this direction: `sim-thermostat` does not see the mesh state, only the per-tet heat sources.
- **No constraint-like coupling.** Nothing looks like a Lagrange-multiplier enforcement of temperature continuity or equal-heat-flux constraints at the interface. The handshake is wrench-shaped (scalar heat source, scalar temperature) rather than constraint-shaped — same discipline as [`sim-core`](00-mjcf.md)'s wrench-only surface.

## What this sub-leaf commits the crate to

- **`sim-thermostat` coupling is bidirectional with temperature in and dissipative heating out.** One-way thermal (no heat out) is opt-in only for cold-regime simulations where viscous dissipation is provably small.
- **The handshake surface is `ThermostatHandshake` on the `sim-soft` side.** `TemperatureView` is read-only (no copy; `Field` trait view with a clock stamp); `DissipationReport` is per-tet scalar integrated over the handshake window.
- **Subcycling is `coupling/`'s responsibility.** Time-averaged temperature in the common faster-thermostat case; time-integrated heating in the rarer coarser-thermostat case. Neither crate encodes the other's rate.
- **Temperature is frozen inside the Newton loop.** Fixed-point iteration across handshakes; no cross-crate thermal-coupling term in `sim-soft`'s Hessian. [`solver/`'s factor-on-tape contract](../00-module-layout/04-solver.md) is preserved.
- **Material-side temperature modulation is `Thermal<M>`'s job.** `coupling/` is the bridge; the constitutive details live in [Part 2 Ch 08](../../20-materials/08-thermal-coupling.md) and [`material/`](../00-module-layout/00-material.md).
- **Determinism-in-θ is preserved across the coupling.** `ForwardMap`'s contract holds on coupled `sim-soft` ↔ `sim-thermostat` evaluations by the same construction as `sim-core`.
- **Phase F closes this coupling alongside `sim-core`.** Both per-step-coupled siblings come up together; [Phase F](../03-build-order.md#the-committed-order) is the first multi-crate milestone.
