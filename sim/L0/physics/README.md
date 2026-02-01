<div align="center">
  <img src="assets/emo-toad.svg" width="120" alt="sim-physics mascot">
</div>

# sim-physics

Unified physics simulation API — re-exports `sim-types`, `sim-core`, `sim-constraint`, `sim-mjcf`, `sim-urdf`, `sim-muscle`, `sim-tendon`, and `sim-sensor`.

## Usage

Add to your `Cargo.toml`:

```toml
[dependencies]
sim-physics = { path = "sim/L0/physics" }
```

Then import everything through the unified API:

```rust
use sim_physics::prelude::*;
```
