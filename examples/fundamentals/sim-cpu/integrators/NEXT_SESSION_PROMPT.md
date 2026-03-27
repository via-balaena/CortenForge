We're on branch `feature/integrator-examples` with 4 commits ahead of main. Last session we built the integrator examples suite (7 examples, all passing) and added `Data.energy_initial` to sim-core.

There's a cleanup spec at `examples/fundamentals/sim-cpu/MJCF_ENERGY_FLAG_CLEANUP_SPEC.md` — 14 examples manually set `model.enableflags |= ENABLE_ENERGY` when the MJCF `<flag energy="enable"/>` already handles it automatically. The fix is to add the flag to each example's MJCF and remove the manual code.

**Task:** Stress test the MJCF energy flag cleanup spec before implementing. Specifically verify:

1. For each of the 14 examples, confirm they DON'T have `<flag energy="enable"/>` in their MJCF (some might already have it — the spec assumed none do)
2. Check if any of the 14 examples use `ENABLE_ENERGY` for anything OTHER than energy tracking (e.g., passed to a helper, stored in a variable, checked conditionally)
3. Verify that `load_model()` with `<flag energy="enable"/>` actually sets the bit — run the integrator comparison as proof (it works without manual flag)
4. Check if removing `sim_core::ENABLE_ENERGY` import would break anything else in each file (some might import other things from sim_core on the same line)
5. Look for any example that uses `let mut model` for reasons BESIDES the enableflags mutation — those would need to stay `mut`

After stress testing, implement the cleanup (one example at a time, verify each), then move on to drafting the **solvers** example spec (next Track 1 item after integrators — see `examples/COVERAGE_SPEC.md` item 6).
