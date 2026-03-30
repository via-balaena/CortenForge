# Stress Test — Headless Validation

Headless (no Bevy) validation of the full muscle actuator system. Tests
both MuJoCo `<muscle>` (conformant) and CortenForge HillMuscle (extension)
across activation dynamics, curve pinning, pipeline force computation, and
dynamic behavior. Exits with code 1 if any check fails.

## Checks

| # | Group | Check | Tolerance |
|---|-------|-------|-----------|
| 1 | A: Activation | Asymmetry — rise faster than fall | qualitative |
| 2 | A: Activation | Hard-switch exact (ctrl=0.8, act=0.3) | 1e-10 |
| 3 | A: Activation | Hard-switch deactivation exact | 1e-10 |
| 4 | A: Activation | Boundary act=0 | 1e-10 |
| 5 | A: Activation | Boundary act=1 | 1e-10 |
| 6 | A: Activation | Ctrl clamping (1.5 == 1.0) | 1e-15 |
| 7 | A: Activation | Smooth blend (tausmooth=0.2) | 1e-3 |
| 8–12 | B: MuJoCo FL | 5 pinned values (0.5, 0.75, 1.0, 1.3, 1.6) | 1e-10 |
| 13–14 | B: MuJoCo FL | Outside range (0.3, 1.8) = 0 | exact |
| 15–18 | B: MuJoCo FV | 4 pinned values (-1.0, -0.5, 0.0, 0.4) | 1e-10 |
| 19–22 | B: MuJoCo FP | 4 pinned values (0.8, 1.0, 1.3, 1.5) | 1e-10 |
| 23–28 | C: Hill FL | 6 pinned values (Gaussian curve) | 1e-10 |
| 29–33 | C: Hill FV | 5 pinned values (Hill hyperbola) | 1e-10 |
| 34–37 | C: Hill FP | 4 pinned values (exponential) | 1e-10 |
| 38 | D: MuJoCo pipeline | Isometric force negative, F0 positive | qualitative |
| 39 | D: MuJoCo pipeline | Passive force at stretch | < 0 |
| 40 | E: Hill pipeline | Isometric force ≈ -F0 (penn=0) | 1% |
| 41 | E: Hill pipeline | Pennation reduces force (ratio ≈ cos 20°) | 2% |
| 42 | E: Hill pipeline | Passive force at stretch | < 0 |
| 43–44 | E: Hill pipeline | F0 auto-computation (scale/acc0) | 1e-6 |
| 45 | E: Hill pipeline | Dynamics identical to Muscle | 1e-15 |
| 46 | F: Dynamic | Activation rise time in 5–20ms range | range check |
| 47–48 | F: Dynamic | Forearm flexion — arm moved, force produced | qualitative |
| 49–51 | F: Dynamic | Cocontraction — equilibrium, both active | qualitative |

## Run

```
cargo run -p example-muscle-stress-test --release
```

Exit code 0 = all pass, exit code 1 = failure.
