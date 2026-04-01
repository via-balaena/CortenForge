# Stress Test — Headless Mocap Validation

Headless validation of all mocap body engine invariants. No window, no Bevy
 — pure sim-core assertions.

## Checks

| # | Name | What it verifies |
|---|------|-----------------|
| 1 | Position tracks mocap_pos | `xpos` matches `mocap_pos` exactly after `forward()`. |
| 2 | Quaternion tracks mocap_quat | `xquat` matches `mocap_quat` exactly after `forward()`. |
| 3 | World-child invariant | `body_parent[mocap_body] == 0`. |
| 4 | Contact generation | Mocap geom near dynamic geom produces `ncon > 0`. |
| 5 | Immune to gravity | 1000 steps under gravity — mocap position unchanged. |
| 6 | Immune to contact forces | Contact with dynamic body — mocap position unchanged. |
| 7 | Multiple mocap independent | Moving one mocap body does not affect another. |
| 8 | Keyframe restores mocap state | `reset_to_keyframe()` restores `mocap_pos` and `mocap_quat`. |
| 9 | Weld-to-mocap: force generated | Displacing mocap creates nonzero `qfrc_constraint` on dynamic body. |
| 10 | Weld-to-mocap: follower tracks | Dynamic body converges toward mocap target through soft weld. |
| 11 | Zero-mass FK override | Mocap body with `mass="0"` — FK still positions correctly. |
| 12 | Child follows mocap parent | Moving mocap parent shifts child body by the same amount. |

## Run

```
cargo run -p example-mocap-bodies-stress-test --release
```
