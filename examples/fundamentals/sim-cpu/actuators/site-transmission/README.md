# Site Transmission — Configuration-Dependent Moment Arms

A site-based wrench actuator on a 2-DOF arm. Instead of acting on a joint
directly, the actuator applies a force at the tip in the tip's local frame.
The effective torque on each joint comes from the site Jacobian — and the
shoulder's moment arm depends on the arm's configuration.

## What you see

- **Two-link arm** (upper arm + forearm) floating in zero gravity, viewed at
  a slight angle to show both links clearly
- A **pink ball** marks the hand tip where the site force acts
- The **elbow oscillates** smoothly between 0 and 90 deg (10-second period)
- The **shoulder deflects** in response, but by different amounts — more when
  the elbow is straight (long lever), less when the elbow is bent (short lever)
- Watch the HUD: `moment arm -> shoulder (varies)` swings between -0.50 and
  -0.23, while `moment arm -> elbow (constant)` stays locked at -0.2000

The shoulder "breathes" in sync with the elbow — same force, different lever,
different torque. That configuration dependence is the whole point of
site-based actuation.

## Physics

A `<general site="tip_site" gear="1 0 0 0 0 0">` actuator applies a 6D wrench
at a body site. The `gear` vector encodes the wrench direction in the site's
local frame — here, a pure force in local X. The generalized force on each
joint is:

```
tau = J^T * R * gear * force
```

where J is the site Jacobian and R is the site's rotation matrix. Because J
depends on configuration, the same end-effector force produces different joint
torques at different arm poses.

For this 2-DOF arm (upper arm L1=0.3 m, forearm L2=0.2 m):

```
Elbow at 0 deg:   shoulder moment arm = -0.500   (full arm is the lever)
Elbow at 90 deg:  shoulder moment arm = -0.230   (shorter lever)
Elbow at any:     elbow moment arm    = -0.200   (always L2)
```

A passive spring (stiffness=10 Nm/rad) on the shoulder converts the varying
torque into visible deflection. The shoulder angle "breathes" between about
-29 deg (straight arm) and -13 deg (bent arm).

| Parameter | Value |
|-----------|-------|
| Site force | 10 N (gain=10, ctrl=1) |
| Upper arm length | 0.3 m |
| Forearm length | 0.2 m |
| Shoulder spring | 10 Nm/rad, damping 1.5 |
| Elbow servo | kp=20, critically damped |
| Oscillation period | 10 s |
| Integrator | RK4, dt = 1 ms |

**Key distinction from joint actuators:** a joint motor applies torque in joint
space — the torque is independent of configuration. A site actuator applies
force in Cartesian space — the resulting joint torques depend on the arm's
pose. This is the foundation of operational-space control in robotics.

## Expected console output

```
t=  1.0s  q_sh= -28.6 deg  q_el=  2.1 deg  arm_sh=-0.4998
t=  5.0s  q_sh= -13.3 deg  q_el= 84.3 deg  arm_sh=-0.2300
t= 10.0s  q_sh= -28.6 deg  q_el= -5.7 deg  arm_sh=-0.4985
t= 15.0s  q_sh= -13.2 deg  q_el= 84.2 deg  arm_sh=-0.2303
```

The shoulder angle and moment arm oscillate together — large deflection when
the elbow is straight (long lever), small deflection when bent (short lever).
The elbow moment arm stays at -0.2000 throughout.

## Validation

Four automated checks at t=17s:

| Check | Expected | Threshold |
|-------|----------|-----------|
| **Actuator length = 0** | Site transmission has no length | < 1e-12 |
| **Shoulder moment arm varies** | \|straight\| > 0.25 and > \|bent\| | structural |
| **Elbow moment arm constant** | Both samples near 0.200 | < 10% |
| **Config-dependent** | \|straight - bent\| > 0.05 | structural |

## Run

```
cargo run -p example-actuator-site-transmission --release
```

Orbit: left-drag | Pan: right-drag | Zoom: scroll
