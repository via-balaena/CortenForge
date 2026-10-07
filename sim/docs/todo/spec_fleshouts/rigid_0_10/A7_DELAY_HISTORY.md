> Appendix to `RIGID_SPEC.md`, written during planning by a read-only researcher (code at `3520544e`). Paths under `$SCRATCH` are the planning session's scratch and are **not in the repo**; every claim here is re-established by the tests its commit adds. Where this appendix and `RIGID_SPEC.md` disagree, `RIGID_SPEC.md` wins.

# Delay and history — actuator and sensor history buffers (P-L34 / ledger-L34)

Jon, 2026-10-06: implement MuJoCo 3.5.0's history buffers in Rigid. This section gives MuJoCo's algorithm with citations, golden measurements on `mujoco==3.5.0`, a prototype in a scratch copy of sim-core + sim-mjcf compared against them, the sim-mjcf rules, the commits, and the questions the fixed decisions do not settle.

Paths: `$DH` = `SCRATCH/rigid_spec/delay_history/`. MuJoCo citations are 3.5.0 (`SCRATCH/mj350src/mujoco`, tag 881544c) relative to `src/` unless they start with `doc/` or `include/`. Repo citations are at `3520544e` (the branch head moved to `139f7b25` during this work; that commit changes only `RIGID_SPEC.md`).

**Method, and what it can see.** I read MuJoCo's source; recorded 30 golden trajectories plus 5,720 API reads from MuJoCo 3.5.0 (`$DH/scripts/oracle*.py` → `$DH/golden*.json`); built a prototype (`$DH/prototype_core.diff`, `$DH/prototype_mjcf.diff`) and compared it with those trajectories (`$DH/probe`). I compared it with `main` on the in-tree MJCF corpus by fingerprinting 100 steps (`$DH/probe/src/bin/fp.rs`, 10 runs each), and ran the copied sim-core lib tests, 58 of the 67 integration-test modules, and clippy with the repo's lints. Section 9 lists what this does not cover.

---

## 1. MuJoCo 3.5.0's algorithm

### 1.1 Buffer layout and sizes

- One buffer per actuator with `nsample > 0` and per sensor with `nsample > 0`. Actuators come first, then sensors, both in index order (`user/user_model.cc:3760-3770`, `:3806-3819`). The address is −1 when `nsample <= 0`.
- Layout `[user, cursor, times(n), values(n·dim)]`. Size is `2 + 2n` for an actuator (dim 1) and `2 + n + n·dim` for a sensor (`user/user_model.cc:2252-2264`).
- `user` (slot 0) is the sensor's last compute tick in interval mode (`engine/engine_forward.c:862`). `cursor` (slot 1) is the physical index of the newest sample, stored as `mjtNum`. That is why `nsample <= 2^24` (`user/user_objects.cc:6969-6973`).
- Model storage: `actuator_history[2i] = nsample`, `[2i+1] = interp`, `actuator_delay`, `actuator_historyadr`, and for sensors the same plus `sensor_interval[2i] = period`, `[2i+1] = phase`.
- `history` is part of the physics state: `mjSTATE_HISTORY ⊂ mjSTATE_PHYSICS` (`include/mujoco/mjdata.h:32,46`).

**Ours matches the layout.** We have the same fields: `model.rs:605-626` and `:699-720`. Addresses come from sim-mjcf `builder/build.rs:515-556`, called at `:35`, with the same formula and order. Measured: for the golden models, `nhistory` and both address arrays equal MuJoCo's, and the initial actuator buffers are bit-identical.

### 1.2 Ring-buffer primitives (`engine/engine_util_misc.c:913-1124`)

- Indexing.
  - Logical index 0 is the oldest sample and n−1 the newest. `physical = (cursor + 1 + logical) % n` (`:917-919`).
  - `historyFindIndex` returns 0 if `t <= oldest`, n if `t > newest`, otherwise the smallest logical i with `times[i] >= t`, by binary search (`:925-955`).
- `mju_historyInsert(buf, n, dim, t)` (`:987-1031`) has four cases:
  - `|t − times[i]| < mjMINVAL`: overwrite that sample.
  - `i == 0` (older than the oldest): replace the oldest.
  - `i == n`: advance the cursor and write the new newest.
  - otherwise: shift logical `[1, i−1]` down one place, dropping the oldest, and insert at `i−1`.
  - `mjMINVAL = 1e-15` (`include/mujoco/mjtnum.h:26`).
- `mju_historyRead(buf, n, dim, res, t, interp)` (`:1036-1124`):
  - `t <= oldest + mjMINVAL` returns the oldest sample. `t >= newest − mjMINVAL` returns the newest. Both clamp; neither extrapolates.
  - An exact match returns that sample.
  - Otherwise: `interp == 0` returns the lower sample (ZOH); `interp == 1` interpolates linearly; **any other value** gives a cubic Hermite with Catmull-Rom slopes, set to 0 at the buffer ends (`:1081-1113`).

### 1.3 Initialisation: `_resetData` (`engine/engine_io.c:1266-1270, 1377-1427`) and keyframes

- `nhistory > 0 && timestep <= 0` → `mjERROR("history buffers require positive timestep")` (`:1266-1270`).
- Actuators: `user = 0`, `cursor = n−1`, times `−(n−j)·dt` (`[−n·dt … −dt]`), values 0.
- Sensors: `user = period > 0 ? (phase == 0 ? −period : phase) : −dt`, `cursor = n−1`.
  - With a period, the times are `ceil((t0 − (n−1−j)·period)/dt)·dt` with `t0 = phase == 0 ? −period : phase`.
  - Without a period, the times are as for actuators. Values are 0.
- `mj_resetDataKeyframe` runs `_resetData` and then copies `time` (`:1562-1576`). The buffers therefore keep timestamps relative to 0 whatever the keyframe's time.

### 1.4 Reads

- **Actuators** (`engine/engine_forward.c:297-305`). The local copy is `ctrl[i] = actuator_delay[i] ? mj_readCtrl(m,d,i,d->time,interp) : d->ctrl[i]`.
  - The test is *non-zero*, not `> 0`.
  - The copy is then clamped (`:307-310`) and checked for bad values (`:312-319`).
  - `mj_readCtrl` returns `d->ctrl[id]` when `nsample == 0`, otherwise it reads at `time − delay` (`engine/engine_support.c:845-866`).
  - Under RK4 the stages set `d->time` to the stage time (`engine/engine_forward.c:1097`) before `mj_forwardSkip` (`:1100`), so delayed actuators read at stage time − delay.
- **Sensors**: `compute_or_read_sensor` (`engine/engine_sensor.c:1346-1388`).
  - `nsample <= 0`: compute.
  - `delay > 0`: read the buffer at `time − delay`.
  - Else, `period > 0`: compute if `user + period <= time`, else read at `time`.
  - Else: compute.
  - It is called from the three stage loops after the sleep skip (`:1474/1493`, `:1527/1547`, `:1576/1601`).
  - **User and plugin sensors never go through it.** The loops handle them separately (`:1488`, `:1537`, `:1591`, then `compute_user_sensors`/`compute_plugin_sensors`).
- **Cutoff** is applied inside `mj_computeSensor` (`:1322-1343`). A value read from the buffer is **not** clamped again.

### 1.5 Insertion: `mj_advance` (`engine/engine_forward.c:833-884`)

The history block is the first thing in `mj_advance`. It comes before activations (`:886`), sleep (`:898`), velocity (`:907`), position (`:915`), time (`:921`), plugins (`:923`) and warmstart (`:938`).

- **Actuators:** insert `d->ctrl[i]` at `d->time` (`:839-847`). This is the `ctrl` as left by the step's control callback(s).
- **Sensors** (`:849-883`). Interval (`period > 0`): if `user + period <= time`, then `user += period` and insert at `time`. Otherwise every step inserts at `time`. The slot is filled with:
  - a fresh `mj_computeSensor(...)` (with cutoff) when `delay > 0`;
  - otherwise a copy of `sensordata`.
- **Callers of `mj_advance`:**
  - `mj_EulerSkip` (`:1008`) and `mj_implicitSkip` (`:1348`).
  - `mj_RungeKutta`, after the stages. Time and `qpos`/`qvel`/`act` are restored to the step start first (`:1114-1121`), but the derived quantities are the last stage's.
  - `mj_step2`, which always uses Euler or implicit (`:1505-1512`).
- `mj_forward` never inserts.

### 1.6 Interactions

| | MuJoCo 3.5.0 |
|---|---|
| `forward()` | reads only (no `mj_advance`) |
| `step1`/`step2` | step1 reads pos/vel sensors; step2 reads actuators and acc sensors, then `mj_Euler`/`mj_implicit` inserts (`:1492-1515`) |
| RK4 | stage reads at stage time; insertion once, after the stages, at the step's start time |
| control callback | fires before actuation in every `mj_forwardSkip`, RK4 stages included (`:1399-1401`). Insertion takes `ctrl` after the last callback |
| sleep | insertion has no sleep filter (`:838-883`); stage reads skip sleeping sensors. `mj_sleep`'s re-forward (`:899-904`) runs after insertion, at the same `time`, so it re-reads delayed vel/acc sensors with the new sample present |
| `DISABLE_SENSOR` | stage reads return early. `mj_advance` still inserts, computing delayed sensors (no flag check, `:849-883`) |
| reset / keyframe | §1.3 |
| FD | `mjd_stepFD`: `nhistory` → `mjERROR("delays are not supported")` (`engine/engine_derivative_fd.c:298-300`). `mjd_transitionFD` refuses RK4 (`:544-546`) and history (`:547-549`) |
| user/plugin sensor with `delay > 0` | `mj_advance` → `mj_computeSensor` → `mjERROR("invalid sensor type…")` (`engine/engine_sensor.c:739-740, 858-859, 1316-1317`). The MJCF schema prevents it (§1.7) |

### 1.7 Compiler and XML rules

- Reading.
  - `nsample` is read as an int (`xml/xml_native_reader.cc:2297`, `:4052`). `"1.5"` → "problem reading attribute 'nsample'" (measured).
  - `interp` is a keyword: `zoh|linear|cubic` (`:754-760`).
  - `delay` is one real; `"0.01 0.02"` → "too much data" (measured).
  - `interval` is up to 2 reals, `period phase` (`:4055`, `exact=false`); 3 values → "too much data" (measured).
- Schema.
  - `nsample interp delay` are allowed on every actuator element, in `<default>` too (`:185-207`, `:392-436`).
  - `nsample interp delay interval` are allowed on every sensor **except `user` and `plugin`** (`:448-499`; measured: `<user … nsample>` → "unrecognized attribute: 'nsample'").
- Actuator compile checks (`user/user_objects.cc:6964-6974`):
  - `delay > 0 && nsample <= 0` → "setting delay > 0 without a history buffer";
  - `nsample > 2^24` → "at most 2^24 samples in history buffer, got %d".
- Sensor compile checks (`:7305-7328`):
  - "negative interval in sensor";
  - "positive phase in sensor";
  - `period > 0 && phase <= −period` → "phase must be greater than -period in sensor";
  - the delay/nsample check;
  - "at most 2^24 samples in sensor history buffer, got %d".
- **No dyntype gating.** Measured: filter, integrator, filterexact and position actuators with `nsample`+`delay` load and act on the delayed control. This contradicts DT-108 (`sim/docs/todo/POST_V1_ROADMAP.md:183`).

**What MuJoCo loads without complaint (measured, `$DH/scripts/edges.py`):**
- Negative `delay` with `nsample > 0`: an actuator then acts on the *newest sample*, a one-step delay (forces 0,1,2,3,4 for ctrl 1..5); a sensor ignores it. The sensor docs say delay "cannot be negative" (`doc/modeling.rst:1129-1130`).
- `interval` with no `nsample`: the sensor is recomputed every step. The docs say interval "Requires a history buffer" (`doc/modeling.rst:1138`).
- A phase with period 0: the phase is ignored.
- `nsample` −1: no buffer, which the docs allow ("If greater than 0").
- `interp` with no `nsample`: has no effect.
- `delay > nsample·timestep`: the read clamps to the oldest sample. Measured: `nsample=2`, delay 3·dt acts as 2·dt. `doc/modeling.rst:1232-1236` warns about this ("non-causal extrapolation … will not lead to a runtime error").

---

## 2. Measurements on MuJoCo 3.5.0

- The golden cases (`$DH/scripts/oracle.py`) run for 20 steps each, with ctrl `sin(0.9k)+0.05k`.
  - The actuator model is a 1-dof slide with 12 motors: no history; history only; delay {1, 1.5, 2.5, 0.3, 3, 3.7}·dt; every interp; buffers of 2–6 samples.
  - The sensor model is a damped hinge with 14 sensors: jointpos, jointvel, framepos, framequat, actuatorfrc and accelerometer, in history-only, delayed, interval, interval+phase and interval+delay modes, plus cubic + cutoff.
- Drivers: `mj_step` under Euler, RK4 and implicitfast; `mj_forward` ×5; `mj_step1`+`mj_step2`; reset at step 7; keyframes at t = 0.5, −0.015, −0.02 and −0.0333. The negative times exercise the out-of-order, exact-match and older-than-oldest insert branches; I chose the timestamps for that and did not instrument the code.
- **API reads** (`$DH/scripts/oracle_api.py`): `mj_readCtrl` and `mj_readSensor` at 55 times × 4 interp values for every actuator and sensor, after 12 steps. Also `mj_initCtrlHistory` and `mj_initSensorHistory`.

**Three MuJoCo behaviours that are not what the call asks for (measured):**
- **RK4 records delayed sensors from the last stage's kinematics.** Delayed[k] is compared with undelayed[k−1] (`$DH/scripts/rk4_stale.py`).
  - Under Euler: 0 for jointpos, framepos, jointvel and accelerometer.
  - Under RK4: jointpos 0 and jointvel 0, but framepos **3.06e-3** and accelerometer **6.84e-2**.
  - The sample inserted at time t holds values that are not the sensor's value at t.
- **`mj_step1`+`mj_step2` reads stale accelerations.**
  - On the same trajectory (qpos Δ = 0), the *undelayed* accelerometer differs from `mj_step`'s by **5.9** (`$DH/scripts/stale_rnepost.py`).
  - `flg_rnepost` is cleared only in `mj_forwardSkip` (`engine/engine_forward.c:1407`), `_resetData` (`engine/engine_io.c:1341`) and inverse (`engine/engine_inverse.c:230`), never in `mj_step2`.
  - Ours clears it in every `forward_acc` (`forward/mod.rs:579`).
  - This is not a history issue. It does affect the golden data, so step1/step2 goldens exclude acc-stage sensors that need `rnePostConstraint`.
- **Negative delay.** See §1.7.

---

## 3. Items

### H-1 sim-core: delays act (the runtime)

**Now.**
- Nothing in a step reads or writes `Data.history`. `history` appears only in init and reset: `data.rs:1099-1116`, `:1268-1284`, `model_init.rs:512-531`.
- Those blocks initialise **actuator buffers only**; sensor buffers stay all-zero (A1 §4 noted this).
- Measured on `main` against the goldens:
  - actuator force off by up to **2.1**;
  - sensordata off by up to **4.6**;
  - initial sensor history off by **4.0**;
  - the golden test below therefore fails on `main`.
- The FD entry points `assert!` `nhistory == 0` (`derivatives/fd.rs:72-76`, `hybrid.rs:2361-2364`).

**Target.** MuJoCo 3.5.0 §1, with one deviation still open (Q1: RK4 sensor samples).

**Change (prototype, `$DH/prototype_core.diff`).**

1. **New crate-private module `sim/L0/core/src/history.rs`.** Every function is `pub fn` inside `pub(crate) mod history`, as clippy `redundant_pub_crate` requires; the repo already uses that pattern in `forward/check.rs`.
   ```rust
   fn insert(buf: &mut [f64], n: usize, dim: usize, t: f64) -> usize;        // mju_historyInsert → value-slot offset
   fn read(buf: &[f64], n: usize, dim: usize, res: &mut [f64], t: f64,
           interp: InterpolationType) -> Option<usize>;                        // mju_historyRead
   fn init(model: &Model, history: &mut [f64]);                               // _resetData, both loops
   fn read_ctrl(model: &Model, data: &Data, i: usize, time: f64) -> f64;      // mj_readCtrl, model interp
   fn sensor_reads_history(model: &Model, data: &Data, i: usize) -> bool;     // compute_or_read_sensor's choice
   fn read_sensor(model: &Model, data: &mut Data, i: usize);                  // delayed value → sensordata
   fn advance_ctrl(model: &Model, data: &mut Data, time: f64);                // mj_advance :839-847
   fn advance_sensors(model: &Model, data: &mut Data);                        // mj_advance :849-883
   ```
   - The arithmetic follows MuJoCo's expression order, e.g. `-((n-j) as f64)*dt` and `ceil(t/dt)*dt`.
   - No allocation. For a delayed sensor, the advance-time compute swaps the slot with `sensordata[adr..adr+dim]`, calls `sensor::compute_sensor`, and swaps back.
2. **Gate on `model.nhistory > 0`** in the actuation read, `sensor_reads_history` and the advance. This is MuJoCo's own guard (`engine/engine_forward.c:838`).
   - It makes "no history → no change" structural.
   - It keeps hand-built models that leave the history arrays empty working. Measured: without it, **46 sim-core lib tests panic** on an index out of bounds into the empty history arrays (the first at the delayed-read line in `actuation.rs`), because `fiber.rs`, `sensor_tests.rs` and `actuation.rs` tests, and `tests/integration/derivatives.rs`'s helpers, build models with no history fields.
3. **Init.**
   - `make_data` (`model_init.rs:512-531`), `Data::reset` (`data.rs:1099-1116`) and `reset_to_keyframe` (`:1268-1284`) call `history::init`. That adds sensor buffers.
   - After K4, `reset_to_keyframe` runs `reset`, so its own block goes.
4. **Actuation.** The delayed read is inside K10's `actuator_ctrl_input(model, data, i)` (A2 §5):
   `raw = if model.nhistory > 0 && model.actuator_delay[i] != 0.0 { history::read_ctrl(...) } else { data.ctrl[i] }`, then clamp.
   - `!= 0.0` is MuJoCo's test. sim-mjcf refuses negative delays at load (H-3), so the two tests differ only on code-built models.
   - The bad-ctrl check then sees the delayed value, as MuJoCo's `:312-319` does.
5. **Sensors.**
   - Each stage loop's per-sensor body is extracted into `compute_pos_sensor`, `compute_vel_sensor` and `compute_acc_sensor`. That is MuJoCo's `mj_computeSensorPos/Vel/Acc` shape. The move is a 4-space dedent plus `continue`→`return`: 19/27/31 changed lines ignoring whitespace, about 1,400 with it.
   - New `sensor::compute_sensor(model, data, i)` dispatches on stage and then applies `apply_sensor_cutoff`, as `mj_computeSensor` does.
   - In each loop, after the sleep skip: `if history::sensor_reads_history(..) { history::read_sensor(..); continue; } compute_X_sensor(..)`.
   - `mj_sensor_postprocess` skips sensors that read their buffer this pass, as MuJoCo does (cutoff only at compute). Mutation-checked: removing the skip makes the cubic+cutoff golden sensor differ from MuJoCo by **1.2e-3**; with it, 3.5e-18.
6. **Insertion.**
   - `Data::integrate` (`integrate/mod.rs:39`, the Euler/implicit path and `step2`): `advance_ctrl(time)` then `advance_sensors`, first thing, before activations. This is MuJoCo's place.
   - `rk4::mj_runge_kutta`: `advance_ctrl(t0)` after the stages and before the final update (MuJoCo's place). For the sensors, see Q1. The prototype implements **option C**: `advance_sensors` at RK4 entry, at the step-start state.
7. **User/plugin sensors** with a buffer and `delay > 0`, which only a code-built `Model` can have: their slot gets a copy of `sensordata`. MuJoCo `mjERROR`s here. This is a deviation listed under Q6.

**Public API.** H-1 adds none. Fields and the `Data.history` layout are unchanged and already MuJoCo's. Doc changes:
- `sensor_interval`'s "Phase is always initialized to 0.0 by the compiler" (`model.rs:624-625`) becomes false (H-3);
- `Data.history` (`data.rs:78-80`) gets the layout;
- `actuator_delay` gets "non-zero reads the buffer (MuJoCo); a negative value reads the newest sample";
- `sensor_delay` says user/plugin sensors are excluded.

**Tests to add** (sim-conformance-tests `integration/history.rs`).
- Golden data: `assets/golden/history/` holds the models plus MuJoCo **3.5.0** reference arrays. History did not exist in 3.4.0, so these are the one 3.5.0 golden set; the README says so. The generator `scripts/gen_history_reference.py` pins `mujoco==3.5.0`.
- `delayed_actuators_match_mujoco_3_5_0`: Euler, RK4, implicitfast; step, step1+2, forward; reset; keyframes at 0.5, −0.015, −0.02, −0.0333. Force and history to 1e-12. **Fails on main** (force Δ 2.1).
- `delayed_sensors_match_mujoco_3_5_0`.
  - Runs Euler step and keyframe; implicitfast without the accelerometer (§8 F1); step1+2 without acc-stage `rnePost` sensors (§2).
  - **Fails on main** (Δ 4.6; initial history Δ 4.0).
- `sensor_history_initial_state_matches_mujoco` (interval/phase rounding). **Fails on main** (Δ 4.0).
- `delayed_sensor_reads_the_value_one_step_earlier`: `nsample=2, delay=dt` against the undelayed sensor at k−1, `to_bits` equal, under Euler, RK4, implicitfast and implicit, for jointpos, framepos, jointvel and accelerometer.
  - Prototype: 0 differing values in 4×19 steps.
  - **Main: 18–37 differing values per integrator.**
  - Under option C this test is also what makes the RK4 behaviour a tested deviation.
- `interpolated_value_is_not_reclamped_by_cutoff` (cubic + cutoff golden). Made to fail by removing the postprocess skip (1.2e-3).

**Tests and examples that flip.** None in sim-core lib: 720 pass on the prototype copy, and it adds no tests. None in the integration modules run (§4). No example uses history (`git grep`).

**Downstream.**
- sim-mjcf builder: compiles unchanged for H-1.
- cf-design `mechanism/model_builder.rs:920-925` pushes `nsample 0` / `delay 0`: unaffected.
- sim-bevy `sensor_viz.rs:724-728` (test) fills the sensor history arrays with zeros: unaffected.
- sim-gpu has no actuation path (`pipeline/orchestrator.rs:289` clears `qfrc_actuator`): not affected.
- Code that moves state between `Data`s by copying `qpos`/`qvel` does not carry `history`, while MuJoCo's physics state includes it (`include/mujoco/mjdata.h:46`). For example sim-coupling `articulated.rs:141-142,194-195,431-432`. No in-tree model there has `nsample > 0` (not checked exhaustively).

### H-2 sim-core: the history API (Q3)

**Now.** No way to read a buffer except decoding `Data.history` by hand, so the "history-only" mode (`nsample > 0, delay = 0`, `doc/modeling.rst:1180-1184`) is unusable.

**Target.** MuJoCo's four `MJAPI` functions (`engine/engine_support.c:845-948`, `engine/engine_support.h:151-157`). Where MuJoCo `mjERROR`s (bad id, no buffer, times not strictly increasing), we return `Err`, per the parity rule.

**Change (prototype, in `history.rs`):**
```rust
#[derive(Debug, Clone, Copy, PartialEq, Eq)] #[non_exhaustive]
pub enum HistoryError {
    InvalidActuator { id: usize, nu: usize }, InvalidSensor { id: usize, nsensor: usize },
    NoBuffer, WrongLength { expected: usize, actual: usize }, TimesNotIncreasing { index: usize },
}
impl Data {
    pub fn read_ctrl(&self, model: &Model, id: usize, time: f64, interp: Option<InterpolationType>) -> Result<f64, HistoryError>;
    pub fn read_sensor(&self, model: &Model, id: usize, time: f64, interp: Option<InterpolationType>, out: &mut [f64]) -> Result<(), HistoryError>;
    pub fn init_ctrl_history(&mut self, model: &Model, id: usize, times: Option<&[f64]>, values: Option<&[f64]>) -> Result<(), HistoryError>;
    pub fn init_sensor_history(&mut self, model: &Model, id: usize, times: Option<&[f64]>, values: Option<&[f64]>, phase: f64) -> Result<(), HistoryError>;
}
```
- `sim_core::HistoryError` is re-exported from `lib.rs`.
- Also replace `impl From<i32> for InterpolationType` (`enums.rs:296-305`; it maps out-of-range values to ZOH, where MuJoCo's read treats them as cubic) with `TryFrom<i32>`. It has 0 in-tree callers (`git grep`).

**Measured.**
- Reads: 2,640 actuator reads match `mj_readCtrl` to **2.2e-16**; 3,080 sensor reads match `mj_readSensor` to **8.9e-16**, which equals the gap between the two buffers' contents.
- `init_ctrl_history`: the buffer is bit-identical to MuJoCo's, and so are the next 4 steps' forces.
- `init_sensor_history`: bit-identical.
- Errors: no buffer, equal times, and id 99 each give the matching variant.

**Tests.** The four reads/inits against `golden_api` arrays, plus one test per error variant. These are new API, so they do not compile on main.

### H-3 sim-mjcf: parsing and rules (RIGID_SPEC M25)

**Now.**
- Parsing: `parser/actuator.rs:111-113`, `parser/defaults.rs:282-284`, `parser/sensor.rs:126-129`.
- `interval` goes through `parse_float_attr`, so `"0.03 -0.01"` fails to parse and **silently becomes 0**. The phase is never read: `types.rs:3469-3471`, `builder/sensor.rs:130` pushes `(interval, 0.0)`.
- Checks present: delay/nsample (actuator `builder/actuator.rs:147-155`, sensor `builder/sensor.rs:91-99`) and negative interval (`:101-107`).
- Missing: the 2^24 caps, positive phase, and `phase ≤ −period`.

**Target and change.**

| rule | kind | citation / reason |
|---|---|---|
| `interval` = 1–2 reals `period [phase]` | parity | `xml/xml_native_reader.cc:4055`. 3 values → "too much data" via M6's reader |
| `nsample > 2^24` refused (actuator, sensor) | parity | `user/user_objects.cc:6969-6973`, `:7325-7328` |
| `phase > 0` refused; `period > 0 && phase <= −period` refused | parity | `user/user_objects.cc:7310-7318` |
| `nsample`/`interp`/`delay`/`interval` on `<user>`/`<plugin>` sensors | parity, schema (M15) | `xml/xml_native_reader.cc:496-499` |
| negative `delay` refused (actuator, sensor) | **stricter** | an actuator with a buffer reads the newest sample (one-step delay, measured); sensor docs say "cannot be negative" (`doc/modeling.rst:1129-1130`) |
| `period > 0` without `nsample > 0` refused | **stricter** | MuJoCo recomputes every step (measured); docs: "Requires a history buffer" (`doc/modeling.rst:1138`) |
| `phase != 0` with `period == 0` refused | **stricter** | phase ignored (`engine/engine_io.c:1404-1413`); documented range `(−period, 0]` is empty (`doc/modeling.rst:1157`) |
| `delay > nsample·timestep` (actuators, non-interval sensors) | **open, Q2** | `doc/modeling.rst:1232-1236` |
| RK4 with a delayed sensor | **open, Q1** | §2 |

Shape:
- `MjcfSensor.interval: Option<f64>` → `Option<(f64, f64)>`.
- `MjcfSensor::with_interval(self, period: f64)` → `with_interval(self, period: f64, phase: f64)`. There are no callers outside `types.rs` (`git grep`).
- The checks stay in the builder on resolved values, where today's are (`builder/actuator.rs`, `builder/sensor.rs`), and return M1's `MjcfError` variants with MuJoCo's message substrings (A3 §1 keeps "setting delay > 0 without a history buffer", "negative interval in sensor", "invalid interp keyword").
- Prototype: `$DH/prototype_mjcf.diff` (Q2 is not included).

**Tests to add.**
- One per rule, each failing on main because main loads the doc: positive phase; `phase = −period`; nsample 16777217 for actuator and sensor (the refusal comes before `make_data`, so nothing large is allocated); negative delay; interval without nsample; phase without period; `interval="0.03 -0.01"` stored as `(0.03, −0.01)`.
- Main stores `(0.0, 0.0)` for the last one, because the parse fails silently.

**Tests that flip** (sim-conformance-tests `integration`, which CI runs: `quality-gate.yml:721`). Measured on the copy: main 1185 pass, prototype 1182 + 3 fail.
- `sensor_phase6_spec_d.rs:39` `spec_d_t02_parse_interval`: `interval="0.5"` with no nsample. Add `nsample="1"`.
- `:413` `spec_d_t12_negative_delay_accepted`: becomes the negative-delay refusal test.
- `:436` `spec_d_t13_interval_without_nsample`: becomes a refusal test.
- If Q2 is adopted, three more (from reading `delay` against `nsample·0.002`; not run): `actuator_phase5.rs:818` t5 (class nsample 8 / delay 0.02), `:1128` t15 (pins "delay exceeding buffer accepted"), and `sensor_phase6_spec_d.rs:14` t01 (nsample 5 / delay 0.02).

**Corpus** (33 docs carry the attributes, all in those two test files, `$DH/scripts/corpus_hist.py`).
- 23 are MuJoCo-3.5.0-ok. With the stricter rules, 3 of those are refused: `ad46c289a0afd287`, `f7fdbcd7a023d6ac` (interval without nsample) and `16a6bb4899559dfe` (negative delay).
- Q2 would add `560ba7043018cbec`, `24ae2bdf505a738b` and `dc2267f6c52bba1f`.
- 5 docs that MuJoCo refuses for framepos `site=` are A4's allowlist question.

**Downstream.** None outside sim-mjcf constructs `MjcfSensor.interval` (`git grep`).

### H-4 Docs

- `sim/docs/MUJOCO_CONFORMANCE.md` divergences table:
  - three stricter rows (negative delay, interval without nsample, phase without period);
  - Q1's row, either way;
  - Q2's row if adopted;
  - user/plugin delayed sensors (Q6);
  - make_data with `timestep ≤ 0` (Q7).
- `POST_V1_ROADMAP.md:182` (DT-107): done. `:183` (DT-108): drop. MuJoCo 3.5.0 has no dyntype gating (§1.7, measured).
- sim-core docs: a "Delays" section (§1 compressed), the field docs from H-1, and `Data::integrate`'s doc ("inserts history samples, as `mj_Euler`").

---

## 4. Prototype results

All numbers are from `$DH/golden_cmp_final.txt` and the files it names. Error is max |ours − MuJoCo 3.5.0| over all steps.

| case | force | history | sensordata | qpos / qvel |
|---|---|---|---|---|
| actuators: Euler, RK4, implicitfast; step, step1+2, forward; reset; 4 keyframes | ≤ 2.2e-16 | **0 (bit-identical)** | — | ≤ 1.1e-19 / ≤ 1.7e-18 |
| sensors, Euler step / keyframes | — | ≤ 1.8e-15 | ≤ 1.8e-15 | ≤ 2.8e-17 / ≤ 2.2e-16 |
| sensors, forward only | — | 0 | 0 | 0 |
| sensors, RK4 | — | 1.8e-1 | framepos 2.9e-3, framequat 5.1e-3, accelerometer 1.7e-1; others ≤ 1.1e-16 | ≤ 3.5e-18 |
| sensors, implicitfast | — | 1.8e-1 | accelerometer 1.7e-1; others ≤ 1.1e-16 | ≤ 3.5e-18 |

How to read the table:
- The sensor floor (≤ 1.8e-15) is the existing sensor-level gap: the *undelayed* accelerometer under Euler differs by 8.9e-16 on both main and the prototype.
- The RK4 rows are option C (Q1). The bitwise test (H-1) shows they are delayed[k] = undelayed[k−1].
- The implicitfast accelerometer gap is not caused by this change: the undelayed accelerometer differs by 1.8e-1 **on main** (§8 F1).

Other checks:
- **Bit-identity of models without history.**
  - The 1,478 corpus docs our loader accepts, 100 steps, fingerprinting the bits of qpos, qvel, act, actuator_force, sensordata, history and time. 10 runs on main and 10 on the prototype, repeated 5× after the final edit.
  - **1,445 are deterministic. Of those, all 1,421 with `nhistory = 0` are bit-identical**, except the 2 docs the new interval rule refuses. All 22 with `nhistory > 0` changed or are refused.
  - 33 docs vary run-to-run on main itself (P-L27); 17 give 8–10 distinct fingerprints in 10 runs. This method cannot see them.
  - A false lead: an RK4 mesh doc looked changed in 2-run samples, and 12 main-only runs showed it bimodal (8/4).
- **sim-core lib tests (copy): 720 passed, 0 failed.**
- **Integration tests** (58 of 67 modules; 9 excluded because they need sim-urdf, sim-thermostat, sim-types or file assets): 1182 passed, 3 failed (H-3's flips). Main: 1185 passed.
- **clippy** `--all-targets --all-features -D warnings` with the repo's `[workspace.lints]` and `clippy.toml` on the sim-core copy: clean.

---

## 5. Open questions

**Q1 — RK4 with a delayed sensor** (settled for actuators; open for sensors).
- MuJoCo inserts the sample after the stages from the last stage's kinematics. Measured off by up to 6.8e-2 from the sensor's value at the sample's time; jointpos and jointvel are exact.
- Options:
  - (C) Sample at the step-start state. Implemented. The test proves delayed[k] = undelayed[k−1] bitwise under all four integrators.
  - (R) Refuse RK4 + `sensor_delay > 0`, at load and at step (a new `StepError` variant).
  - (P) Reproduce MuJoCo. This needs its stage-stale derived quantities *and* its `flg_rnepost` not being cleared in RK4 stages (`engine/engine_forward.c:1407`, where ours clears it at `forward/mod.rs:579`).
- **Recommend C**, as a listed deviation backed by that test. This follows Jon's "parity doesn't mean we have to inherit its bugs".
- The fixed rule read literally says R, and the "load where MuJoCo fails" decision covers load failures only, so this is Jon's call.
- What would differ if C is wrong:
  - under R, RK4 models with delayed sensors stop loading (0 in-tree);
  - under P, framepos, framequat, accelerometer and state-dependent actuatorfrc samples follow MuJoCo's stale values;
  - either way, the 6 lines at RK4 entry change.

**Q2 — `delay > nsample·timestep`.**
- MuJoCo loads it and silently clamps to the oldest sample; its docs warn about this.
- Options: refuse at load (actuators and sensors without interval; tolerance `delay > nsample·timestep·(1+1e-12)`, my choice, no MuJoCo referent), or parity.
- **Recommend refuse.** The file asks for a delay the buffer cannot hold. Interval sensors are not checked, since MuJoCo documents no bound for them.
- If wrong: three parse-only tests keep their numbers, and a too-long delay acts shorter than written.
- Runtime edits of `Model.actuator_delay` are not re-checked under either choice.

**Q3 — the four API functions (H-2) in Rigid.**
- **Recommend yes.** Without them, history-only mode has no reader.
- If wrong: 4 methods and `HistoryError` are frozen in 0.10.

**Q4 — `InterpolationType: From<i32>` → `TryFrom<i32>`.** **Recommend yes** (0 callers). If wrong: one breaking change with no user.

**Q5 — move `compute_history_addresses` (`builder/build.rs:515-556`) into sim-core's `recompute_derived` (K6).**
- **Recommend yes.** This is A1 §7 (b)'s reasoning: it reads only `Model` fields.
- If not: A1's doc table keeps "sim-mjcf-private", and a code-built model with `nsample > 0` must set its own addresses and `nhistory`.

**Q6 — user/plugin sensors with `delay > 0` in a code-built `Model`.** MuJoCo `mjERROR`s; ours copies `sensordata` into the slot.
- **Recommend document it as a limitation.** MJCF cannot express it (schema).
- The alternative is a refusal in K1's pre-step check, at O(nsensor) per step.

**Q7 — `make_data`/`reset` with `nhistory > 0` and `timestep ≤ 0`.** MuJoCo refuses at reset (`engine/engine_io.c:1266-1270`).
- **Recommend no new error.** K1 refuses every stepping and forward entry point at `timestep ≤ 0` (A1 §1), and sim-mjcf refuses such timesteps at load.
- If wrong: a `Data` with meaningless timestamps can exist until `reset` is called after the timestep is fixed.

---

## 6. Commit list and dependencies

1. **DH-1** `feat(sim-core): actuator and sensor delays act, as MuJoCo 3.5.0` (H-1, plus Q4).
   - Contents: `history.rs`, the init in make_data/reset, the actuation read, the sensor extraction and gate, the postprocess skip, the insertion in `integrate` and RK4, field docs, the golden assets and the 5 tests.
   - Depends on:
     - K1 (the shape check covers `history`/`nhistory`, so indexing cannot go out of range);
     - K4 (`reset_to_keyframe` → `reset`);
     - K7 (cb_control in RK4 stages; insertion after the last callback);
     - K10 (`actuator_ctrl_input` is where the read goes);
     - K5 and the sleep-timing fix, if they introduce a shared `advance` (see §7).
   - Place it after K10 in the core series. If Q5 is yes and K6 lands first, move `compute_history_addresses` here.
2. **DH-2** `feat(sim-core): read and initialise history buffers` (H-2, Q3). Depends on DH-1.
3. **DH-3 = M25** `feat(sim-mjcf): delay and history as MuJoCo 3.5.0` (H-3). Carries the 3 flips (6 with Q2) and the phase golden case.
   - Depends on M1 (error variants), M6 (attribute reader: "too much data", integer `nsample`), M11 (resolved defaults), M15 (schema for user/plugin).
4. Docs in K14 and M26 (H-4).

DH-1's golden tests load MJCF through today's sim-mjcf, which already parses `nsample`/`interp`/`delay`/single-value `interval`. The two-value interval case moves to DH-3, or sets `model.sensor_interval[i].1` directly. K8 (FD returns `Err` on history) is independent: FD refuses before any insertion.

---

## 7. Where insertion sits relative to the pre-step check (A1 §1) and callbacks (A2)

`step` order after the series:

1. K1 `check_step_inputs`: timestep, then the 24 lengths including `history`/`nhistory`.
2. `check_pos` / `check_vel`.
3. `forward`:
   - passive, then control callback (K7);
   - actuation **reads** the delayed ctrl at `time − delay`;
   - sensors **read** at `time − delay`.
4. `check_acc`.
5. Integrator:
   - **Euler/implicit**: `integrate` → **insert ctrl and sensors** → activations → … → time.
   - **RK4** (option C): **insert sensors** → stages, each firing the control callback and reading ctrl at stage time → **insert ctrl at t0** → final update.

`step2` has the same order through `integrate`. `forward`/`forward_skip` only read; FD refuses before any of this.

- **Sleep (Jon: fix sleep timing in Rigid).** MuJoCo inserts *before* `mj_sleep` and its re-forward (`engine/engine_forward.c:838` before `:898-904`).
  - If that fix moves `mj_sleep` into a shared advance, the ctrl and sensor insertion must stay first in it.
  - The re-forward then re-reads delayed vel/acc sensors with the new sample present, as MuJoCo does. That only changes values for `delay < dt` with linear or cubic interp (from §1.2's read rule; not measured).
- **Plugins (K5).** If K5 makes RK4 call a shared `mj_advance`-like helper, ctrl insertion belongs at its top, and the RK4 sensor insertion stays at entry under option C.

---

## 8. Found outside this area (measured; not fixed by DH-*)

- **F1 — implicit/implicitfast accelerometer.**
  - Under `implicitfast` and `implicit`, the *undelayed* accelerometer differs from MuJoCo 3.5.0 by **1.8e-1** on a damped hinge, on **main** (`$DH/golden_acc.json`, run against the repo crates). Euler: 8.9e-16.
  - MuJoCo keeps the explicit `qacc` in `d->qacc` (the implicit one is a stack local, `engine/engine_forward.c:1134,1348`). Ours runs `mj_fwd_acceleration` for these integrators inside `forward_acc` (`forward/mod.rs:565-571`), which writes `data.qacc` before the acc sensors.
  - That the second of these is the cause is my reading of the code, not isolated by an experiment.
- **F2 — `step2` under an RK4 model ignores eulerdamp.**
  - MuJoCo's `mj_step2` calls `mj_Euler` (`engine/engine_forward.c:1505-1512`), which damps implicitly. Ours, `integrate/mod.rs:185-197`, adds `qacc·h`.
  - Measured: qvel off by **2.6e-2** after 20 steps with damping 0.05, and **2.8e-17** with `<flag eulerdamp="disable"/>`. That isolates it, on main and on the prototype.
- **F3 — MuJoCo's `mj_step1`+`mj_step2` accelerometer is stale** (§2, Δ 5.9 vs `mj_step`). Ours does not have this defect. A2 may want to list it as a lenient divergence with a step-vs-step1/2 test.
- **F4 — serde_json without `float_roundtrip` misparses** `0.09999999999999999` as `0.1` (measured as time mismatches at steps 10, 11, 14). Any JSON golden harness needs the feature.

---

## 9. What I could not see

- Not run:
  - the prototype in the repo's own workspace, so no `grade`, doc-theft, licensed gates, or the `mujoco_conformance` target;
  - the 9 excluded integration modules (`keyframes.rs`, `runtime_flags.rs`, `batch_sim.rs`, `golden_flags.rs`, …);
  - sim-mjcf lib tests;
  - downstream crates.
- Not measured:
  - sleep with history (our sleep timing differs today);
  - `DISABLE_SENSOR` with delayed sensors;
  - `mjd_inverse_fd` sensor derivatives with history (MuJoCo's `mjd_inverseFD` does not refuse history);
  - plugin and user sensors with history;
  - muscle actuators with delay (MuJoCo's lengthrange failed on my model);
  - per-commit greenness in the K-order, because the prototype sits on main, not on K1/K4/K7/K10.
- The extracted sensor bodies were not re-reviewed beyond `diff -w`; bit-identity rests on the corpus fingerprint and the 720 lib tests.
- The 33 corpus docs that vary run-to-run, and every runtime-generated MJCF.
- Q1's option P was not prototyped.

Repo: HEAD `ea11c8d6` at start, `139f7b25` at end (spec-only commit by the coordinator); `git status --short` empty both times. I wrote only under `SCRATCH/rigid_spec/delay_history/` and this file.
