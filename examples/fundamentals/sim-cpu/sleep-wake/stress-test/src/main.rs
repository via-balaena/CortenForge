//! Stress test — headless validation of the sleep/wake subsystem.
//!
//! 18 checks covering: sleep threshold, velocity zeroing, countdown mechanics,
//! wake triggers (contact, equality, applied force, cascade), island discovery
//! (singletons, shared, count, selective wake), countdown reset, policy flags,
//! narrowphase skip, and bookkeeping consistency.
//!
//! Run: `cargo run -p example-sleep-wake-stress-test --release`

#![allow(
    clippy::doc_markdown,
    clippy::expect_used,
    clippy::cast_possible_truncation,
    clippy::cast_precision_loss,
    clippy::similar_names,
    clippy::too_many_lines,
    clippy::needless_range_loop,
    clippy::cast_sign_loss,
    clippy::cast_lossless
)]

use sim_core::validation::{Check, print_report};
use sim_core::{ENABLE_SLEEP, MIN_AWAKE, SleepState};

// ── MJCF Models ───────────────────────────────────────────────────────────

/// Single free body on a plane with generous sleep tolerance.
const MJCF_SINGLE: &str = r#"
<mujoco model="single_box">
  <option gravity="0 0 -9.81" timestep="0.002" sleep_tolerance="0.1">
    <flag sleep="enable"/>
  </option>
  <worldbody>
    <geom type="plane" size="5 5 0.1" solref="0.005 1.5"/>
    <body name="box" pos="0 0 0.3">
      <freejoint name="box_free"/>
      <geom type="box" size="0.1 0.1 0.1" mass="1.0" solref="0.005 1.5"/>
    </body>
  </worldbody>
</mujoco>
"#;

/// Two isolated bodies on a plane (no mutual contact).
const MJCF_TWO_ISOLATED: &str = r#"
<mujoco model="two_isolated">
  <option gravity="0 0 -9.81" timestep="0.002" sleep_tolerance="0.1">
    <flag sleep="enable"/>
  </option>
  <worldbody>
    <geom type="plane" size="5 5 0.1" solref="0.005 1.5"/>
    <body name="A" pos="-2 0 0.3">
      <freejoint/>
      <geom type="box" size="0.1 0.1 0.1" mass="1.0" solref="0.005 1.5"/>
    </body>
    <body name="B" pos="2 0 0.3">
      <freejoint/>
      <geom type="box" size="0.1 0.1 0.1" mass="1.0" solref="0.005 1.5"/>
    </body>
  </worldbody>
</mujoco>
"#;

/// Two stacked boxes (shared island via contact).
/// Tight sleep_tolerance so bodies stay awake long enough for island check.
const MJCF_STACKED: &str = r#"
<mujoco model="stacked">
  <option gravity="0 0 -9.81" timestep="0.002" sleep_tolerance="1e-6">
    <flag sleep="enable"/>
  </option>
  <worldbody>
    <geom type="plane" size="5 5 0.1" solref="0.005 1.5"/>
    <body name="bottom" pos="0 0 0.5">
      <freejoint/>
      <geom type="box" size="0.1 0.1 0.1" mass="1.0" solref="0.005 1.5"/>
    </body>
    <body name="top" pos="0 0 0.8">
      <freejoint/>
      <geom type="box" size="0.1 0.1 0.1" mass="1.0" solref="0.005 1.5"/>
    </body>
  </worldbody>
</mujoco>
"#;

/// Two stacks of 2 boxes each, separated (for island count check).
/// Tight sleep_tolerance so bodies stay awake long enough for island check.
const MJCF_TWO_STACKS: &str = r#"
<mujoco model="two_stacks">
  <option gravity="0 0 -9.81" timestep="0.002" sleep_tolerance="1e-6">
    <flag sleep="enable"/>
  </option>
  <worldbody>
    <geom type="plane" size="10 10 0.1" solref="0.005 1.5"/>
    <body name="a1" pos="-2 0 0.5">
      <freejoint/>
      <geom type="box" size="0.1 0.1 0.1" mass="1.0" solref="0.005 1.5"/>
    </body>
    <body name="a2" pos="-2 0 0.8">
      <freejoint/>
      <geom type="box" size="0.1 0.1 0.1" mass="1.0" solref="0.005 1.5"/>
    </body>
    <body name="b1" pos="2 0 0.5">
      <freejoint/>
      <geom type="box" size="0.1 0.1 0.1" mass="1.0" solref="0.005 1.5"/>
    </body>
    <body name="b2" pos="2 0 0.8">
      <freejoint/>
      <geom type="box" size="0.1 0.1 0.1" mass="1.0" solref="0.005 1.5"/>
    </body>
  </worldbody>
</mujoco>
"#;

/// Init-sleep box + awake ball (for wake-on-contact).
const MJCF_WAKE_CONTACT: &str = r#"
<mujoco model="wake_contact">
  <option gravity="0 0 -9.81" timestep="0.002" sleep_tolerance="0.1">
    <flag sleep="enable"/>
  </option>
  <worldbody>
    <geom type="plane" size="5 5 0.1" solref="0.005 1.5"/>
    <body name="box" pos="0 0 0.1" sleep="init">
      <freejoint/>
      <geom type="box" size="0.1 0.1 0.1" mass="2.0" solref="0.005 1.5"/>
    </body>
    <body name="ball" pos="0 0 1.0">
      <freejoint/>
      <geom type="sphere" size="0.08" mass="0.5" solref="0.005 1.5"/>
    </body>
  </worldbody>
</mujoco>
"#;

/// Two bodies connected by equality constraint (for wake-on-equality).
const MJCF_WAKE_EQUALITY: &str = r#"
<mujoco model="wake_equality">
  <option gravity="0 0 -9.81" timestep="0.002" sleep_tolerance="0.1">
    <flag sleep="enable"/>
  </option>
  <worldbody>
    <geom type="plane" size="5 5 0.1" solref="0.005 1.5"/>
    <body name="A" pos="-0.5 0 0.1">
      <freejoint/>
      <geom type="sphere" size="0.1" mass="1.0" solref="0.005 1.5"/>
    </body>
    <body name="B" pos="0.5 0 0.1">
      <freejoint/>
      <geom type="sphere" size="0.1" mass="1.0" solref="0.005 1.5"/>
    </body>
  </worldbody>
  <equality>
    <connect body1="A" body2="B" anchor="0 0 0.1"/>
  </equality>
</mujoco>
"#;

/// Three boxes touching in a line for cascade wake.
/// Spacing = 0.2m center-to-center with 0.1 half-extent = edges touching.
const MJCF_CASCADE: &str = r#"
<mujoco model="cascade">
  <option gravity="0 0 -9.81" timestep="0.002" sleep_tolerance="0.1">
    <flag sleep="enable"/>
  </option>
  <worldbody>
    <geom type="plane" size="5 5 0.1" solref="0.005 1.5"/>
    <body name="left" pos="-0.2 0 0.1">
      <freejoint/>
      <geom type="box" size="0.1 0.1 0.1" mass="1.0" solref="0.005 1.5"/>
    </body>
    <body name="middle" pos="0 0 0.1">
      <freejoint/>
      <geom type="box" size="0.1 0.1 0.1" mass="1.0" solref="0.005 1.5"/>
    </body>
    <body name="right" pos="0.2 0 0.1">
      <freejoint/>
      <geom type="box" size="0.1 0.1 0.1" mass="1.0" solref="0.005 1.5"/>
    </body>
  </worldbody>
</mujoco>
"#;

/// Sleep policy "never".
const MJCF_NEVER: &str = r#"
<mujoco model="sleep_never">
  <option gravity="0 0 -9.81" timestep="0.002" sleep_tolerance="0.1">
    <flag sleep="enable"/>
  </option>
  <worldbody>
    <geom type="plane" size="5 5 0.1" solref="0.005 1.5"/>
    <body name="box" pos="0 0 0.3" sleep="never">
      <freejoint/>
      <geom type="box" size="0.1 0.1 0.1" mass="1.0" solref="0.005 1.5"/>
    </body>
  </worldbody>
</mujoco>
"#;

/// Two init-sleep boxes side by side (for narrowphase skip check).
const MJCF_BOTH_INIT: &str = r#"
<mujoco model="both_init">
  <option gravity="0 0 0" timestep="0.002" sleep_tolerance="0.1">
    <flag sleep="enable"/>
  </option>
  <worldbody>
    <body name="A" pos="-0.05 0 0" sleep="init">
      <freejoint/>
      <geom type="box" size="0.1 0.1 0.1" mass="1.0" margin="0.1"/>
    </body>
    <body name="B" pos="0.05 0 0" sleep="init">
      <freejoint/>
      <geom type="box" size="0.1 0.1 0.1" mass="1.0" margin="0.1"/>
    </body>
  </worldbody>
</mujoco>
"#;

// ── Helpers ───────────────────────────────────────────────────────────────

/// Step until all non-world bodies sleep, or return false after `max_steps`.
fn settle_until_asleep(
    model: &sim_core::Model,
    data: &mut sim_core::Data,
    max_steps: usize,
) -> bool {
    for _ in 0..max_steps {
        data.step(model).expect("step");
        if data.nbody_awake() == 1 {
            return true;
        }
    }
    false
}

// ── Checks ────────────────────────────────────────────────────────────────

/// Check 1: Body sleeps after velocity drops below threshold.
fn check_sleep_after_threshold() -> Check {
    let model = sim_mjcf::load_model(MJCF_SINGLE).expect("load");
    let mut data = model.make_data();

    let settled = settle_until_asleep(&model, &mut data, 5000);
    let body_id = 1;
    let state = data.sleep_state(body_id);

    Check {
        name: "Sleep after threshold",
        pass: settled && state == SleepState::Asleep,
        detail: format!("settled={settled}, state={state:?}"),
    }
}

/// Check 2: Sleeping body has qvel == 0 bitwise.
fn check_sleeping_qvel_zero() -> Check {
    let model = sim_mjcf::load_model(MJCF_SINGLE).expect("load");
    let mut data = model.make_data();
    settle_until_asleep(&model, &mut data, 5000);

    let dof_adr = model.body_dof_adr[1];
    let nv = model.body_dof_num[1];
    let all_zero = (0..nv).all(|i| data.qvel[dof_adr + i] == 0.0);

    Check {
        name: "Sleeping qvel = 0",
        pass: all_zero,
        detail: format!(
            "dofs[{dof_adr}..{}]: max |qvel| = {:.2e}",
            dof_adr + nv,
            (0..nv)
                .map(|i| data.qvel[dof_adr + i].abs())
                .fold(0.0_f64, f64::max)
        ),
    }
}

/// Check 3: Sleeping body has qacc == 0 bitwise.
fn check_sleeping_qacc_zero() -> Check {
    let model = sim_mjcf::load_model(MJCF_SINGLE).expect("load");
    let mut data = model.make_data();
    settle_until_asleep(&model, &mut data, 5000);

    let dof_adr = model.body_dof_adr[1];
    let nv = model.body_dof_num[1];
    let all_zero = (0..nv).all(|i| data.qacc[dof_adr + i] == 0.0);

    Check {
        name: "Sleeping qacc = 0",
        pass: all_zero,
        detail: format!(
            "max |qacc| = {:.2e}",
            (0..nv)
                .map(|i| data.qacc[dof_adr + i].abs())
                .fold(0.0_f64, f64::max)
        ),
    }
}

/// Check 4: Sleeping body has qfrc_applied == 0.
fn check_sleeping_qfrc_zero() -> Check {
    let model = sim_mjcf::load_model(MJCF_SINGLE).expect("load");
    let mut data = model.make_data();
    settle_until_asleep(&model, &mut data, 5000);

    let dof_adr = model.body_dof_adr[1];
    let nv = model.body_dof_num[1];
    let all_zero = (0..nv).all(|i| data.qfrc_applied[dof_adr + i] == 0.0);

    Check {
        name: "Sleeping qfrc_applied = 0",
        pass: all_zero,
        detail: format!(
            "max |qfrc_applied| = {:.2e}",
            (0..nv)
                .map(|i| data.qfrc_applied[dof_adr + i].abs())
                .fold(0.0_f64, f64::max)
        ),
    }
}

/// Check 5: Countdown takes at least MIN_AWAKE steps.
fn check_countdown_duration() -> Check {
    let model = sim_mjcf::load_model(MJCF_SINGLE).expect("load");
    let mut data = model.make_data();
    let tree = model.body_treeid[1];

    // Settle to near-sleep (velocity very low but not yet sleeping)
    for _ in 0..5000 {
        data.step(&model).expect("step");
        if data.tree_asleep[tree] >= 0 {
            break;
        }
    }

    // Reset: apply a brief kick, then let it settle again while counting
    data.xfrc_applied[1][5] = 5.0; // small upward force
    data.step(&model).expect("step");
    data.xfrc_applied[1][5] = 0.0;

    // Count steps from first sub-threshold observation to actual sleep
    let mut sub_threshold_steps = 0;
    let mut counting = false;
    for _ in 0..5000 {
        data.step(&model).expect("step");
        let timer = data.tree_asleep[tree];
        if timer < 0 && timer > -(1 + MIN_AWAKE) {
            // Timer is counting down (between -(MIN_AWAKE) and -1)
            counting = true;
        }
        if counting {
            sub_threshold_steps += 1;
        }
        if timer >= 0 {
            break;
        }
    }

    Check {
        name: "Countdown duration",
        pass: sub_threshold_steps >= MIN_AWAKE,
        detail: format!("steps={sub_threshold_steps}, MIN_AWAKE={MIN_AWAKE}"),
    }
}

/// Check 6: Contact with sleeping body wakes it.
fn check_wake_on_contact() -> Check {
    let model = sim_mjcf::load_model(MJCF_WAKE_CONTACT).expect("load");
    let mut data = model.make_data();

    let box_body = model.body_id("box").expect("box");
    let box_tree = model.body_treeid[box_body];

    // Verify box starts asleep (init policy)
    let starts_asleep = data.tree_asleep[box_tree] >= 0;

    // Step until ball hits box (free fall ~0.45s at 1m drop = ~225 steps)
    let mut woke = false;
    for _ in 0..500 {
        data.step(&model).expect("step");
        if data.tree_asleep[box_tree] < 0 {
            woke = true;
            break;
        }
    }

    Check {
        name: "Wake on contact",
        pass: starts_asleep && woke,
        detail: format!("starts_asleep={starts_asleep}, woke={woke}"),
    }
}

/// Check 7: Equality constraint coupling wakes sleeping tree.
fn check_wake_on_equality() -> Check {
    let model = sim_mjcf::load_model(MJCF_WAKE_EQUALITY).expect("load");
    let mut data = model.make_data();

    // Let both settle and sleep
    let settled = settle_until_asleep(&model, &mut data, 5000);

    let body_a = model.body_id("A").expect("A");
    let body_b = model.body_id("B").expect("B");
    let tree_a = model.body_treeid[body_a];
    let tree_b = model.body_treeid[body_b];

    // Apply force to A — should wake A, and equality constraint should wake B
    data.xfrc_applied[body_a][5] = 20.0; // upward force
    data.step(&model).expect("step");
    data.xfrc_applied[body_a][5] = 0.0;

    let a_awake = data.tree_asleep[tree_a] < 0;
    let b_awake = data.tree_asleep[tree_b] < 0;

    Check {
        name: "Wake on equality",
        pass: settled && a_awake && b_awake,
        detail: format!("settled={settled}, A_awake={a_awake}, B_awake={b_awake}"),
    }
}

/// Check 8: Applied force wakes sleeping body.
fn check_wake_on_xfrc() -> Check {
    let model = sim_mjcf::load_model(MJCF_SINGLE).expect("load");
    let mut data = model.make_data();
    settle_until_asleep(&model, &mut data, 5000);

    let tree = model.body_treeid[1];
    let was_asleep = data.tree_asleep[tree] >= 0;

    data.xfrc_applied[1][5] = 10.0; // upward force
    data.step(&model).expect("step");

    let now_awake = data.tree_asleep[tree] < 0;
    data.xfrc_applied[1][5] = 0.0;

    Check {
        name: "Wake on xfrc_applied",
        pass: was_asleep && now_awake,
        detail: format!("was_asleep={was_asleep}, now_awake={now_awake}"),
    }
}

/// Check 9: Wake cascades through contact chain.
fn check_wake_cascade() -> Check {
    let model = sim_mjcf::load_model(MJCF_CASCADE).expect("load");
    let mut data = model.make_data();

    let left = model.body_id("left").expect("left");
    let middle = model.body_id("middle").expect("middle");

    // Settle all three
    let settled = settle_until_asleep(&model, &mut data, 5000);

    // Poke the left box toward middle with a strong impulse
    data.xfrc_applied[left][3] = 500.0; // strong force in +X
    for _ in 0..10 {
        data.step(&model).expect("step");
    }
    data.xfrc_applied[left][3] = 0.0;

    // Track: did middle ever wake during the cascade?
    let mut middle_woke = false;
    for _ in 0..500 {
        data.step(&model).expect("step");
        if data.tree_asleep[model.body_treeid[middle]] < 0 {
            middle_woke = true;
            break;
        }
    }

    Check {
        name: "Wake cascade",
        pass: settled && middle_woke,
        detail: format!("settled={settled}, middle_woke={middle_woke}"),
    }
}

/// Check 10: Isolated bodies are singletons (tree_island == -1).
fn check_singleton_islands() -> Check {
    let model = sim_mjcf::load_model(MJCF_TWO_ISOLATED).expect("load");
    let mut data = model.make_data();

    let body_a = model.body_id("A").expect("A");
    let body_b = model.body_id("B").expect("B");
    let tree_a = model.body_treeid[body_a];
    let tree_b = model.body_treeid[body_b];

    // Step a few times so island discovery runs (while bodies are awake)
    for _ in 0..10 {
        data.step(&model).expect("step");
    }

    let island_a = data.tree_island[tree_a];
    let island_b = data.tree_island[tree_b];

    Check {
        name: "No island for singletons",
        pass: island_a == -1 && island_b == -1,
        detail: format!("island_A={island_a}, island_B={island_b}"),
    }
}

/// Check 11: Stacked boxes share an island.
fn check_shared_island() -> Check {
    let model = sim_mjcf::load_model(MJCF_STACKED).expect("load");
    let mut data = model.make_data();

    let bottom = model.body_id("bottom").expect("bottom");
    let top = model.body_id("top").expect("top");
    let tree_bottom = model.body_treeid[bottom];
    let tree_top = model.body_treeid[top];

    // Step until both are in contact and share an island (scan up to 2000 steps)
    let mut found_shared = false;
    let mut island_bottom = -1_i32;
    let mut island_top = -1_i32;
    for _ in 0..2000 {
        data.step(&model).expect("step");
        island_bottom = data.tree_island[tree_bottom];
        island_top = data.tree_island[tree_top];
        if island_bottom == island_top && island_bottom >= 0 {
            found_shared = true;
            break;
        }
    }

    Check {
        name: "Shared island",
        pass: found_shared,
        detail: format!("island_bottom={island_bottom}, island_top={island_top}"),
    }
}

/// Check 12: Two stacks produce nisland == 2.
fn check_island_count() -> Check {
    let model = sim_mjcf::load_model(MJCF_TWO_STACKS).expect("load");
    let mut data = model.make_data();

    // Step until we see nisland == 2 (both stacks in contact, still awake)
    let mut found_two = false;
    let mut nisland = 0;
    for _ in 0..2000 {
        data.step(&model).expect("step");
        nisland = data.nisland();
        if nisland == 2 {
            found_two = true;
            break;
        }
    }

    Check {
        name: "Island count",
        pass: found_two,
        detail: format!("nisland={nisland} (expected 2)"),
    }
}

/// Check 13: Poking one stack wakes only that stack.
fn check_selective_wake() -> Check {
    let model = sim_mjcf::load_model(MJCF_TWO_STACKS).expect("load");
    let mut data = model.make_data();

    let a1 = model.body_id("a1").expect("a1");
    let a2 = model.body_id("a2").expect("a2");
    let b1 = model.body_id("b1").expect("b1");
    let b2 = model.body_id("b2").expect("b2");

    // Settle everything
    let settled = settle_until_asleep(&model, &mut data, 5000);

    // Poke stack A
    data.xfrc_applied[a2][5] = 20.0;
    data.step(&model).expect("step");
    data.xfrc_applied[a2][5] = 0.0;

    let a1_awake = data.sleep_state(a1) == SleepState::Awake;
    let a2_awake = data.sleep_state(a2) == SleepState::Awake;
    let b1_asleep = data.sleep_state(b1) == SleepState::Asleep;
    let b2_asleep = data.sleep_state(b2) == SleepState::Asleep;

    Check {
        name: "Selective wake",
        pass: settled && a1_awake && a2_awake && b1_asleep && b2_asleep,
        detail: format!(
            "settled={settled}, a1={a1_awake}, a2={a2_awake}, b1_asleep={b1_asleep}, b2_asleep={b2_asleep}"
        ),
    }
}

/// Check 14: Velocity spike resets the countdown timer.
fn check_countdown_reset() -> Check {
    let model = sim_mjcf::load_model(MJCF_SINGLE).expect("load");
    let mut data = model.make_data();
    let tree = model.body_treeid[1];

    // Settle to sleep
    settle_until_asleep(&model, &mut data, 5000);

    // Wake it with a small force
    data.xfrc_applied[1][5] = 5.0;
    data.step(&model).expect("step");
    data.xfrc_applied[1][5] = 0.0;
    if data.tree_asleep[tree] >= 0 {
        return Check {
            name: "Countdown reset",
            pass: false,
            detail: "precondition failed: body did not wake after force".to_string(),
        };
    }

    // Step until timer starts counting down (past the full-awake reset)
    let mut saw_counting = false;
    for _ in 0..3000 {
        data.step(&model).expect("step");
        let timer = data.tree_asleep[tree];
        if timer > -(1 + MIN_AWAKE) && timer < 0 {
            saw_counting = true;
            // Now apply another velocity spike to reset
            data.xfrc_applied[1][5] = 5.0;
            data.step(&model).expect("step");
            data.xfrc_applied[1][5] = 0.0;
            break;
        }
    }

    // After reset, timer should be back to fully awake
    let timer_after = data.tree_asleep[tree];
    let reset_ok = timer_after == -(1 + MIN_AWAKE);

    Check {
        name: "Countdown reset",
        pass: saw_counting && reset_ok,
        detail: format!(
            "saw_counting={saw_counting}, timer_after={timer_after} (expect {})",
            -(1 + MIN_AWAKE)
        ),
    }
}

/// Check 15: Sleep policy "never" prevents sleeping.
fn check_policy_never() -> Check {
    let model = sim_mjcf::load_model(MJCF_NEVER).expect("load");
    let mut data = model.make_data();
    let tree = model.body_treeid[1];

    // Step a long time — body should never sleep
    for _ in 0..5000 {
        data.step(&model).expect("step");
    }

    let still_awake = data.tree_asleep[tree] < 0;

    Check {
        name: "Sleep policy Never",
        pass: still_awake,
        detail: format!("tree_asleep={} (expect < 0)", data.tree_asleep[tree]),
    }
}

/// Check 16: Clearing ENABLE_SLEEP prevents all sleeping.
fn check_sleep_disabled() -> Check {
    let mut model = sim_mjcf::load_model(MJCF_SINGLE).expect("load");
    model.enableflags &= !ENABLE_SLEEP; // Disable sleep
    let mut data = model.make_data();

    for _ in 0..5000 {
        data.step(&model).expect("step");
    }

    // All bodies should remain awake (nbody_awake == nbody)
    let all_awake = data.nbody_awake() == model.nbody;

    Check {
        name: "ENABLE_SLEEP disabled",
        pass: all_awake,
        detail: format!(
            "nbody_awake={}, nbody={} (expect equal)",
            data.nbody_awake(),
            model.nbody
        ),
    }
}

/// Check 17: Narrowphase skips contacts between two sleeping bodies.
fn check_narrowphase_skip() -> Check {
    let model = sim_mjcf::load_model(MJCF_BOTH_INIT).expect("load");
    let mut data = model.make_data();

    let body_a = model.body_id("A").expect("A");
    let body_b = model.body_id("B").expect("B");

    // Both bodies should start asleep (init policy)
    let a_asleep = data.sleep_state(body_a) == SleepState::Asleep;
    let b_asleep = data.sleep_state(body_b) == SleepState::Asleep;

    // Step — narrowphase should skip the sleeping pair
    data.step(&model).expect("step");

    // Count contacts between A and B specifically
    let mutual_contacts: usize = data.contacts[..data.ncon]
        .iter()
        .filter(|c| {
            let g1_body = model.geom_body[c.geom1];
            let g2_body = model.geom_body[c.geom2];
            (g1_body == body_a && g2_body == body_b) || (g1_body == body_b && g2_body == body_a)
        })
        .count();

    Check {
        name: "Narrowphase skip",
        pass: a_asleep && b_asleep && mutual_contacts == 0,
        detail: format!(
            "a_asleep={a_asleep}, b_asleep={b_asleep}, mutual_contacts={mutual_contacts}"
        ),
    }
}

/// Check 18: nbody_awake matches manual count of non-Asleep bodies.
fn check_nbody_awake_bookkeeping() -> Check {
    let model = sim_mjcf::load_model(MJCF_TWO_ISOLATED).expect("load");
    let mut data = model.make_data();

    // Check in three states: initial (all awake), mid-settle, and fully asleep
    let mut all_match = true;
    let mut detail = String::new();

    for phase in 0..3 {
        let steps = match phase {
            0 => 1,
            1 => 500,
            _ => 5000,
        };
        for _ in 0..steps {
            data.step(&model).expect("step");
        }

        let manual_count = (0..model.nbody)
            .filter(|&b| data.body_sleep_state[b] != SleepState::Asleep)
            .count();
        let reported = data.nbody_awake();

        if manual_count != reported {
            all_match = false;
            detail = format!("phase {phase}: manual={manual_count}, reported={reported}");
            break;
        }
    }

    if all_match {
        detail = "all 3 phases match".to_string();
    }

    Check {
        name: "nbody_awake bookkeeping",
        pass: all_match,
        detail,
    }
}

// ── Main ──────────────────────────────────────────────────────────────────

fn main() {
    println!("=== CortenForge: Sleep/Wake Stress Test ===\n");

    let checks = vec![
        check_sleep_after_threshold(),
        check_sleeping_qvel_zero(),
        check_sleeping_qacc_zero(),
        check_sleeping_qfrc_zero(),
        check_countdown_duration(),
        check_wake_on_contact(),
        check_wake_on_equality(),
        check_wake_on_xfrc(),
        check_wake_cascade(),
        check_singleton_islands(),
        check_shared_island(),
        check_island_count(),
        check_selective_wake(),
        check_countdown_reset(),
        check_policy_never(),
        check_sleep_disabled(),
        check_narrowphase_skip(),
        check_nbody_awake_bookkeeping(),
    ];

    let all_passed = print_report("Sleep/Wake (18 checks)", &checks);

    if !all_passed {
        std::process::exit(1);
    }
}
